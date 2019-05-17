#include "fpsInterpolator.h"
#include <windows.h>
#include <gl/GL.h>

#include <queue>

FpsInterpolator interpolator;

#define SETGLMODE(functionCall, newValue, persistentValue) if ((persistentValue) != (newValue)) {(functionCall); persistentValue = newValue; }
#define SETCOL(x)  if(x.col!=ulOLDCOL) {ulOLDCOL=x.col;glColor4ubv((GLubyte*)&x.col);} 
#define SETPCOL(x)  if(x->c.lcol!=ulOLDCOL) {ulOLDCOL=x->c.lcol;glColor4ubv(x->c.col);}

#define TOREGION(x, y) ( (((((y) / regionSplitY) % regionSplits) + regionSplits) % regionSplits) * regionSplits + (((((x) / regionSplitX) % regionSplits) + regionSplits) % regionSplits))

extern "C" {
	void CallPrimFunc(int gpuCommand, unsigned char* data); 
	void resetGpuThing();
}

void FpsInterpolator::Render() {
	const bool doClear = true;
	const bool doInterpolation = true;
	const int numInterpolations = 2;

	if (currentFrameDraws.size() == 0) {
		return;
	}

	// Update entities
	currentEntities = IsolateEntities(&currentFrameDraws[0], currentFrameDraws.size());

	// Identify interpolatable vertices
	std::vector<PolyMatch> matches = 
		(previousFrameDraws.size() > 0 && currentFrameDraws.size() > 0 && doInterpolation) ? 
			MatchVertices(&previousFrameDraws[0], previousFrameDraws.size(), &currentFrameDraws[0], currentFrameDraws.size()) : std::vector<PolyMatch>();

	// go through matches, determining the most common entity connections
	// find the most confident matches

	// ok, now be amazing and shit, and go through the entity's neighbours matching everything up nicely
	std::vector<const DrawCommand*> bestMatches;
	if (previousEntities.size() > 0 && false) {
		std::queue<const DrawCommand*> connectedPolys;
		bool* closed = new bool[previousFrameDraws.size()];
		PolyMatch* bestMatch = nullptr;

		memset(closed, 0, sizeof(bool) * previousFrameDraws.size());

		do {
			// Find the best starting point for this entity
			int bestSimilarity = 99999;
			bestMatch = nullptr;

			for (int i = 0; i < matches.size(); i++) {
				if (!closed[i] && matches[i].divergence < bestSimilarity && matches[i].polyA != matches[i].polyB) {
					bestMatch = &matches[i];
					bestSimilarity = matches[i].divergence;
				}
			}

			if (bestMatch) {
				// Connet this and the next frame's neighbours equivalently
				closed[bestMatch->polyA - previousFrameDraws.data()] = true;
				connectedPolys.push(bestMatch->polyA);

				while (!connectedPolys.empty()) {
					const DrawCommand* previous = connectedPolys.front();
					const DrawCommand* next = matches[previous - previousFrameDraws.data()].polyB;
					const DrawCommand** previousNeighbours = previousEntities[previous - previousFrameDraws.data()].neighbours;
					const DrawCommand** nextNeighbours = currentEntities[next - currentFrameDraws.data()].neighbours;

					for (int n = 0; n < 4; n++) {
						if (previousNeighbours[n] != nullptr && nextNeighbours[n] != nullptr && !closed[previousNeighbours[n] - previousFrameDraws.data()]) {
							connectedPolys.push(previousNeighbours[n]);

							matches[previousNeighbours[n] - previousFrameDraws.data()].polyB = nextNeighbours[n];
							closed[previousNeighbours[n] - previousFrameDraws.data()] = true;
						}
					}

					connectedPolys.pop();
				}

				bestMatches.push_back(bestMatch->polyA);
			}
		} while (bestMatch);
		
		delete[] closed;
	}

	// Begin render
	bool doRenderFrame = true;
	int numFramesRendered = 0;

	static int targetFrameDuration = 1000/25;

	if (HIWORD(GetKeyState(VK_RIGHT))) {
		targetFrameDuration -= 16;
	}
	if (HIWORD(GetKeyState(VK_LEFT))) {
		targetFrameDuration += 16;
	}
	if (HIWORD(GetKeyState(VK_DOWN))) {
		targetFrameDuration = 16;
	}
	if (targetFrameDuration <= 0) {
		targetFrameDuration = 1;
	}

	float blendFactor = 0.0f;
	do {
		bool doFreezeFrame = HIWORD(GetAsyncKeyState(VK_DOWN));
		blendFactor = (timeGetTime() - currentFrameTime) / (float)targetFrameDuration;

		// Clear the screen
		if (doClear) {
			glClearColor(1, 0, 0, 1);
			glClear(GL_COLOR_BUFFER_BIT);
		}

		// Freeze frame
		if (doFreezeFrame) {
			blendFactor = sin((timeGetTime() - currentFrameTime) / (float)1000.0f * 6.28f) / 2.0f + 0.5f;
			if (blendFactor >= 1.0f) blendFactor = 0.99f;
		}

		if (doInterpolation /*&& blendFactor < 1.0f*/) {
			// wait for the next frame
			/*while (timeGetTime() - previousFrameTime < (int)(targetFrameDuration * blendFactor)) {
				continue;
			}*/

			if (blendFactor > 1) blendFactor = 1;

			DrawBlendedCommands(matches, blendFactor);
			DrawSolidPolygons(bestMatches, 0xFF00FF00);
			DrawMotionVectors(matches);
			//DrawEntities(currentEntities, currentFrameDraws.data(), currentFrameDraws.size());
		}

		// Present to screen
		SwapBuffers(wglGetCurrentDC());
	} while (blendFactor < 1.0f && doInterpolation);

	// Replace the command lists
	previousFrameDraws = std::move(currentFrameDraws);
	previousEntities = std::move(currentEntities);
	currentFrameDraws.clear();
	currentEntities.clear();
	
	// Update timings
	previousFrameTime = currentFrameTime;
	currentFrameTime = timeGetTime();
}

void FpsInterpolator::DrawBlendedCommands(const std::vector<PolyMatch>& matches, float blendFactor) {
	GLuint currentTex = 0;
	unsigned long currentColour = 0;
	bool currentSmooth = false;
	bool currentBlend = false;
	bool doRenderSimilarities = false;
	int renderSimilarityMax = 0;

	if (doRenderSimilarities && matches.size() > 0) {
		// find the maximum similarity
		std::vector<PolyMatch> copy = matches;
		std::sort(copy.begin(), copy.end(), [](PolyMatch& a, PolyMatch& b) { return a.divergence < b.divergence; });
		renderSimilarityMax = copy[copy.size() * 4 / 5].divergence;//std::max_element(matches.begin(), matches.end(), [](const VertexMatch& a, const VertexMatch& b) -> bool {return a.similarity < b.similarity; })->similarity;
		if (renderSimilarityMax == 0) {
			renderSimilarityMax = 1; // no-divide-by-zero hack
		}
	}

	// Setup the render mode
	glShadeModel(GL_FLAT);
	glDisable(GL_BLEND);
	glBindTexture(GL_TEXTURE_2D, currentTex);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glColor4ubv((GLubyte*)&currentColour);

	isRendering = true;

	// Render each interpolatable draw command
	for (DrawCommand& command : previousFrameDraws) {
		const PolyMatch& match = matches[&command - &previousFrameDraws[0]];
		const DrawCommand* nextCommand = match.polyB;

		// set texture
		if (currentTex != nextCommand->texture) {
			if (currentTex == -1) {
				glEnable(GL_TEXTURE_2D);
			} else if (nextCommand->texture == -1) {
				glDisable(GL_TEXTURE_2D);
			}

			currentTex = nextCommand->texture;
			glBindTexture(GL_TEXTURE_2D, currentTex);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		}

		// set shading mode
		SETGLMODE(glShadeModel(nextCommand->isSmoothShaded ? GL_SMOOTH : GL_FLAT), nextCommand->isSmoothShaded, currentSmooth);

		// set blend mode
		SETGLMODE(currentBlend ? glEnable(GL_BLEND) : glDisable(GL_BLEND), nextCommand->isBlended, currentBlend);

		// start poly render
		if (command.numVertices == 4) {
			glBegin(GL_TRIANGLE_STRIP);
		} else {
			glBegin(GL_TRIANGLES);
		}

		// blend vertices and render
		static const int vertexOrderQuad[4] = { 0, 1, 3, 2 };
		static const int vertexOrderTri[3] = { 0, 1, 2 };

		const int* vertexOrder = command.numVertices == 4 ? vertexOrderQuad : vertexOrderTri;

		for (int _v = 0; _v < command.numVertices; _v++) {
			int v = vertexOrder[_v];
			Vertex blended = Vertex(command.vertices[v], nextCommand->vertices[v], blendFactor);
			unsigned int vertexColour = nextCommand->vertices[v].col;

			if (doRenderSimilarities) {
				vertexColour = ((int)match.divergence * 0xFF / renderSimilarityMax); // red means least similar because I named the variable backwards lol
				if (vertexColour > 0xFF) {
					vertexColour = 0xFF;
				}
				vertexColour = (0xFF - vertexColour) << 16 | vertexColour | 0xFF007F00;
			}

			SETGLMODE(glColor4ubv((GLubyte*)&vertexColour), vertexColour, currentColour);
			glTexCoord2fv((GLfloat*)&nextCommand->vertices[v].sow); // test
			glVertex3fv((GLfloat*)&blended.x);
		}

		glEnd();
	}

	isRendering = false;
}

void FpsInterpolator::DrawMotionVectors(const std::vector<PolyMatch>& matches) {
	GLuint currentColour = 0xFF000000;

	glBegin(GL_LINES);

	glColor4ubv((GLubyte*)&currentColour);

	for (int i = 0; i < matches.size(); i++) {
		const DrawCommand& command = *matches[i].polyA;
		const DrawCommand& nextCommand = *matches[i].polyB;

		// draw motion vector betwen centres of the polygons
		float coords[6] = { 0 };

		for (int v = 0; v < command.numVertices; v++) {
			coords[0] += command.vertices[v].x;
			coords[1] += command.vertices[v].y;
			coords[3] += nextCommand.vertices[v].x;
			coords[4] += nextCommand.vertices[v].y;
		}
		coords[0] /= command.numVertices;
		coords[1] /= command.numVertices;
		coords[3] /= nextCommand.numVertices;
		coords[4] /= nextCommand.numVertices;

		glVertex3fv((GLfloat*)coords);
		glVertex3fv((GLfloat*)&coords[3]);
	}

	glEnd();
}

void FpsInterpolator::DrawSolidPolygons(const std::vector<const DrawCommand*> commands, GLuint colour) {
	if (commands.size() == 0) {
		return;
	}

	glDisable(GL_TEXTURE_2D);
	glShadeModel(GL_FLAT);
	glColor4ubv((GLubyte*)&colour);

	for (const DrawCommand* const* _command = commands.data(); _command < &commands.data()[commands.size()]; _command++) {
		const DrawCommand* command = *_command;
		// start poly render
		if (command->numVertices == 4) {
			glBegin(GL_TRIANGLE_STRIP);
		} else {
			glBegin(GL_TRIANGLES);
		}

		// blend vertices and render
		static const int vertexOrderQuad[4] = { 0, 1, 3, 2 };
		static const int vertexOrderTri[3] = { 0, 1, 2 };

		const int* vertexOrder = command->numVertices == 4 ? vertexOrderQuad : vertexOrderTri;

		for (int _v = 0; _v < command->numVertices; _v++) {
			int v = vertexOrder[_v];

			glTexCoord2fv((GLfloat*)&command->vertices[v].sow);
			glVertex3fv((GLfloat*)&command->vertices[v].x);
		}

		glEnd();
	}
}

std::vector<FpsInterpolator::NeighbourLinks> FpsInterpolator::IsolateEntities(const DrawCommand* commands, int numCommands) {
	// Collect neighbour references into each vertex
	const int regionSplits = 16;
	const int regionWidth = 800, regionHeight = 700;
	int regionSplitX = regionWidth / regionSplits, regionSplitY = regionHeight / regionSplits;
	const DrawCommand** regions[regionSplits * regionSplits];
	int regionSizes[regionSplits * regionSplits] = { 0 };
	std::vector<NeighbourLinks> links;

	// Initialise regions
	for (int i = 0; i < regionSplits * regionSplits; i++) {
		regions[i] = new const DrawCommand*[numCommands * 4];
	}

	// Initialise links and region map
	links.reserve(numCommands);

	for (const DrawCommand* command = commands; command < &commands[numCommands]; command++) {
		NeighbourLinks link;

		link.entity = 0;
		link.neighbours[0] = link.neighbours[1] = link.neighbours[2] = link.neighbours[3] = nullptr;
		links.push_back(link);

		for (int v = 0; v < command->numVertices; v++) {
			int region = TOREGION(command->vertices[v].iX, command->vertices[v].iY);

			regions[region][regionSizes[region]++] = command;
		}
	}

	// Find neighbour matches
	for (const DrawCommand* command = commands; command < &commands[numCommands]; command++) {
		NeighbourLinks& link = links[command - commands];

		for (int v1 = 0; v1 < command->numVertices; v1++) {
			const Vertex *v1a = &command->vertices[v1], *v1b = &command->vertices[(v1 + 1) % command->numVertices];
			int region = TOREGION(v1a->iX, v1a->iY);

			for (const DrawCommand** _other = regions[region]; _other < &regions[region][regionSizes[region]]; _other++) {
				const DrawCommand* other = *_other;

				if (other == command) {
					continue;
				}

				for (int v2 = v1; v2 < other->numVertices; v2++) {
					const Vertex* v2a = &other->vertices[v2], *v2b = &other->vertices[(v2 + 1) % other->numVertices];

					if (v1a->iX == v2a->iX && v1a->iY == v2a->iY && v1b->iX == v2b->iX && v1b->iY == v2b->iY) {
						link.neighbours[v1] = other;
						links[other - commands].neighbours[v2] = command;
					}
					else if (v1a->iX == v2b->iX && v1a->iY == v2b->iY && v1b->iX == v2a->iX && v1b->iY == v2a->iY) {
						link.neighbours[v1] = other;
						links[other - commands].neighbours[v2] = command;
					}
				}
			}
		}
	}

	// Sort links into entities
	int currentEntity = 1;

	for (NeighbourLinks* link = links.data(); link < &links.data()[numCommands]; link++) {
		if (link->entity == 0) {
			// Begin fill search into neighbours and set their entity
			std::queue<NeighbourLinks*> searchLinks;

			link->entity = currentEntity;
			searchLinks.push(link);

			while (searchLinks.size() > 0) {
				NeighbourLinks* next = searchLinks.front();

				for (int n = 0; n < 4; n++) {
					if (next->neighbours[n] != nullptr && links[next->neighbours[n] - commands].entity == 0) {
						links[next->neighbours[n] - commands].entity = currentEntity;
						searchLinks.push(&links[next->neighbours[n] - commands]);
					}
				}

				searchLinks.pop();
			}

			// move on to the next
			currentEntity++;
		}
	}

	// Cleanup
	for (int i = 0; i < regionSplits * regionSplits; i++) {
		delete[] regions[i];
	}

	return links;
}

void FpsInterpolator::DrawEntities(const std::vector<NeighbourLinks>& links, const DrawCommand* commands, int numCommands) {
	const bool doRenderEdges = true;
	const bool doRenderFaces = false;
	GLuint entityColours[4] = { 0xFF0000FF, 0xFF00FF00, 0xFFFF0000, 0xFFFFFFFF };
	GLuint black = 0xFF000000;

	if (doRenderFaces) {
		glDisable(GL_TEXTURE_2D);

		// Render entity faces
		static const int vertexOrderQuad[4] = { 0, 1, 3, 2 };
		static const int vertexOrderTri[3] = { 0, 1, 2 };

		for (int i = 0; i < numCommands; i++) {
			const int* vertexOrder = commands[i].numVertices == 4 ? vertexOrderQuad : vertexOrderTri;

			glBegin(GL_TRIANGLE_STRIP);

			for (int v = 0; v < commands[i].numVertices; v++) {
				glColor4ubv((GLubyte*)&entityColours[links[i].entity & 3]);
				glVertex3fv((GLfloat*)&commands[i].vertices[vertexOrder[v]].x);
			}

			glEnd();
		}
	}

	if (doRenderEdges) {
		glDisable(GL_TEXTURE_2D);

		// Render entity edges
		for (int i = 0; i < numCommands; i++) {
			for (int v = 0; v < commands[i].numVertices; v++) {
				if (links[i].neighbours[v] != NULL) {
					glBegin(GL_LINES);

					glColor4ubv((GLubyte*)&entityColours[links[i].entity & 3]);
					glVertex3fv((GLfloat*)&commands[i].vertices[v].x);
					glVertex3fv((GLfloat*)&commands[i].vertices[(v + 1) % commands[i].numVertices].x);

					glEnd();
				}
			}
		}
	}
}

inline float Distance(float aX, float aY, float bX, float bY) {
	return sqrt((aX - bX * aX - bX) + (aY - bY * aY - bY));
}

int matchProcessTime = 0;
int regionProcessTime = 0;

std::vector<FpsInterpolator::PolyMatch> FpsInterpolator::MatchVertices(const DrawCommand* commandsA, int numCommandsA, const DrawCommand* commandsB, int numCommandsB) {
	const int regionWidth = 800, regionHeight = 700;
	const int regionSplits = 6;
	const int numRegions = regionSplits * regionSplits;
	const int regionSplitX = regionWidth / regionSplits, regionSplitY = regionHeight / regionSplits;
	const int regionOverlap = 1;
	std::vector<const DrawCommand*> regionsA[numRegions], regionsB[numRegions];
	std::vector<PolyMatch> matches;

	// Prealloc regions
	for (int i = 0; i < numRegions; i++) {
		regionsB[i].reserve(500);
	}

	// Assign regions in vertsB
	DWORD time = timeGetTime();
	int globalSmallestX = 99999, globalSmallestY = 999999, globalLargestX = -99999, globalLargestY = -99999;
	for (const DrawCommand* b = commandsB; b < &commandsB[numCommandsB]; b++) {
		// refresh globalSmallestX
		if (b->minX < globalSmallestX) globalSmallestX = b->minX;
		if (b->minY < globalSmallestY) globalSmallestY = b->minY;
		if (b->maxX > globalLargestX) globalLargestX = b->maxX;
		if (b->maxY > globalLargestY) globalLargestY = b->maxY;

		// add it into overlapping regions
		for (int y = b->minY - regionOverlap * regionSplitY; y <= b->minY + regionOverlap * regionSplitY; y += regionSplitY) {
			for (int x = b->minX - regionOverlap * regionSplitX; x <= b->minX + regionOverlap * regionSplitX; x += regionSplitX) {
				regionsB[TOREGION(x, y)].push_back(b);
			}
		}
	}

	// record timestamps
	regionProcessTime = timeGetTime() - time;
	time = timeGetTime();
	matches.reserve(numCommandsA);

	// search for matches
	int* numBPolyMatches = new int[numCommandsB]();

	for (const DrawCommand* a = commandsA; a < &commandsA[numCommandsA]; a++) {
		PolyMatch match;
		int bestDivergence = 9999999;

		match.polyA = a;
		match.polyB = a;

		// Find the most similar vertex
		int aRegion = TOREGION(a->minX, a->minY);

		for (const DrawCommand* b : regionsB[aRegion]) {
			// ignore polys with mismatching side counts
			if (b->numVertices != a->numVertices) {
				continue;
			}

			int divergence = a->CalculateDivergence(*b);

			if (divergence < bestDivergence) {
				bestDivergence = divergence;
				match.polyB = b;
				match.divergence = divergence;
			}
		}

		// count the number of times this has been matched
		if (match.polyB != a) {
			numBPolyMatches[match.polyB - commandsB]++;
		}

		matches.push_back(match);
	}

	// Find over-matched polygons and identify stuff that make it not do that anymore...
	for (int i = 0; i < numCommandsB; i++) {
		if (numBPolyMatches[i] > 1) {
			// find them again (move this?)
			std::vector<PolyMatch*> aPolys;

			for (int j = 0; j < numCommandsA; j++) {
				if (matches[j].polyB == &commandsB[i]) {
					aPolys.push_back(&matches[j]);
				}
			}

			// allow the closest match to do the thingy
			int bestDivergence = 999999;
			int bestDiverger = 0;

			for (int p = 0; p < numBPolyMatches[i]; p++) {
				if (aPolys[p]->divergence < bestDivergence) {
					bestDiverger = p;
					bestDivergence = aPolys[p]->divergence;
				}
			}

			// now everything that *isn't* the closest match will be given the closest alternative
			for (int p = 0; p < numBPolyMatches[i]; p++) {
				if (p != bestDiverger) {
					int bestRematchDivergence = 99999;
					int bestRematch = 0;

					for (int m = 0; m < numCommandsB; m++) {
						if (numBPolyMatches[m] == 0 && commandsB[m].numVertices == aPolys[p]->polyA->numVertices) {
							int divergence = aPolys[p]->polyA->CalculateDivergence(commandsB[m]);

							if (divergence < bestRematchDivergence) {
								bestRematchDivergence = divergence;
								bestRematch = m;
							}
						}
					}

					numBPolyMatches[bestRematch]++;
					aPolys[p]->divergence = bestRematchDivergence;
					aPolys[p]->polyB = &commandsB[bestRematch];
				}
			}
		}
	}

	matchProcessTime = timeGetTime() - time;

	delete[] numBPolyMatches;

	return matches;
}

void FpsInterpolator::RecordDrawCommand(const DrawCommand& command) {
	if (!isRendering) {
		currentFrameDraws.push_back(command);
	}
}

void FpsInterpolator::RecordPsxCommand(int gpuCommand, const unsigned char* data) {
	if (!isRendering) {
		PsxCommand cmd;
		const int wtf = sizeof(PsxCommand);

		cmd.gpuCommand = gpuCommand;
		memcpy(cmd.data, data, 256);

		//currentCommands.push_back(cmd);
	}
}

void FpsInterpolator::RegisterFrame()
{
	currentFrame++;
}

void FpsInterpolator::SetFrameRate(int framerate) {
	this->framerate = framerate;
}


/* C functions */
extern "C" {
#include "stdafx.h"
#include "externals.h"

	int currentPsxCommandIndex = 0;

	FpsInterpolator::Vertex OGLToVertex(OGLVertex* ogl) {
		FpsInterpolator::Vertex vert;

		vert.x = ogl->x;
		vert.y = ogl->y;
		vert.z = ogl->z;
		vert.iX = (int)ogl->x;
		vert.iY = (int)ogl->y;
		vert.iZ = (int)ogl->z;
		vert.sow = ogl->sow;
		vert.tow = ogl->tow;
		vert.col = ogl->c.lcol;
		return vert;
	}

	void interpolatorRecord(OGLVertex* a, OGLVertex* b, OGLVertex* c, OGLVertex* d, int texture, int bDrawSmoothShaded, int bBlend) {
		if (d == NULL) {
			interpolator.RecordDrawCommand(FpsInterpolator::DrawCommand(OGLToVertex(a), OGLToVertex(b), OGLToVertex(c), texture, bDrawSmoothShaded == 0 ? false : true, bBlend == 0 ? false : true));
		}
		else {
			interpolator.RecordDrawCommand(FpsInterpolator::DrawCommand(OGLToVertex(a), OGLToVertex(b), OGLToVertex(c), OGLToVertex(d), texture, bDrawSmoothShaded == 0 ? false : true, bBlend == 0 ? false : true));
		}
	}

	int interpolatorRecordPsx(int gpuCommand, unsigned char* data) {
		//interpolator.RecordPsxCommand(gpuCommand, data);

		// return 0 to do original call
		return 0;
	}

	void interpolatorUpdateDisplay() {
		interpolator.Render();
	}

	void CallPrimFunc(int gpuCommand, unsigned char* data) {
		primTableJ[gpuCommand](data);
	}

	void initInterpolator() {
		interpolator = FpsInterpolator();
	}

	void resetGpuThing() {
		//PSXDisplay.DrawOffset.x = PSXDisplay.DrawOffset.y = 0;
	}

	int interpolatorManageFramecap(int framerate) {
		interpolator.RegisterFrame();
		interpolator.SetFrameRate(framerate);
		return 1;
	}
}