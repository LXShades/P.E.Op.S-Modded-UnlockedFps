#include "fpsInterpolator.h"
#include <windows.h>
#include <gl/GL.h>

FpsInterpolator interpolator;

#define SETGLMODE(functionCall, newValue, persistentValue) if ((persistentValue) != (newValue)) {(functionCall); persistentValue = newValue; }
#define SETCOL(x)  if(x.col!=ulOLDCOL) {ulOLDCOL=x.col;glColor4ubv((GLubyte*)&x.col);} 
#define SETPCOL(x)  if(x->c.lcol!=ulOLDCOL) {ulOLDCOL=x->c.lcol;glColor4ubv(x->c.col);}

extern "C" {
	void CallPrimFunc(int gpuCommand, unsigned char* data); 
	void resetGpuThing();
}

void FpsInterpolator::Render() {
	const bool doClear = true;
	const bool doInterpolation = true;
	const int numInterpolations = 2;

	// Clear the screen
	if (doClear) {
		glClearColor(1, 0, 0, 1);
		glClear(GL_COLOR_BUFFER_BIT);
	}

	// Identify interpolatable vertices
	std::vector<VertexMatch> matches = 
		(previousFrameDraws.size() > 0 && currentFrameDraws.size() > 0 && doInterpolation) ? 
			MatchVertices(&previousFrameDraws[0], previousFrameDraws.size(), &currentFrameDraws[0], currentFrameDraws.size()) : std::vector<VertexMatch>();

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
		blendFactor = (timeGetTime() - currentFrameTime) / (float)targetFrameDuration;

		if (doInterpolation /*&& blendFactor < 1.0f*/) {
			// wait for the next frame
			/*while (timeGetTime() - previousFrameTime < (int)(targetFrameDuration * blendFactor)) {
				continue;
			}*/

			if (blendFactor > 1) blendFactor = 1;

			DrawBlendedCommands(matches, blendFactor);
		}

		if (currentFrameDraws.size()) {
			RenderEntities(&currentFrameDraws[0], currentFrameDraws.size());
		}

		// Present to screen
		SwapBuffers(wglGetCurrentDC());
	} while (blendFactor < 1.0f && doInterpolation);

	// Replace the command lists
	previousFrameDraws = currentFrameDraws;
	currentFrameDraws.clear();

	currentCommands.clear();
	
	// Update timings
	previousFrameTime = currentFrameTime;
	currentFrameTime = timeGetTime();
}

void FpsInterpolator::DrawBlendedCommands(const std::vector<VertexMatch>& matches, float blendFactor) {
	GLuint currentTex = 0;
	unsigned long currentColour = 0;
	bool currentSmooth = false;
	bool currentBlend = false;

	// Setup the render mode
	glShadeModel(GL_FLAT);
	glDisable(GL_BLEND);
	glBindTexture(GL_TEXTURE_2D, currentTex);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glColor4ubv((GLubyte*)&currentColour);

	isRendering = true;

	// Render each interpolatable draw command
	for (DrawCommand& command : previousFrameDraws) {
		const VertexMatch& match = matches[&command - &previousFrameDraws[0]];
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
			SETGLMODE(glColor4ubv((GLubyte*)&nextCommand->vertices[v].col), nextCommand->vertices[v].col, currentColour);
			glTexCoord2fv((GLfloat*)&nextCommand->vertices[v].sow); // test
			glVertex3fv((GLfloat*)&blended.x);
		}

		glEnd();

		// draw motion vectors
		glBegin(GL_LINE_STRIP);
		float coords[6] = { 0 };

		currentColour = 0xFF000000;
		glColor4ubv((GLubyte*)&currentColour);

		for (int v = 0; v < command.numVertices; v++) {
			coords[0] += command.vertices[v].x;
			coords[1] += command.vertices[v].y;
			coords[3] += nextCommand->vertices[v].x;
			coords[4] += nextCommand->vertices[v].y;
		}
		coords[0] /= command.numVertices;
		coords[1] /= command.numVertices;
		coords[3] /= command.numVertices;
		coords[4] /= command.numVertices;


		glVertex3fv((GLfloat*)coords);
		glVertex3fv((GLfloat*)&coords[3]);

		glEnd();
	}

	isRendering = false;
}

struct NeighbourLinks {
	const FpsInterpolator::DrawCommand* neighbours[4];

	struct Border {
		float aX, aY, aZ;
		float bX, bY, bZ;
	} borders[4];
};

#define TOREGION(x, y) ((((y) / regionHeight * regionWidth + (x) / regionWidth) % (regionSplits * regionSplits)) + regionSplits * regionSplits) % (regionSplits * regionSplits)

void FpsInterpolator::RenderEntities(const DrawCommand* commands, int numCommands) {
	// Collect neighbour references into each vertex
	const int regionSplits = 16;
	int regionWidth = 640 / regionSplits, regionHeight = 640 / regionSplits;
	const DrawCommand** regions[regionSplits * regionSplits];
	int regionSizes[regionSplits * regionSplits] = { 0 };
	NeighbourLinks* links = new NeighbourLinks[numCommands];
	
	// Initialise helpers
	memset(links, 0, numCommands * sizeof(links[0]));
	for (int i = 0; i < regionSplits * regionSplits; i++) {
		regions[i] = new const DrawCommand*[numCommands * 4];
	}

	// Fill region map
	for (const DrawCommand* command = commands; command < &commands[numCommands]; command++) {
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
						link.borders[v1].aX = v1a->iX;
						link.borders[v1].aY = v1a->iY;
						link.borders[v1].bX = v1b->iX;
						link.borders[v1].bY = v1b->iY;
						links[other - commands].neighbours[v2] = command;
						link.borders[v2].aX = v1a->iX;
						link.borders[v2].aY = v1a->iY;
						link.borders[v2].bX = v1b->iX;
						link.borders[v2].bY = v1b->iY;
					}
					else if (v1a->iX == v2b->iX && v1a->iY == v2b->iY && v1b->iX == v2a->iX && v1b->iY == v2a->iY) {
						link.neighbours[v1] = other;
						link.borders[v1].aX = v1b->iX;
						link.borders[v1].aY = v1b->iY;
						link.borders[v1].bX = v1a->iX;
						link.borders[v1].bY = v1a->iY;
						links[other - commands].neighbours[v2] = command;
						link.borders[v2].aX = v1b->iX;
						link.borders[v2].aY = v1b->iY;
						link.borders[v2].bX = v1a->iX;
						link.borders[v2].bY = v1a->iY;
					}
				}
			}
		}
	}

	// Render neighbour edges
	GLuint currentColour = 0xFF00FF00;

	for (int i = 0; i < numCommands; i++) 
	{
		for (int v = 0; v < commands[i].numVertices; v++) {
			if (links[i].neighbours[v] != NULL) {
				glBegin(GL_LINES);

				glColor4ubv((GLubyte*)&currentColour);
				glVertex3fv((GLfloat*)&links[i].borders[v].aX);
				glVertex3fv((GLfloat*)&links[i].borders[v].bX);

				glEnd();
			}
		}
	}

	// Cleanup
	for (int i = 0; i < regionSplits * regionSplits; i++) {
		delete[] regions[i];
	}
	delete[] links;
}

inline float Distance(float aX, float aY, float bX, float bY) {
	return sqrt((aX - bX * aX - bX) + (aY - bY * aY - bY));
}

#define XYTOREGION(x, y) (((((y) / regionSplitY * numSplits + (x) / regionSplitX) % numRegions) + numRegions) % numRegions)

int matchProcessTime = 0;
int regionProcessTime = 0;

std::vector<FpsInterpolator::VertexMatch> FpsInterpolator::MatchVertices(const DrawCommand* vertsA, int numVertsA, const DrawCommand* vertsB, int numVertsB) {
	const float regionWidth = 640, regionHeight = 512;
	const int numSplits = 4;
	const int numRegions = numSplits * numSplits;
	const int regionSplitX = regionWidth / numSplits, regionSplitY = regionHeight / numSplits;
	const int regionOverlap = 1;
	std::vector<const DrawCommand*> regionsA[numRegions], regionsB[numRegions];
	std::vector<VertexMatch> matches;
	const float distanceWeight = 1.0f;

	// Prealloc regions
	for (int i = 0; i < numRegions; i++) {
		regionsB->reserve(300);
	}

	// Fill regions in vertsB
	DWORD time = timeGetTime();
	for (const DrawCommand* b = vertsB; b < &vertsB[numVertsB]; b++) {
		// find smallest coordinates in vertices
		int smallestX = 9999, smallestY = 9999;
		
		for (int v = 0; v < b->numVertices; v++) {
			if (b->vertices[v].iX < smallestX) {
				smallestX = b->vertices[v].iX;
			} else if (b->vertices[v].iY < smallestY) {
				smallestY = b->vertices[v].iY;
			}
		}

		if (smallestX < 0) smallestX = 0;
		if (smallestY < 0) smallestY = 0;
		if (smallestX >= regionWidth) smallestX = regionWidth - 1;
		if (smallestY >= regionHeight) smallestY = regionHeight - 1;

		// add it into overlapping regions
		for (int y = smallestY / regionSplitY - regionOverlap; y <= smallestY / regionSplitY + regionOverlap; y++) {
			if (y < 0 || y >= numSplits) {
				continue;
			}

			for (int x = smallestX / regionSplitX - regionOverlap; x <= smallestX / regionSplitX + regionOverlap; x++) {
				if (x < 0 || x >= numSplits) {
					continue;
				}

				regionsB[y * numSplits + x].push_back(b);
			}
		}
	}

	regionProcessTime = timeGetTime() - time;
	time = timeGetTime();
	matches.reserve(numVertsA);

	// search for matches
	for (const DrawCommand* a = vertsA; a < &vertsA[numVertsA]; a++) {
		VertexMatch match;
		int bestSimilarity = 1<<30;

		match.polyA = a;
		match.polyB = a;

		// Detect which region this is in
		int smallestX = 9999, smallestY = 9999;

		for (int v = 0; v < a->numVertices; v++) {
			if (a->vertices[v].iX < smallestX) {
				smallestX = a->vertices[v].iX;
			}
			else if (a->vertices[v].iY < smallestY) {
				smallestY = a->vertices[v].iY;
			}
		}

		if (smallestX < 0) smallestX = 0;
		if (smallestY < 0) smallestY = 0;
		if (smallestX >= regionWidth) smallestX = regionWidth - 1;
		if (smallestY >= regionHeight) smallestY = regionHeight - 1;

		// Find the most similar vertex
		int aRegion = XYTOREGION(smallestX, smallestY);

		for (const DrawCommand* b : regionsB[aRegion]) {
			// ignore polys with mismatching side counts
			if (b->numVertices != a->numVertices) {
				continue;
			}

			int similarity = 0;

			for (int v = 0; v < a->numVertices; v++) {
				similarity += ((a->vertices[v].iX - b->vertices[v].iX) * (a->vertices[v].iX - b->vertices[v].iX))
							+ ((a->vertices[v].iY - b->vertices[v].iY) * (a->vertices[v].iY - b->vertices[v].iY))
							+ ((a->vertices[v].col == b->vertices[v].col) ? 0 : 10000);
			}

			if (similarity < bestSimilarity) {
				bestSimilarity = similarity;
				match.polyB = b;
			}
		}

		matches.push_back(match);
	}

	matchProcessTime = timeGetTime() - time;

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

		currentCommands.push_back(cmd);
		//currentPsxCommand = 
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