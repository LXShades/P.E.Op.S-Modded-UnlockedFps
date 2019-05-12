#include "fpsInterpolator.h"
#include <windows.h>
#include <gl/GL.h>

FpsInterpolator interpolator;

#define SETCOL(x)  if(x.col!=ulOLDCOL) {ulOLDCOL=x.col;glColor4ubv((GLubyte*)&x.col);} 
#define SETPCOL(x)  if(x->c.lcol!=ulOLDCOL) {ulOLDCOL=x->c.lcol;glColor4ubv(x->c.col);}

extern "C" {
	void CallPrimFunc(int gpuCommand, unsigned char* data); 
	void resetGpuThing();
}

void FpsInterpolator::Render() {
	const bool doClear = false;
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

	unsigned long ulOLDCOL = 0;
	bool doRenderFrame = true;
	int numFramesRendered = 0;
	GLuint currentTex = 753489;
	bool currentSmooth = false;
	
	isRendering = true;

	glShadeModel(GL_FLAT);
	glDisable(GL_BLEND);

	DWORD targetFrameDuration = 1000/12;
	float blendFactor = 0.0f;
	do {
		blendFactor = (timeGetTime() - currentFrameTime) / (float)targetFrameDuration;

		if (doInterpolation && blendFactor < 1.0f) {
			// wait for the next frame
			/*while (timeGetTime() - previousFrameTime < (int)(targetFrameDuration * blendFactor)) {
				continue;
			}*/


			// Render the interpolatable vertices multiple times
			for (DrawCommand& command : previousFrameDraws) {
				const VertexMatch& match = matches[&command - &previousFrameDraws[0]];
				const DrawCommand* nextCommand = match.polyB;

				// set texture
				if (currentTex != nextCommand->texture) {
					if (currentTex == -1) {
						glEnable(GL_TEXTURE_2D);
					}
					else if (nextCommand->texture == -1) {
						glDisable(GL_TEXTURE_2D);
					}

					currentTex = nextCommand->texture;
					glBindTexture(GL_TEXTURE_2D, currentTex);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
				}
				if (currentSmooth != nextCommand->isSmoothShaded) {
					glShadeModel(nextCommand->isSmoothShaded ? GL_SMOOTH : GL_FLAT);
					currentSmooth = nextCommand->isSmoothShaded;
				}

				// start poly render
				if (command.numVertices == 4) {
					glBegin(GL_TRIANGLE_STRIP);
				}
				else {
					glBegin(GL_TRIANGLES);
				}

				// blend vertices
				for (int v = 0; v < command.numVertices; v++) {
					Vertex blended = Vertex(command.vertices[v], nextCommand->vertices[v], blendFactor);
					SETCOL(nextCommand->vertices[v]);
					glTexCoord2fv((GLfloat*)&nextCommand->vertices[v].sow); // test
					glVertex3fv((GLfloat*)&blended.x);
				}

				glEnd();

				// draw vectors
				glBegin(GL_LINE_STRIP);
				float coords[6] = { 0 };
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
		}

		// Present to screen
		SwapBuffers(wglGetCurrentDC());
	} while (blendFactor < 0.99f && doInterpolation);

	isRendering = false;

	// Replace the command lists
	previousFrameDraws = currentFrameDraws;
	currentFrameDraws.clear();

	currentCommands.clear();
	
	// Update timings
	previousFrameTime = currentFrameTime;
	currentFrameTime = timeGetTime();
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

		// Insert into the region collection
		int region = XYTOREGION(smallestX, smallestY);

		regionsB[region].push_back(b);

		if (regionOverlap) {
			if (region + numSplits < numRegions) {
				regionsB[region + numSplits].push_back(b);
			}
			if (region - numSplits >= 0) {
				regionsB[region - numSplits].push_back(b);
			}
			if (((region + 1) % numSplits) != 0) {
				regionsB[region + 1].push_back(b);
			}
			if (region - 1 > 0 && ((region - 1) % numSplits) != numSplits - 1) {
				regionsB[region - 1].push_back(b);
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