#pragma once

#ifdef __cplusplus
#include <vector>

class FpsInterpolator {

public:
	// All-purpose vertex that can be preserved
	struct Vertex {
		Vertex() = default;
		Vertex(const Vertex& a, const Vertex& b, float interpolation) {
			float invInterpolation = 1.0f - interpolation;
			x = a.x * invInterpolation + b.x * interpolation;
			y = a.y * invInterpolation + b.y * interpolation;
			z = a.z * invInterpolation + b.z * interpolation;
			sow = a.sow;
			tow = a.tow;
			col = a.col;
		}
		
		// Main attributes
		float x, y, z;             // coords
		float sow, tow;            // UVs
		union {
			unsigned int col;      // colour (RGBA uint32)
			unsigned char bCol[4]; // colour (R,G,B,A bytes)
		};

		// Helper attributes
		int iX, iY, iZ; // integer versions of coords
		int iU, iV; // absolute UV in PSX VRAM
	};

	// A draw command containing a draw type, vertices, and STUFF
	struct DrawCommand {
		DrawCommand(Vertex& a, Vertex& b, Vertex& c, Vertex& d, int texture, bool bDrawSmoothShaded, bool bBlend) {
			vertices[0] = a; vertices[1] = b; vertices[2] = c; vertices[3] = d;
			numVertices = 4;
			isSmoothShaded = bDrawSmoothShaded;
			isBlended = bBlend;
			this->texture = texture;

			if (b.iX < minX) { minX = b.iX; }
			if (c.iX < minX) { minX = c.iX; }
			if (d.iX < minX) { minX = d.iX; }
			if (b.iY < minY) { minY = b.iY; }
			if (c.iY < minY) { minY = c.iY; }
			if (d.iY < minY) { minY = d.iY; }
			if (b.iX > maxX) { maxX = b.iX; }
			if (c.iX > maxX) { maxX = c.iX; }
			if (d.iX > maxX) { maxX = d.iX; }
			if (b.iY > maxY) { maxY = b.iY; }
			if (c.iY > maxY) { maxY = c.iY; }
			if (d.iY > maxY) { maxY = d.iY; }
		}

		DrawCommand(Vertex& a, Vertex& b, Vertex& c, int texture, bool bDrawSmoothShaded, bool bBlend) {
			vertices[0] = a; vertices[1] = b; vertices[2] = c;
			numVertices = 3;
			isSmoothShaded = bDrawSmoothShaded;
			isBlended = bBlend;
			this->texture = texture;

			minX = vertices[0].iX;
			minY = vertices[0].iY;
			maxX = vertices[0].iX;
			maxY = vertices[0].iY;
			
			if (b.iX < minX) { minX = b.iX; }
			if (c.iX < minX) { minX = c.iX; }
			if (b.iY < minY) { minY = b.iY; }
			if (c.iY < minY) { minY = c.iY; }
			if (b.iX > maxX) { maxX = b.iX; }
			if (c.iX > maxX) { maxX = c.iX; }
			if (b.iY > maxY) { maxY = b.iY; }
			if (c.iY > maxY) { maxY = c.iY; }
		}

		int CalculateDivergence(const DrawCommand& other) const {
			int divergence = 0;
			const int distanceWeight = 5000, colourWeight = 0, textureWeight = 1000;
			const int avgMaxDistance = 500;

			for (int v = 0; v < numVertices; v++) {
				int xDiff = (vertices[v].iX - other.vertices[v].iX), yDiff = (vertices[v].iY - other.vertices[v].iY);
				int rDiff = vertices[v].bCol[0] - other.vertices[v].bCol[0];
				int gDiff = vertices[v].bCol[1] - other.vertices[v].bCol[1];
				int bDiff = vertices[v].bCol[2] - other.vertices[v].bCol[2];

				// space difference
				divergence += ((xDiff ^ (xDiff >> 31)) - (xDiff >> 31) + (yDiff ^ (yDiff >> 31)) - (yDiff >> 31)) * distanceWeight / avgMaxDistance;

				if (other.texture != -1 && texture != -1) {
					int sDiff = (vertices[v].sow - other.vertices[v].sow) * textureWeight, tDiff = (vertices[v].tow - other.vertices[v].tow) * textureWeight;

					// texture difference
					divergence += (sDiff ^ (sDiff >> 31)) - (sDiff >> 31) + (tDiff ^ (tDiff >> 31)) - (tDiff >> 31);
				}

				// colour difference
				divergence += ((rDiff ^ (rDiff >> 31)) - (rDiff >> 31) + (gDiff ^ (gDiff >> 31)) - (gDiff >> 31) + (bDiff ^ (bDiff >> 31)) - (bDiff >> 31)) * colourWeight / (255 * 3);
			}

			if (other.texture != texture) {
				divergence += textureWeight;
			}

			return divergence;
		}

		// Rendering attributes
		Vertex vertices[4];
		int numVertices;
		int texture;
		bool isSmoothShaded;
		bool isBlended;

		// Helper attributes
		int minX, minY, maxX, maxY;
	};

	struct PsxCommand {
		int gpuCommand;
		unsigned char data[256];
	};

	// Links two vertices
	struct PolyMatch {
		const DrawCommand* polyA;
		const DrawCommand* polyB;

		float divergence;
	};

	// Links a DrawCommand to an entity and neighbours
	struct NeighbourLinks {
		const FpsInterpolator::DrawCommand* neighbours[4];

		int entity;
	};

public:
	// Renders the interpolated frames
	void Render();

	// Draws previousFrameDraws blended towards currentFrameDraws by 'blendfactor'
	void DrawBlendedCommands(const std::vector<struct FpsInterpolator::PolyMatch>& matches, float blendFactor);

	// Draws motion vectors between previous and current frame
	void DrawMotionVectors(const std::vector<PolyMatch>& matches);

	// Draws polygons as solid colours (useful for debugging)
	void DrawSolidPolygons(const std::vector<const DrawCommand*>& commands, unsigned int colour);

	// Draws entity lines and colours
	void DrawEntities(const std::vector<struct NeighbourLinks>& links, const DrawCommand* commands, int numCommands);

public:
	// Matches vertices from a previous frame to the next frame
	std::vector<PolyMatch> MatchVertices(const DrawCommand* vertsA, int numVertsA, const DrawCommand* vertsB, int numVertsB);

	// Matches vertices (attempt 2)
	std::vector<FpsInterpolator::PolyMatch> MatchVerticesByEntity();

	// Isolates entities on frame thing
	std::vector<struct NeighbourLinks> IsolateEntities(const DrawCommand* commands, int numCommands);

public:
	// Records a draw command from the game
	void RecordDrawCommand(const DrawCommand& command);

	// Records a direct GPU command (testing)
	void RecordPsxCommand(int gpuCommand, const unsigned char * data);

public:
	// Registers that a frame cap has been requested
	void RegisterFrame();

	// Sets the frame rate
	void SetFrameRate(int framerate);

private:
	std::vector<DrawCommand> currentDrawCommands;
	std::vector<NeighbourLinks> currentEntities;
	std::vector<DrawCommand> previousDrawCommands;
	std::vector<NeighbourLinks> previousEntities;
	int currentFrameTime = 0;
	int previousFrameTime = 0;

	int framerate;
	long long int currentFrame = 0;
	long long int lastRenderedFrame = 0;

	bool isRendering = false;
};

extern FpsInterpolator interpolator;
#else
// C wrapper for C++ code
void interpolatorRecord(struct OGLVertex* a, struct OGLVertex* b, struct OGLVertex* c, struct OGLVertex* d, int texture, int bDrawSmoothShades, int bBlend);
int interpolatorRecordPsx(int gpuCommand, const unsigned char* data);
void interpolatorUpdateDisplay();

// used so that C++ code can call primitive functions
void CallPrimFunc(int gpuCommand, unsigned char* data);

void initInterpolator();

int interpolatorManageFramecap(int framerate);
#endif