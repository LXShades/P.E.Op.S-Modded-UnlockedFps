
#include "fpsInterpolator.h"

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
		} else {
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