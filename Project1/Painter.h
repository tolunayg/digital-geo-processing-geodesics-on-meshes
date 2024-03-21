#pragma once

#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoLineSet.h> // Added for drawing lines
#include <Inventor/nodes/SoDrawStyle.h> // Added for setting line width

#include "Mesh.h"

class Painter
{
public:
    SoSeparator* getShapeSep(Mesh* mesh);
    void drawGeodesicPath(SoSeparator* sep, Mesh* mesh, int startIdx, int endIdx);
};