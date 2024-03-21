#include "Painter.h"

SoSeparator* Painter::getShapeSep(Mesh* mesh)
{
    SoSeparator* res = new SoSeparator();

    // Transformation - Not needed

    // Color
    SoMaterial* mat = new SoMaterial();
    mat->diffuseColor.setValue(0, 1, 0); // Paint all vertices with this color

    res->addChild(mat);

    // Shape
    SoCoordinate3* coords = new SoCoordinate3();
    for (int c = 0; c < mesh->verts.size(); c++)
        coords->point.set1Value(c, mesh->verts[c]->coords[0], mesh->verts[c]->coords[1], mesh->verts[c]->coords[2]);

    SoIndexedFaceSet* faceSet = new SoIndexedFaceSet();
    for (int c = 0; c < mesh->tris.size(); c++)
    {
        faceSet->coordIndex.set1Value(c * 4, mesh->tris[c]->v1i);
        faceSet->coordIndex.set1Value(c * 4 + 1, mesh->tris[c]->v2i);
        faceSet->coordIndex.set1Value(c * 4 + 2, mesh->tris[c]->v3i);
        faceSet->coordIndex.set1Value(c * 4 + 3, -1);
    }

    res->addChild(coords);
    res->addChild(faceSet);

    return res;
}

void Painter::drawGeodesicPath(SoSeparator* sep, Mesh* mesh, int startIdx, int endIdx)
{
    SoSeparator* thickLineSep = new SoSeparator();

    // Material
    SoMaterial* ma = new SoMaterial();
    ma->diffuseColor.set1Value(0, 1.0f, 0.0f, 0.0f); // Red color
    thickLineSep->addChild(ma);

    // Draw style
    SoDrawStyle* sty = new SoDrawStyle();
    sty->lineWidth = 5.0f; // Set line width
    thickLineSep->addChild(sty);

    // Shape
    SoLineSet* lineSet = new SoLineSet();
    SoCoordinate3* co = new SoCoordinate3();

    std::vector<int>& path = mesh->verts[endIdx]->geodesicPath; // Get geodesic path

    for (int i = 0; i < path.size(); i++)
    {
        co->point.set1Value(i, mesh->verts[path[i]]->coords[0], mesh->verts[path[i]]->coords[1], mesh->verts[path[i]]->coords[2]);
    }

    lineSet->numVertices.set1Value(0, path.size()); // Set the number of vertices

    thickLineSep->addChild(co);
    thickLineSep->addChild(lineSet);

    sep->addChild(thickLineSep);
}