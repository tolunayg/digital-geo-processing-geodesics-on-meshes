#include <Inventor/Win/SoWin.h>
#include <Inventor/Win/viewers/SoWinExaminerViewer.h>
#include "Mesh.h"
#include "Painter.h"

int main(int, char** argv)
{
    HWND window = SoWin::init(argv[0]);

    SoWinExaminerViewer* viewer = new SoWinExaminerViewer(window);
    Mesh* mesh = new Mesh();
    Painter* painter = new Painter();

    char filename[] = "0.off";
    mesh->loadOff(filename);

    // Calculate geodesic distance matrix
    mesh->calculateGeodesicDistanceMatrix();

    SoSeparator* root = new SoSeparator();
    root->ref();

    // Add mesh to the scene
    root->addChild(painter->getShapeSep(mesh));

    // Draw geodesic path between two vertices (change start and end indices as needed)
    painter->drawGeodesicPath(root, mesh, 0, 5);

    // Add painter to the scene
    root->addChild(painter->getShapeSep(mesh));

    viewer->setSize(SbVec2s(640, 480));
    viewer->setSceneGraph(root);
    viewer->show();

    SoWin::show(window);
    SoWin::mainLoop();
    delete viewer;
    root->unref();
    return 0;
}