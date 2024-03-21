#include "Mesh.h"
#include <limits>
#include <queue>
#include <cmath>
#include <fstream> 

Edge::Edge(int id, int v1, int v2) : idx(id), v1i(v1), v2i(v2) {}

void Edge::computeLength(const std::vector<Vertex*>& verts)
{
    float dx = verts[v2i]->coords[0] - verts[v1i]->coords[0];
    float dy = verts[v2i]->coords[1] - verts[v1i]->coords[1];
    float dz = verts[v2i]->coords[2] - verts[v1i]->coords[2];

    length = std::sqrt(dx * dx + dy * dy + dz * dz);
}



void Mesh::loadOff(const char* name)
{
    FILE* fPtr = fopen(name, "r");
    char str[334];

    fscanf(fPtr, "%s", str);

    int nVerts, nTris, n, i = 0;
    float x, y, z;

    fscanf(fPtr, "%d %d %d\n", &nVerts, &nTris, &n);
    while (i++ < nVerts)
    {
        fscanf(fPtr, "%f %f %f", &x, &y, &z);
        addVertex(x, y, z);
    }

    while (fscanf(fPtr, "%d", &i) != EOF)
    {
        fscanf(fPtr, "%f %f %f", &x, &y, &z);
        addTriangle((int)x, (int)y, (int)z);
    }

    fclose(fPtr);
}

void Mesh::createCube(float sideLen)
{
    // coordinates
    float flbc[3] = { 0, 0, 0 }, deltaX = 0, deltaY = 0, deltaZ = 0;
    for (int v = 0; v < 8; v++)
    {
        switch (v)
        {
        case 1:
            deltaX = sideLen;
            break;
        case 2:
            deltaZ = -sideLen;
            break;
        case 3:
            deltaX = 0;
            break;
        case 4:
            deltaZ = 0;
            deltaY = sideLen;
            break;
        case 5:
            deltaX = sideLen;
            break;
        case 6:
            deltaZ = -sideLen;
            break;
        default:
            deltaX = 0;;
            break;
        }
        addVertex(flbc[0] + deltaX, flbc[1] + deltaY, flbc[2] + deltaZ);
    }

    addTriangle(0, 2, 1);
    addTriangle(0, 3, 2);

    addTriangle(1, 2, 5);
    addTriangle(2, 6, 5);

    addTriangle(2, 3, 6);
    addTriangle(3, 7, 6);

    addTriangle(3, 4, 7);
    addTriangle(3, 0, 4);

    addTriangle(4, 5, 6);
    addTriangle(4, 6, 7);

    addTriangle(0, 1, 5);
    addTriangle(0, 5, 4);
}

void Mesh::addTriangle(int v1, int v2, int v3)
{
    int idx = tris.size();
    tris.push_back(new Triangle(idx, v1, v2, v3));

    // set up structure

    verts[v1]->triList.push_back(idx);
    verts[v2]->triList.push_back(idx);
    verts[v3]->triList.push_back(idx);

    if (!makeVertsNeighbor(v1, v2))
        addEdge(v1, v2);

    if (!makeVertsNeighbor(v1, v3))
        addEdge(v1, v3);

    if (!makeVertsNeighbor(v2, v3))
        addEdge(v2, v3);
}

bool Mesh::makeVertsNeighbor(int v1i, int v2i)
{
    // returns true if v1i already neighbor w/ v2i; false o/w

    for (int i = 0; i < verts[v1i]->vertList.size(); i++)
        if (verts[v1i]->vertList[i] == v2i)
            return true;

    verts[v1i]->vertList.push_back(v2i);
    verts[v2i]->vertList.push_back(v1i);
    return false;
}

void Mesh::addVertex(float x, float y, float z)
{
    int idx = verts.size();
    float* c = new float[3];
    c[0] = x;
    c[1] = y;
    c[2] = z;

    verts.push_back(new Vertex(idx, c));
}

void Mesh::addEdge(int v1, int v2)
{
    int idx = edges.size();

    edges.push_back(new Edge(idx, v1, v2));

    verts[v1]->edgeList.push_back(idx);
    verts[v2]->edgeList.push_back(idx);
}

struct CompareDist
{
    bool operator()(const std::pair<int, int>& a, const std::pair<int, int>& b) const
    {
        return a.second > b.second;
    }
};

// Function to calculate the Euclidean distance between two vertices
float Mesh::distanceBetweenVertices(Vertex* v1, Vertex* v2)
{
    // Calculate the Euclidean distance between two vertices
    float dx = v2->coords[0] - v1->coords[0];
    float dy = v2->coords[1] - v1->coords[1];
    float dz = v2->coords[2] - v1->coords[2];

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void Mesh::dijkstra(int sourceIdx)
{
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, CompareDist> pq;
    pq.push(std::make_pair(sourceIdx, 0));

    // Initialize distances and previous vertices
    for (int i = 0; i < verts.size(); ++i)
    {
        verts[i]->distance = std::numeric_limits<int>::max();
        verts[i]->previous = -1;
    }

    // Set distance of source vertex to 0
    verts[sourceIdx]->distance = 0;

    while (!pq.empty())
    {
        int u = pq.top().first;
        pq.pop();

        // Iterate through all neighboring vertices of u
        for (std::vector<int>::iterator it = verts[u]->vertList.begin(); it != verts[u]->vertList.end(); ++it)
        {
            int v = *it;

            // Calculate new distance to vertex v
            float edgeLength = distanceBetweenVertices(verts[u], verts[v]); // Calculate the actual length of the edge
            float alt = verts[u]->distance + static_cast<float>(edgeLength); // Use the actual edge length

            // Update distance and previous vertex if shorter path found
            if (alt < verts[v]->distance)
            {
                verts[v]->distance = alt;
                verts[v]->previous = u;
                pq.push(std::make_pair(v, alt));
            }
        }
    }
}


void Mesh::calculateGeodesicDistanceMatrix()
{
    // Initialize geodesic distance matrix
    for (int i = 0; i < MAX_VERTICES; ++i)
    {
        for (int j = 0; j < MAX_VERTICES; ++j)
        {
            geodesicDistanceMatrix[i][j] = INT_MAX; // Initialize all distances to "infinity"
        }
    }

    // Calculate geodesic distances using Dijkstra's algorithm for each vertex
    for (int sourceIdx = 0; sourceIdx < verts.size(); ++sourceIdx)
    {
        dijkstra(sourceIdx); // Run Dijkstra's algorithm from the current source vertex

        // Store the calculated distances in the geodesic distance matrix
        for (int targetIdx = 0; targetIdx < verts.size(); ++targetIdx)
        {
            geodesicDistanceMatrix[sourceIdx][targetIdx] = verts[targetIdx]->distance;
        }
    }
}

void Mesh::printDistanceMatrixToFile(const char* filename)
{
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "Unable to open file for writing: " << filename << std::endl;
        return;
    }

    // Write the distance matrix to the file
    for (int i = 0; i < verts.size(); ++i) {
        for (int j = 0; j < verts.size(); ++j) {
            outfile << geodesicDistanceMatrix[i][j] << " ";
        }
        outfile << std::endl;
    }

    outfile.close();
}