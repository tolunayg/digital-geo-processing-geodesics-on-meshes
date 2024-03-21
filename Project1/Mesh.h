#pragma once

#include <iostream>
#include <vector>

struct Vertex
{
    float* coords, * normals; // 3d coordinates etc
    int idx; // who am i; verts[idx]

    std::vector<int> vertList; // adj vertices;
    std::vector<int> triList;
    std::vector<int> edgeList;

    int distance; // Distance from the source vertex in Dijkstra's algorithm
    int previous; // Previous vertex in the shortest path in Dijkstra's algorithm
    std::vector<int> geodesicPath; // Geodesic path from the source vertex to this vertex
    
    Vertex(int i, float* c) : idx(i), coords(c)
    {
        distance = INT_MAX;
        previous = -1;
    }
};

struct Edge
{
    int idx; // edges[idx]
    int v1i, v2i; // endpoints
    float length;
    Edge(int id, int v1, int v2);
    void computeLength(const std::vector<Vertex*>& verts);
};

struct Triangle
{
    int idx; // tris[idx]
    int v1i, v2i, v3i;
    Triangle(int id, int v1, int v2, int v3) : idx(id), v1i(v1), v2i(v2), v3i(v3) {};
};

class Mesh
{
private:
    void addTriangle(int v1, int v2, int v3);
    void addEdge(int v1, int v2);
    void addVertex(float x, float y, float z);
    bool makeVertsNeighbor(int v1i, int v2i);

public:
    std::vector<Vertex*> verts;
    std::vector<Triangle*> tris;
    std::vector<Edge*> edges;

    Mesh() {};
    void createCube(float side);
    void loadOff(const char* name);
    void dijkstra(int sourceIdx);
    void calculateGeodesicDistanceMatrix();
};
