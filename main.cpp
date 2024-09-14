#include <iostream>
#include "graph.hpp"
#include <chrono>

int main(){
    GraphFaces faces;
    int num_v, num_e;
    std::vector<Vertex> vertices;

    std::cin >> num_v;
    std::cin >> num_e;

    // loop to read the input -> Θ(V + 2E)
    for(int i = 0; i < num_v; i++){
        double v_x, v_y; 
        int num_neighbors, neighborIndex;
        std::cin >> v_x >> v_y >> num_neighbors; 
        Point p;
        p.x = v_x;
        p.y = v_y;
        vertices.push_back(Vertex(i+1, p)); // creates the current vertex and inserts it in the vector
        for(int j = 0; j < num_neighbors; j++){ // loop to add the neighbors in the current vertex
            std::cin >> neighborIndex;
            vertices[i].addNeighbor(neighborIndex);
        }
    }

    // For each node v:
    // Get v's neighbors sorted anticlockwise as u_1, u_2, u_3, ..., u_n
    // For each half-edge (v, u_i):
    //     Update half-edge (u_{i+1 mod n}, v) to chain to (v, u_i)
    for(Vertex vertex : vertices){  // -> Θ(V + 4E)
        Point from;
        int fromIndex = vertex.getIndex();
        from.x = vertex.getX();
        from.y = vertex.getY();

        for(int i = 0; i < vertex.getNumNeighbors(); i++){
            Point to;
            to.x = vertices[vertex.getNeighbor(i)-1].getX();
            to.y = vertices[vertex.getNeighbor(i)-1].getY();
            int toIndex = vertices[vertex.getNeighbor(i)-1].getIndex();

            // creates the edge and inserts it in the outgoing edges of the current vertex
            Edge e(from, to);
            e.setFromIndex(fromIndex);
            e.setToIndex(toIndex);
            vertex.addEdge(e);
        }
        vertex.sortEdges(); // sort the outgoing edges of the current vertex by their angle or, if the angle is the same, by their 'to vertex' (vertex of destination) index
        for(int j = 0; j < vertex.getNumNeighbors(); j++){
            Edge e1 = vertex.getEdge(j); // gets the j-index outgoing edge of the vertex
            faces.insertChains(vertex.getEdge(j), vertex.getEdge((j+1) % vertex.getNumNeighbors()).getReverse()); // insert the chain between the j edge and the \
            reverse edge of the j+1 edge
        }
    }

    auto start = std::chrono::high_resolution_clock::now();
    faces.findFaces();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    std::cout << "Execution time: " << duration.count() << " seconds" << std::endl;


    faces.printFaces();    
}