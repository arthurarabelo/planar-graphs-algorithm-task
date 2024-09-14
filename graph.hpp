#include <vector>
#include <cmath>
#include <algorithm>
#include <map>
#include <set>
#include <iostream>

struct Point{
    double x, y;

    // true if x coords & y coords are equal
    bool operator==(const Point& other) const {
        return this->x == other.x && this->y == other.y;
    }

    // true if x coords or y coords are not equal
    bool operator!=(const Point& other) const {
        return this->x != other.x || this->y != other.y;
    }

    // true if x coord is smaller than the other x coord
    // also true, in case x's are equal, if y coord is smaller than the other y coord
    bool operator<(const Point& other) const {
        if (this->x != other.x) return this->x < other.x;
        return this->y < other.y;   
    }
};

class Edge{
    public:
        Edge(Point from, Point to){
            this->from = from;
            this->to = to;
            this->angle = atan2(this->to.y - this->from.y, this->to.x - this->from.x);
        }
        
        //get the reverse directed edge
        Edge getReverse() const{
            Edge reverse(this->to, this->from);
            reverse.fromIndex = this->toIndex;
            reverse.toIndex = this->fromIndex;
            return reverse;
        }

        void setToIndex(int i){
            this->toIndex = i;
        }

        void setFromIndex(int i){
            this->fromIndex = i;
        }

        int getToIndex() const{
            return this->toIndex;
        }

        int getFromIndex() const{
            return this->fromIndex;
        }

        bool operator==(const Edge& other) const{
            if(this->from == other.from && this->to == other.to){
                    return true;
            }
            return false;
        }

        bool operator<(const Edge& other) const {
            if(this->angle == other.angle) return this->toIndex < other.toIndex;
            return this->angle < other.angle;
        }

    private:
        Point from;
        Point to;
        int fromIndex;
        int toIndex;
        double angle;
};

class Vertex{
    public:
        bool visited;

        Vertex(int index, Point p){
            this->index = index;
            this->p = p;
            this->visited = false;
            this->num_neighbors = 0;
        }

        int getIndex() const{
            return this->index;
        }

        void addEdge(Edge e){
            this->outGoingEdges.push_back(e);
        }

        void sortEdges(){
            std::sort(this->outGoingEdges.begin(), this->outGoingEdges.end());
        }

        void addNeighbor(int neighborIndex){
            this->neighbors.push_back(neighborIndex);
            this->num_neighbors++;
        }

        int getNeighbor(int i) const{
            return this->neighbors[i];
        }

        int getNumNeighbors() const{
            return this->num_neighbors;
        }

        double getX() const{
            return this->p.x;
        }

        double getY() const{
            return this->p.y;
        }

        Edge getEdge(int i) const{
            return this->outGoingEdges[i];
        }

        private:
            int index;
            Point p;
            std::vector<Edge> outGoingEdges;
            std::vector<int> neighbors;
            int num_neighbors;
};

class GraphFaces{
    public:
        void insertChains(Edge e1, Edge e2){
            this->chains_map.insert(std::pair<Edge, Edge>(e1, e2)); // -> If a single element is inserted, logarithmic in size in general O(log n), \
            but amortized constant if a hint is given and the position given is the optimal
        }

        void findFaces(){

            // loop inside the map of the chained edges
            for (std::map<Edge, Edge>::iterator iterator = chains_map.begin();
            iterator != chains_map.end(); ++iterator){ // Θ(2E)
                std::vector<int> face; // declare the current face
                face.clear();
                
                Edge firstKey = iterator->first; // firstKey = in the first iteration is equal the first element (edge) of the first pair in the map

                auto itSet = this->chained_edges.find(firstKey); // check if firstKey is already chained -> O(log n)
                if(itSet != chained_edges.end()){ 
                    continue;
                }
                chained_edges.insert(firstKey); // 'mark' the edge as already chained -> Θ(log n)

                face.push_back(firstKey.getFromIndex()); // insert in the face the index of the origin vertex (the 'from vertex' of the edge) -> O(1)

                // declare and initialize current_key and current_value
                Edge current_key = iterator->first;
                Edge current_value = iterator->second;

                while(true){ // runs through the map linking the chains
                    auto it = chains_map.find(current_value); // finds the pair which has the current_value as its key
                    if(it == chains_map.end() || it->second == firstKey){ // check if there's not such key or the key is equal the firstKey of the loop
                        chained_edges.insert(it->first); // 'mark' the edge as already chained

                        // inserts the last vertex of the face and the first one
                        face.push_back(it->first.getFromIndex());
                        face.push_back(firstKey.getFromIndex()); 
                        break;
                    }

                    // update the current_key and the current_value w/ the element which has the current_value as its key
                    current_key = it->first;
                    current_value = it->second;


                    chained_edges.insert(current_key); // 'mark' the edge as already chained
                    face.push_back(current_key.getFromIndex()); // inserts the vertex in the face
                }
                faces.push_back(face); // inserts the face in the vector of faces
            }
        }

        void printFaces(){ // Θ(2E)
            std::cout << faces.size() << std::endl;
            for(auto face : faces){
                std::cout << face.size() << " ";
                for(auto v : face){
                    std::cout << v << " ";
                }
                std::cout << std::endl;
            }
        }

    private:
        std::multimap<Edge, Edge> chains_map;
        std::vector<std::vector<int>> faces;
        std::set<Edge> chained_edges;
};