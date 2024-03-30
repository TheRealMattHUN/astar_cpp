//
// Created by MattHUN on 3/28/2024.
//
#ifndef PATHFINDING_ASTAR_ASTAR_H
#define PATHFINDING_ASTAR_ASTAR_H
#endif //PATHFINDING_ASTAR_ASTAR_H
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <vector>
#include <functional>
namespace A_star {
    // Coordinate with an x and y value
    struct Coordinate {
        int x, y;
        // Constructors
        Coordinate() = default;
        Coordinate(int x_, int y_) : x(x_), y(y_) {};
        // Operators: one to compare, and one to add 2 Coordinates together
        bool operator == (const Coordinate& right_) const {
            return (x == right_.x && y == right_.y);
        }
        friend Coordinate operator + (Coordinate& left_, Coordinate& right_) {
            return {left_.x + right_.x, left_.y + right_.y};
        }
    };
    // Hard-coded directions, since we can add Coordinates together now
    Coordinate dir[8] = { {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1} };
    // Node
    struct Node {
        // Parent (so we can backtrack through our path when we finished)
        Node* parent;
        Coordinate coords;
        // g = the current shortest path found from the start to this node
        // h = a heuristic (estimate) of the distance between this node and goal node
        // f = g + h
        unsigned int g, h, f;
        // Constructor
        Node(Coordinate coords_, Node* parent_, unsigned int g_, unsigned int h_) : coords(coords_), parent(parent_), g(g_), h(h_), f(g_ + h_) {};
    };
    // Special hashing algo for unordered_map to store values with a Coordinate key
    struct coord_hash {
        size_t operator() (const Coordinate& coords_) const {
            return ((size_t)coords_.x << 32) | coords_.y;
        }
    };
    // Special sorting algo for the open set to store Node pointers sorted by the most recent F value
    struct sortbyFValue {
        bool operator() (const Node* left_, const Node* right_) const {
            return left_->f <= right_->f;
        }
    };
    // Type definition for unordered_set (for the "closed" map and the "walls" map)
    typedef std::unordered_set<Coordinate, coord_hash> coordSet;
    // Type definition for set (for the open set)
    typedef std::set<Node*, sortbyFValue> nodeSet;
    // Type definition for unordered_map (for the "open set iterator" map)
    typedef std::unordered_map<Coordinate, nodeSet::iterator, coord_hash> nodeSetMap;
    // Type definition for wrapper for heuristic functions
    typedef std::function<unsigned int(unsigned int, unsigned int)> HFunc;
    // Admissible heuristics (ones that don't overestimate the cost of travelling from current node to goal node)
    class Heuristic {
    public:
        // Manhattan heuristic for grids that allow 4 directions of movement
        static unsigned int manhattan(const unsigned int& dx, const unsigned int& dy) {
            return 10 * (dx + dy);
        }
        // Diagonal heuristic for grids that allow 8 directions of movement
        static unsigned int diagonal(const unsigned int& dx, const unsigned int& dy) {
            return 10 * (dx + dy) - (6 * std::min(dx, dy));
        }
        // Euclidean heuristic for grids that allow more than 8 directions of movement
        static unsigned int euclidean(const unsigned int& dx, const unsigned int& dy) {
            // Getting the answer with square root approximation (7 iterations are enough, it seems)
            // Starting off with the sum of dx2 + dy2
            unsigned int num = (dx*dx + dy*dy), estimate = 1;
            // Heron's method
            for (unsigned short i = 0; i < 7; i++) estimate = 0.5 * (estimate + num/estimate);
            return estimate;
        }
    };
    // Map
    // Calling findPath requires a Map object.
    class Map {
    private:
        // Walls (cannot pass through these nodes)
        coordSet walls;
        // Size of map
        int mapSize_x = 100, mapSize_y = 100;
        // Heuristic function wrapper
        HFunc wrapper;
        // Diagonal??
        bool diagonal = false;
    public:
        // Set world size
        void setWorldSize(const int& x_, const int& y_) {
            if (x_ > 0) mapSize_x = x_;
            if (y_ > 0) mapSize_y = y_;
        }
        // Get world size
        Coordinate getWorldSize() const {
            return {mapSize_x, mapSize_y};
        }
        // Set diagonal
        void setDiagonal(const bool& diagonal_) {
            diagonal = diagonal_;
        }
        // Get diagonal
        bool getDiagonal() const {
            return diagonal;
        }
        // Add walls
        void addWall(const Coordinate& wall_) {
            walls.insert(wall_);
        }
        // Get walls
        coordSet getWalls() const {
            return walls;
        }
        // Clear walls
        void clearWalls() {
            walls.clear();
        }
        // Set Heuristic
        void setHeuristic(const HFunc& func_) {
            wrapper = [func_](auto && PH1, auto && PH2) { return func_(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2)); };
        }
        // Get Heuristic
        unsigned int getHeuristic(const Coordinate& start_, const Coordinate& goal_) const {
            unsigned int dx = abs(start_.x - goal_.x), dy = abs(start_.y - goal_.y);
            return wrapper(dx, dy);
        }
        // Is coordinate valid? (is inside boundaries and isn't a wall)
        bool isValid(const Coordinate& coords_) const {
            return (coords_.x > -1 && coords_.y > -1 && coords_.x < mapSize_x && coords_.y < mapSize_y && walls.find(coords_) == walls.end());
        }
    };
    // Searching algo and its supplements
    class Search {
    private:
        // Reconstruct
        // We iterate through the parents of the nodes from the final node, since that is the shortest path
        // We add their coordinates to a vector of type Coordinate, and that will be our final result.
        static std::vector<Coordinate> reconstruct(Node* current) {
            std::vector<Coordinate> result;
            result.push_back(current->coords);
            while (current->parent) {
                current = current->parent;
                result.push_back(current->coords);
            }
            return result;
        }
    public:
        // findPath, the main algo
        // Returns
        //      -> an empty vector if no path exists from start node to goal node
        //      -> every coordinate on the shortest path, including the start node and the goal node
        static std::vector<Coordinate> findPath(const Coordinate& start_, const Coordinate& goal_, const Map& map) {
            // If start node or goal node is invalid, or the size of the map is invalid -> return an empty vector
            if (!(map.isValid(start_) || map.isValid(start_))) return {};
            // Closed set
            coordSet closed;
            // Open set (sorted by H value)
            nodeSet open;
            // A Coordinate map for storing iterators for the open set (for access with the Coordinate type)
            nodeSetMap itrMap;
            // Put the starting node in the open set and the Coordinate map
            // This "diagonal" boolean is a terrible way to implement directions, but I'm too lazy to fix it ¯\_(ツ)_/¯
            itrMap[start_] = open.insert(new Node(start_, nullptr, 0, map.getHeuristic(start_, goal_))).first;
            // Prepping a bunch of stuff
            Node* current;
            Coordinate next;
            short directions = (map.getDiagonal()) ? 8 : 4;
            unsigned int tentative;
            // Now the fun begins...
            while (!open.empty()) {
                // Get an iterator for the Node with the lowest H value
                current = *open.begin();
                // If it's the goal, return the path
                if (current->h == 0) return reconstruct(current);
                // Else put this Node in the closed set, and erase it from the sets and map
                closed.insert(current->coords);
                itrMap.erase(current->coords);
                open.erase(open.begin());
                // For all neighbors of this Node
                for (short i = 0; i < directions; i++) {
                    // Get Coordinate of this neighbor
                    next = current->coords + dir[i];
                    // If Coordinate is invalid (is a wall or out of boundaries), or is already evaluated (in the closed set), skip this neighbor
                    if (!map.isValid(next) || closed.find(next) != closed.end()) {
                        continue;
                    }
                    // Calculate the G cost of this neighbor
                    tentative = current->g + ((i < 4) ? 10 : 14);
                    // If this neighbor isn't in the open set and Coordinate map, insert it
                    if (itrMap.find(next) == itrMap.end()) {
                        itrMap[next] = open.insert(new Node(next, current, tentative, map.getHeuristic(next, goal_))).first;
                    }
                    // Else if it has a lower G value than what is stored, update its G cost and parent
                    else if (tentative < (*itrMap[next])->g) {
                        (*itrMap[next])->g = tentative;
                        (*itrMap[next])->f = tentative + (*itrMap[next])->h;
                        (*itrMap[next])->parent = current;
                    }
                }
            }
            // If we checked every possibility, return an empty vector
            return {};
        }
    };
}