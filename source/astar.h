//
// Created by MattHUN on 3/28/2024.
//
#ifndef PATHFINDING_ASTAR_ASTAR_H
#define PATHFINDING_ASTAR_ASTAR_H
#endif //PATHFINDING_ASTAR_ASTAR_H
#include <unordered_map>
#include <set>
#include <vector>
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
    // Type definition for unordered_map (for the "closed" map and the "walls" map)
    typedef std::unordered_map<Coordinate, Node*, coord_hash> nodeMap;
    // Type definition for set (for the open set)
    typedef std::set<Node*, sortbyFValue> nodeSet;
    // Type definition for unordered_map (for the "open set iterator" map)
    typedef std::unordered_map<Coordinate, nodeSet::iterator, coord_hash> nodeSetMap;
    // Admissible heuristics (ones that don't overestimate the cost of travelling from current node to goal node)
    class Heuristic {
    public:
        // Manhattan heuristic for grids that allow 4 directions of movement
        static unsigned int manhattan(const Coordinate& start_, const Coordinate& goal_) {
            unsigned int dx = abs(start_.x - goal_.x), dy = abs(start_.y - goal_.y);
            return 10 * (dx + dy);
        }
        // Diagonal heuristic for grids that allow 8 directions of movement
        static unsigned int diagonal(const Coordinate& start_, const Coordinate& goal_) {
            unsigned int dx = abs(start_.x - goal_.x), dy = abs(start_.y - goal_.y);
            return 10 * (dx + dy) - (6 * std::min(dx, dy));
        }
        // Euclidean heuristic for grids that allow more than 8 directions of movement
        static unsigned int euclidean(const Coordinate& start_, const Coordinate& goal_) {
            unsigned int dx = abs(start_.x - goal_.x), dy = abs(start_.y - goal_.y);
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
        nodeMap walls;
    public:
        // Size of map
        int mapSize_x = 100, mapSize_y = 100;
        // Add walls
        void addWall(const Coordinate& wall_) {
            walls[wall_] = new Node(wall_, nullptr, 0, 0);
        }
        // Clear walls
        void clearWalls() {
            walls.clear();
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
        static std::vector<Coordinate> findPath(const Coordinate& start_, const Coordinate& goal_, const bool& diagonal_, const Map& map) {
            // If start node or goal node is invalid, or the size of the map is invalid -> return an empty vector
            if (!map.isValid(start_) || !map.isValid(start_) || map.mapSize_x <= 0  || map.mapSize_y <= 0) return {};
            // Closed map
            nodeMap closed;
            // Open set (sorted by H value)
            nodeSet open;
            // A Coordinate map for storing iterators for the open set (for access with the Coordinate type)
            nodeSetMap itrMap;
            // Put the starting node in the open set and the Coordinate map
            // This "diagonal" boolean is a terrible way to implement directions, but I'm too lazy to fix it ¯\_(ツ)_/¯
            itrMap[start_] = open.insert(new Node(start_, nullptr, 0, diagonal_ ? Heuristic::diagonal(start_, goal_) : Heuristic::manhattan(start_, goal_))).first;
            // Prepping a bunch of stuff
            Node* current;
            Coordinate next;
            short directions = (diagonal_) ? 8 : 4;
            unsigned int tentative;
            // Now the fun begins...
            while (!open.empty()) {
                // Get an iterator for the Node with the lowest H value
                current = *open.begin();
                // If it's the goal, return the path
                if (current->h == 0) return reconstruct(current);
                // Else put this Node in the closed map, and erase it from the set and other map
                closed[current->coords] = current;
                itrMap.erase(itrMap.find(current->coords));
                open.erase(open.begin());
                // For all neighbors of this Node
                for (short i = 0; i < directions; i++) {
                    // Get Coordinate of this neighbor
                    next = current->coords + dir[i];
                    // If Coordinate is invalid (is a wall or out of boundaries), or is already evaluated (in the closed map), skip this neighbor
                    if (!map.isValid(next) || closed.find(next) != closed.end()) {
                        continue;
                    }
                    // Calculate the G cost of this neighbor
                    tentative = current->g + ((i < 4) ? 10 : 14);
                    // If this neighbor isn't in the open set and Coordinate map, insert it
                    if (itrMap.find(next) == itrMap.end()) {
                        itrMap[next] = open.insert(new Node(next, current, tentative, diagonal_ ? Heuristic::diagonal(next, goal_) : Heuristic::manhattan(next, goal_))).first;
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