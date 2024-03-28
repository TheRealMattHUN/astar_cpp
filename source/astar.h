//
// Created by MattHUN on 3/28/2024.
//
#ifndef PATHFINDING_ASTAR_ASTAR_H
#define PATHFINDING_ASTAR_ASTAR_H
#endif //PATHFINDING_ASTAR_ASTAR_H
#include <unordered_map>
#include <vector>
namespace A_star {
    // Coordinate with an x and y value
    struct Coordinate {
        int x, y;
        // Constructors
        Coordinate() = default;
        Coordinate(int x_, int y_) : x(x_), y(y_) {};
        // Operators: to compare, and to add together
        bool operator == (const Coordinate& right_) const {
            return (x == right_.x && y == right_.y);
        }
        friend Coordinate operator + (Coordinate& left_, Coordinate& right_) {
            return {left_.x + right_.x, left_.y + right_.y};
        }
    };
    // Special hashing algo for unordered_map to store values with a Coordinate key
    struct coord_hash {
        size_t operator() (const Coordinate& coords_) const {
            return ((size_t)coords_.x << 16) | coords_.y;
        }
    };
    // Hard-coded directions, since we can add Coordinates together now
    Coordinate dir[8] = { {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1} };
    // Node
    struct Node {
        // Parent (so we can backtrack through our path when we finished)
        Node* parent;
        Coordinate coords;
        // g = the current shortest path found from the start to end
        // h = a heuristic (estimate) of the distance between this node and goal node
        // f = g + h
        unsigned int g, h;
        // Constructor
        Node(Coordinate coords_, Node* parent_, unsigned int g_, unsigned int h_) : coords(coords_), parent(parent_), g(g_), h(h_) {};
    };
    // Type definition for unordered_map
    typedef std::unordered_map<Coordinate, Node*, coord_hash> nodeMap;
    // Admissible heuristics (ones that don't overestimate the cost of travelling from current node to goal node)
    class Heuristic {
    public:
        // Manhattan heuristic for grids that only allow 4 directions of movement
        static unsigned int manhattan(const Coordinate& start_, const Coordinate& goal_) {
            unsigned int dx = abs(start_.x - goal_.x), dy = abs(start_.y - goal_.y);
            return 10 * (dx + dy);
        }
        // Diagonal heuristic for grids that only allow 4 directions of movement
        static unsigned int diagonal(const Coordinate& start_, const Coordinate& goal_) {
            unsigned int dx = abs(start_.x - goal_.x), dy = abs(start_.y - goal_.y);
            return 10 * (dx + dy) - (6 * std::min(dx, dy));
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
        // Is coordinate valid? (is inside boundaries and isn't a wall)
        bool isValid(const Coordinate& coords_) const {
            return (coords_.x > -1 && coords_.y > -1 && coords_.x < mapSize_x && coords_.y < mapSize_y && walls.find(coords_) == walls.end());
        }
    };
    // Searching algo and its supplements
    class Search {
    private:
        //Reconstruct
        //We iterate through the parents of the nodes from the final node, since that is the shortest path
        //We add them to a vector of Coordinate, and that will be our final result.
        static std::vector<Coordinate> reconstruct(Node* current) {
            std::vector<Coordinate> result;
            result.push_back(current->coords);
            while (current->parent) {
                current = current->parent;
                result.push_back(current->coords);
            }
            return result;
        }
        //lowestHValue
        //We iterate through the open map to find the Node pointer that has the minimum H value in the map
        //Every pseudocode of A-star ever says that you must search for the lowest F, but I don't see the point in doing that when you already
        //precomputed H, so I'm doing this instead.
        static nodeMap::iterator lowestHValue(nodeMap& open_) {
            auto result = open_.begin();
            for (auto it = open_.begin(); it != open_.end(); it++) {
                if (it->second->h < result->second->h) result = it;
            }
            return result;
        }
    public:
        //findPath, the main algo
        static std::vector<Coordinate> findPath(const Coordinate& start_, const Coordinate& goal_, const bool& diagonal_, const Map& map) {
            if (map.mapSize_x <= 0  || map.mapSize_y <= 0) return {};
            if (!map.isValid(start_) || !map.isValid(start_)) return {};
            nodeMap closed, open;
            open[start_] = new Node(start_, nullptr, 0, diagonal_ ? Heuristic::diagonal(start_, goal_) : Heuristic::manhattan(start_, goal_));
            Node* current;
            Coordinate next;
            nodeMap::iterator it;
            short directions = (diagonal_) ? 8 : 4;
            unsigned int tentative;
            while (!open.empty()) {
                it = lowestHValue(open);
                current = it->second;
                if (current->coords == goal_) return reconstruct(current);
                closed[current->coords] = current;
                open.erase(it);
                for (short i = 0; i < directions; i++) {
                    next = current->coords + dir[i];
                    if (!map.isValid(next) || closed.find(next) != closed.end()) {
                        continue;
                    }
                    tentative = current->g + ((i < 4) ? 10 : 14);
                    if (open.find(next) == open.end()) {
                        open[next] = new Node(next, current, tentative, diagonal_ ? Heuristic::diagonal(next, goal_) : Heuristic::manhattan(next, goal_));
                    }
                    else if (tentative < open[next]->g) {
                        open[next]->parent = current;
                        open[next]->g = tentative;
                    }
                }
            }
            return {};
        }
    };
}