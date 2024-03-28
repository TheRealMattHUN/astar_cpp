#include <iostream>
#include <chrono>
#include "source/astar.h"

//Example

using namespace std;
int main() {
    A_star::Coordinate start, goal, temp;
    int w;
    bool diagonal;
    A_star::Map map;
    cout << "Size:\n";
    cin >> map.mapSize_x >> map.mapSize_y;
    cout << "Start:\n";
    cin >> start.x >> start.y;
    cout << "Goal:\n";
    cin >> goal.x >> goal.y;
    cout << "Diagonal:\n";
    cin >> diagonal;
    cout << "Number of walls:\n";
    cin >> w;
    for (int i = 0; i < w; i++) {
        cin >> temp.x >> temp.y;
        map.addWall(temp);
    }
    auto s = chrono::high_resolution_clock::now();
    auto path = A_star::Search::findPath(start, goal, diagonal, map);
    auto e = chrono::high_resolution_clock::now();
    for (auto& elem : path) cout << elem.x << "   " << elem.y << endl;
    cout << "Time:\n" << chrono::duration_cast<chrono::nanoseconds>(e-s).count();
}
