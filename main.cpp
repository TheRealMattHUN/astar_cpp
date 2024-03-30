#include <iostream>
#include <chrono>
#include "source/astar.h"

//Example

using namespace std;
int main() {
    A_star::Coordinate start, goal, temp;
    int sizex, sizey, w, h;
    bool diagonal;
    A_star::Map map;
    cout << "Size:\n";
    cin >> sizex >> sizey;
    map.setWorldSize(sizex, sizey);
    cout << "Start:\n";
    cin >> start.x >> start.y;
    cout << "Goal:\n";
    cin >> goal.x >> goal.y;
    cout << "Diagonal:\n";
    cin >> diagonal;
    map.setDiagonal(diagonal);
    cout << "Heuristic:\nManhattan: 1\nDiagonal: 2\nEuclidean: 3\n";
    cin >> h;
    if (h == 2) map.setHeuristic(A_star::Heuristic::diagonal);
    else if (h == 3) map.setHeuristic(A_star::Heuristic::euclidean);
    else map.setHeuristic(A_star::Heuristic::manhattan);
    cout << "Number of walls:\n";
    cin >> w;
    for (int i = 0; i < w; i++) {
        cin >> temp.x >> temp.y;
        map.addWall(temp);
    }
    auto s = chrono::high_resolution_clock::now();
    std::vector<A_star::Coordinate> path = A_star::Search::findPath(start, goal, map);
    auto e = chrono::high_resolution_clock::now();
    cout << "Time:\n" << chrono::duration_cast<chrono::nanoseconds>(e-s).count();
}
