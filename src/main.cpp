#include <iostream>
#include <cmath>
#include "../include/graph.h"

using namespace std;

int main(int argc, char **argv)
{

    Graph g(argv[1]);
    cout << "Graph loaded" << endl;
    // g.addEdge(0, 1, 8);
    // g.addEdge(1, 2, 3);
    // g.addEdge(2, 3, 4);

    // g.addEdge(0, 4, 3);
    // g.addEdge(1, 4, 2);
    // g.addEdge(1, 5, 2);
    // g.addEdge(2, 6, 2);
    // g.addEdge(3, 6, 2);
    // g.addEdge(3, 7, 2);

    // g.addEdge(4, 5, 3);
    // g.addEdge(5, 6, 1);
    // g.addEdge(6, 7, 3);

    g.verbose = 1;
    Timer tm;
    g.forceC = stoi(argv[3]);
    g.batchM = stoi(argv[4]);
    g.getKCC(stoi(argv[2]));
    if (g.verbose)
        cout << "Time for KCC: " << tm.CheckTimer() << endl;
    cout << g.kCC.size() << endl;
    // for (auto &v : g.kCC)
    // {
    //     cout << v.size() << endl;
    //     // for (auto &i : v)
    //     // {
    //     //     cout << i << " ";
    //     // }
    //     // cout << endl;
    // }
    cout << "Time for merge:\t" << g.timeForMerge << endl;
    cout << "Time for kMas:\t" << g.timeForKMas << endl;
    cout << "Number of Mas Visit:\t" << g.cntVisit << endl;
    // cout << "Number of update kMas Saved:\t" << g.cntKMasSave << endl;
    return 0;
}