#include <iostream>
#include <cmath>
#include <string>
#include "../include/graph.h"

using namespace std;

string getDataName(string s)
{
    auto i = s.find_last_of('/');
    auto j = s.find_last_of('.');
    return s.substr(i + 1, j - i - 1);
}
int main(int argc, char **argv)
{

    Graph g(argv[1]);
    // cout << "Graph loaded" << endl;
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

    g.verbose = stoi(argv[2]);
    Timer tm;
    g.forceC = stoi(argv[4]);
    g.batchM = stoi(argv[5]);
    g.getKCC(stoi(argv[3]));

    cout << getDataName(string(argv[1])) << "\t";
    cout << argv[3] << "\t";
    if (g.batchM)
        cout << "Yes\t";
    else
        cout << "No\t";

    if (g.forceC)
        cout << "Yes\t";
    else
        cout << "No\t";
    cout << g.timeForMerge << "\t";
    cout << g.timeForKMas << "\t";
    cout << tm.CheckTimer() << "\t";
    cout << endl;
    // cout << "Time for KCC: " << tm.CheckTimer() << endl;
    // cout << g.kCC.size() << endl;
    // for (auto &v : g.kCC)
    // {
    //     cout << v[0] << endl;
    //     // for (auto &i : v)
    //     // {
    //     //     cout << i << " ";
    //     // }
    //     // cout << endl;
    // }
    // cout << "Time for merge:\t" << g.timeForMerge << endl;
    // cout << "Time for kMas:\t" << g.timeForKMas << endl;
    // cout << "Number of Mas Visit:\t" << g.cntVisit << endl;
    // cout << "Number of update kMas Saved:\t" << g.cntKMasSave << endl;
    return 0;
}