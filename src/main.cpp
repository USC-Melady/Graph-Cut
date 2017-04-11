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
    cout << "arguments:\n";
    for (int i = 0; i < argc; i++)
        cout << i << ":\t" << argv[i] << endl;

    Graph g(argv[1]);

    g.verbose = stoi(argv[2]);
    Timer tm;
    g.forceC = stoi(argv[4]);
    g.batchM = stoi(argv[5]);

    // g.printGraph();
    g.getKCC((WTYPE)stof(argv[3]));

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

    cout << "Time for KCC: " << tm.CheckTimer() << endl;
    cout << g.kCC.size() << endl;
    for (auto &v : g.kCC)
    {
        cout << v.size() << endl;
        // for (auto &i : v)
        // {
        //     cout << i << " ";
        // }
        // cout << endl;
    }
    cout << "Time for merge:\t" << g.timeForMerge << endl;
    cout << "Time for kMas:\t" << g.timeForKMas << endl;
    cout << "Number of Mas Visit:\t" << g.cntVisit << endl;
    return 0;
}