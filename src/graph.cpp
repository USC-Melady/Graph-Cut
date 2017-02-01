#include "../include/graph.h"
// #include "graph.h"
#include <vector>
#include <iostream>
#include <algorithm>
#include <set>
#include <map>
#include <unordered_set>
#include <stack>
#include <fstream>
#include <chrono>

using namespace std;

void Graph::sortKCC()
{
    for (auto &v : kCC)
    {
        sort(v.begin(), v.end());
    }
    sort(kCC.begin(), kCC.end());
}

Graph::Graph(int _nNode, int _K) : nNodes(_nNode), K(_K)
{
    preallocate();
}

int Graph::findParent(const int v)
{
    if (parents[v] == v)
        return v;
    else
    {
        int p = parents[v];
        return parents[v] = findParent(p);
    }
}

void Graph::preallocate()
{
    kCC = vector<vector<int>>();
    adjMatrix_graph.resize(nNodes);
    adjMatrix_kcut.resize(nNodes);

    parent2child.resize(nNodes);
    parents.resize(nNodes);
    parentsAfterMerge.resize(nNodes);
    for (int i = 0; i < nNodes; i++)
    {
        parentsAfterMerge[i] = i;
        parents[i] = i;
        parent2child[i].push_back(i);
    }

    node2degree.resize(nNodes);
    // weight_node = set<pair<WTYPE, int>>();

    bfsMark.resize(nNodes, false);

    L = unordered_set<int>();
    u2wL.resize(nNodes);
    wL2u = new MaxHeap(nNodes);
    toMerge = vector<pair<int, int>>();
}

void Graph::addEdge(const int u, const int v, const WTYPE weight)
{
    if (u != v && u >= 0 && v >= 0 && u < nNodes && v < nNodes && weight > 0)
    {
        adjMatrix_graph[u][v] += weight;
        adjMatrix_graph[v][u] += weight;
    }
}

Graph::Graph(vector<vector<int>> _edges, int _K) : K(_K)
{
    Timer tm;
    nNodes = 0;
    for (auto &v : _edges)
    {
        nNodes = max(nNodes, v[0]);
        nNodes = max(nNodes, v[1]);
    }
    nNodes++;
    preallocate();
    for (auto &t : _edges)
        addEdge(t[0], t[1]);

    if (verbose)
        cout << "Time for Construct the graph: " << tm.CheckTimer() << endl;
    return;
}

Graph::Graph(string filename, int _K) : K(_K)
{
    fstream fin;
    fin.open(filename.c_str());
    int nedge = 0;
    fin >> nNodes >> nedge;
    preallocate();

    for (int i = 0; i < nedge; i++)
    {
        int u, v;
        fin >> u >> v;
        addEdge(u, v);
    }
    fin.close();
    return;
}

void Graph::KCoreOptimization()
{
    Timer tm;
    fixDegree();
    // unordered_map<int, int> node2delta;

    vector<int> tmpNode2RM;

    for (int i = 0; i < nNodes; i++)
    {
        if (node2degree[i] < K && !adjMatrix_graph[i].empty())
            tmpNode2RM.push_back(i);
    }

    while (!tmpNode2RM.empty())
    {
        vector<int> newNode2RM;
        for (auto &nodeRm : newNode2RM)
        {

            for (const auto &edge : adjMatrix_graph[nodeRm])
            {
                int weight = edge.second;
                int NB = edge.first;
                node2degree[NB] -= weight;
                if (node2degree[NB] < K && node2degree[NB] + weight >= K)
                    newNode2RM.push_back(NB);
                // remove the opposite edge
                adjMatrix_graph[NB].erase(nodeRm);
            }
            node2degree[nodeRm] = 0;
            adjMatrix_graph[nodeRm].clear();
        }
        tmpNode2RM = move(newNode2RM);
    }

    if (verbose)
        cout << "Time to KCoreOptimization:\t" << tm.CheckTimer() << endl;
}

void Graph::fixDegree()
{
    Timer tm;
    // weight_node.clear();
    for (int i = 0; i < nNodes; i++)
    {
        node2degree[i] = 0;
        for (auto &p : adjMatrix_graph[i])
            node2degree[i] += p.second;
        // if (node2degree[i])
        //     weight_node.insert(make_pair(node2degree[i], i));
    }
    if (verbose)
        cout << "Time to fix degree:\t" << tm.CheckTimer() << endl;
}

void Graph::editDegree(const int node, const WTYPE delta)
{
    if (delta == 0)
        return;

    WTYPE oldweight = node2degree[node];
    // if (oldweight != 0)
    //     weight_node.erase(make_pair(oldweight, node));
    if (oldweight + delta < -EPS)
        cout << "can not decrease degree to negative!" << endl;
    node2degree[node] += delta;
    // if (oldweight + delta > EPS)
    //     weight_node.insert(make_pair(oldweight + delta, node));
}

vector<pair<int, int>> Graph::getConnectedComp()
{
    Timer tm;
    vector<pair<int, int>> seedNSize;
    fill(bfsMark.begin(), bfsMark.end(), false);
    for (int i = 0; i < nNodes; i++)
    {
        if (bfsMark[i])
        {
            continue;
        }
        else if (adjMatrix_graph[i].empty())
        {
            bfsMark[i] = true;
        }
        else
        {
            int seed = i;
            bfsMark[i] = true;
            int ncomp = 1;
            stack<int> s;
            s.push(i);
            while (!s.empty())
            {
                int curr = s.top();
                s.pop();
                for (auto &p : adjMatrix_graph[curr])
                {
                    int next = p.first;
                    if (!bfsMark[next])
                    {
                        s.push(next);
                        bfsMark[next] = true;
                        ncomp++;
                    }
                }
            }
            seedNSize.emplace_back(seed, ncomp);
        }
    }
    if (verbose)
        cout << "Time to getConnected Components:\t" << tm.CheckTimer() << endl;
    return seedNSize;
}

void Graph::returnConnectedComp()
{
    kCC.clear();
    fill(bfsMark.begin(), bfsMark.end(), false);
    for (int i = 0; i < nNodes; i++)
    {
        if (bfsMark[i] || adjMatrix_graph[i].empty())
        {
            bfsMark[i] = true;
        }
        else
        {
            kCC.push_back(vector<int>());
            int seed = i;
            bfsMark[i] = true;
            int ncomp = 1;
            stack<int> s;
            s.push(i);
            while (!s.empty())
            {
                int curr = s.top();
                kCC.back().push_back(curr);
                s.pop();
                for (auto &p : adjMatrix_graph[curr])
                {
                    int next = p.first;
                    if (!bfsMark[next])
                    {
                        s.push(next);
                        bfsMark[next] = true;
                        ncomp++;
                    }
                }
            }
        }
    }
}

void Graph::getKCore()
{
    if (verbose)
        cout << "prepare K core" << endl;
    KCoreOptimization();
    returnConnectedComp();
    if (verbose)
        cout << "return connected components" << endl;
    sortKCC();
}

void Graph::getKCC()
{
    Timer tm1;
    depth = 0;
    while (true)
    {
        for (int i = 0; i < nNodes; i++)
        {
            parentsAfterMerge[i] = i;
            parents[i] = i;
            parent2child[i].push_back(i);
        }
        fill(u2wL.begin(), u2wL.end(), 0);

        depth++;
        KCoreOptimization();

        // getConencted Components
        auto seedNSize = getConnectedComp();
        if (verbose)
            cout << "# of CC.:\t" << seedNSize.size() << endl;

        if (seedNSize.empty())
            break;
        for (auto &p : seedNSize)
        {
            int seed = p.first;
            int ncomp = p.second;
            kCut(seed, ncomp);
        }
    }
    sortKCC();
}

// return 0, k-cut found
// return 1, no k-cut found, add to solution
// global variable used
// L, adjMatrix_kcut, adjMatrix_graph, toMerge
int Graph::kCut(int &seed, int &ncomp)
{
    adjMatrix_kcut = adjMatrix_graph;
    int forceCntrCnt = 0;
    int earlyMergingCnt = 0;
    if (verbose)
        cout << "Seed:\t" << seed << "\nSize of Comp.:\t" << ncomp << endl;
    while (true)
    {
        if (verbose)
        {
            cout << "decomposition depth:" << depth
                 << "\t seed:" << seed
                 << "\t remainning size:\t" << ncomp
                 << "\tForce Contracted:\t" << forceCntrCnt
                 << "\tEarly Merging:\t" << earlyMergingCnt << endl;
        }

        toMerge.clear();

        int status = kMas(seed, ncomp);
        if (verbose)
            cout << "Early Merging Count:\t" << toMerge.size() << "\nStatus:\t" << status << endl;
        // end of KCut remove cut
        if (status == -3) //add L to return
        {
            addLtoRet();
            return 0;
        }
        else if (status == -2)
        {
            removeCutL();
            return 0;
        }

        seed = mergeAll(ncomp);
        if (status == 0 && ncomp > 1)
        {
            continue;
        }
        else if (ncomp == 1)
        {
            addLtoRet();
            return 1;
        }
        else
        {
            cout << "ERROR! NOT POSSIBLE in KCut" << endl;
            cout << "Ncomp\t" << ncomp << endl;
            exit(1);
        }
    }
}

// global variable used
// L, parent2child
void Graph::getNodesInL(unordered_set<int> &nodeInL)
{
    for (auto &v : L)
        nodeInL.insert(parent2child[v].begin(), parent2child[v].end());
    return;
}

// global variable used
// L, parent2child
void Graph::getNodesInL(vector<int> &nodeInL)
{
    for (auto &v : L)
        nodeInL.insert(nodeInL.end(), parent2child[v].begin(), parent2child[v].end());
    return;
}

// global variable used
// kCC, L, parent2child, adjMatrix_graph
void Graph::addLtoRet()
{
    unordered_set<int> nodeInL;
    getNodesInL(nodeInL);
    kCC.emplace_back(nodeInL.begin(), nodeInL.end());

    for (auto &v : kCC.back())
    {
        // remove edges outside of VL on adjMatrix_graph
        for (auto &p : adjMatrix_graph[v])
        {
            int u = p.first;
            if (nodeInL.count(u) == 0)
                adjMatrix_graph[u].erase(v);
        }
        adjMatrix_graph[v].clear();
    }
}

void Graph::removeCutL()
{
    // cout << "1" << endl;
    unordered_set<int> nodeInL;
    // cout << "2" << endl;
    getNodesInL(nodeInL);
    // cout << "3" << endl;

    if (verbose > 2)
    {
        cout << "node in L:" << endl;
        for (auto &v : nodeInL)
            cout << v << " ";
        cout << endl;
    }
    for (auto &v : nodeInL)
    {
        // remove edges outside of VL on adjMatrix_graph
        if (verbose > 2)
            cout << "checking node:\t" << v << endl;
        vector<int> toRemove;
        for (auto &p : adjMatrix_graph[v])
        {
            int u = p.first;
            // cout << "checking neigbour:\t" << u << endl;
            if (nodeInL.count(u) == 0)
            {
                toRemove.push_back(u);
                adjMatrix_graph[u].erase(v);
            }
        }
        for (auto &u : toRemove)
            adjMatrix_graph[v].erase(u);
    }
}

// ncomp is the size of components after merge
// return the seed of the merged subgraph
int Graph::mergeAll(int &ncomp)
{
    int seed;
    while (!toMerge.empty())
    {
        if (verbose > 1)
            cout << "Remaining pairs to merge:\t" << toMerge.size() << endl;

        int status = mergeNodes();
        if (status >= 0)
        {
            ncomp--;
            seed = status;
        }
    }
    return seed;
}

// return 0  for normal exit
// return -1 for non-valid input
// return -2 for earlyStop and Remove Cut
// return -3 for earlyStop and add L to KCC
int Graph::kMas(int &prevNode, int &currNNode)
{
    if (currNNode <= 1)
        return -1;

    // prevNode from a node
    int nMerge = 0;
    int oldStartNode = prevNode;
    // set<pair<int, int>> wL2u;
    wL2u->size = 0;

    L.clear();
    L.insert(prevNode);

    int currCut = 0;
    for (auto &p : adjMatrix_kcut[prevNode])
    {
        int node = p.first;
        WTYPE weight = p.second;
        currCut += weight;
        u2wL[node] += weight;
        wL2u->insert(weight, node + 1);
    }

    while (L.size() < currNNode)
    {
        // find the next vertex to add;
        int MA = wL2u->getMaxKey();
        int weight = wL2u->getMax();
        if (weight >= K)
            recordMerge(MA, prevNode);

        if (verbose > 2)
        {
            cout << "\nprevNode:\t" << prevNode << endl;
            cout << "Cut Before:\t" << currCut << endl;
            cout << "|L|:\t" << L.size() << " of " << currNNode << endl;
            cout << "u:\t" << MA << "\nw(L,u):\t" << weight << endl;
        }

        unordered_set<int> toAdd;
        toAdd.insert(MA);
        L.insert(MA);
        u2wL[MA] = 0;
        wL2u->removeMax();
        prevNode = MA;

        // if (verbose)
        //     cout << "wL2u->size\t" << wL2u->size << "\nwL2u->getMax()\t" << wL2u->getMax() << endl;
        while (wL2u->size > 0 && wL2u->getMax() >= K)
        {
            MA = wL2u->getMaxKey();
            weight = wL2u->getMax();
            if (verbose > 2)
                cout << "kMas++: add:\t" << MA << endl;
            recordMerge(MA, prevNode);
            nMerge++;
            prevNode = MA;

            L.insert(MA);
            toAdd.insert(MA);
            wL2u->removeMax();
        }

        for (auto &MA : toAdd)
        {
            u2wL[MA] = 0;
            for (auto &p : adjMatrix_kcut[MA])
            {
                WTYPE weightTmp = p.second;
                int node = p.first;
                if (L.count(node))
                {
                    if (toAdd.count(node) == 0)
                        currCut -= weightTmp;
                }
                else
                {
                    currCut += weightTmp;
                    if (u2wL[node] > 0)
                        wL2u->delEle(node + 1);

                    u2wL[node] += weightTmp;
                    wL2u->insert(u2wL[node], node + 1);
                }
            }
        }

        // early stop
        if (currCut < K && L.size() < currNNode)
        {
            // count number of merges in L
            // remove the cut L vs V/L
            if (nMerge + 1 == L.size()) // also add L to ret directly
                return -3;
            else
                return -2;
        }
    }
    return 0;
}
void Graph::mergeParentsAfterMerge(const int v, const int u)
{
    int p1 = findParentsAfterMerge(v);
    int p2 = findParentsAfterMerge(u);
    if (p1 != p2)
    {
        parentsAfterMerge[p1] = p2;
    }
}
int Graph::findParentsAfterMerge(const int v)
{
    if (parentsAfterMerge[v] == v)
        return v;
    else
    {
        int p = parentsAfterMerge[v];
        return parentsAfterMerge[v] = findParentsAfterMerge(p);
    }
}

// return -1; no merging happenned
// return other; the remaining node
int Graph::mergeNodes()
{
    auto top = toMerge.back();
    toMerge.pop_back();

    int v = top.first;
    int u = top.second;

    int p1 = findParent(v);
    int p2 = findParent(u);
    if (p1 == p2) // they are already merged together
        return -1;

    if (verbose > 2)
        cout << "merge:\t" << v << " " << u << endl;
    v = findParent(v);
    u = findParent(u);
    // merge the smaller to larger
    if (adjMatrix_kcut[u].size() > adjMatrix_kcut[v].size())
        swap(v, u);

    //merge u to v
    parents[u] = v;

    parent2child[v].splice(parent2child[v].begin(), parent2child[u]);
    if (verbose > 2)
        cout << v << " contains:" << parent2child[v].size() << endl;

    adjMatrix_kcut[v].erase(u);
    //go through all edges of u
    for (auto &p : adjMatrix_kcut[u])
    {
        int node = p.first;
        int weight = p.second;
        // add to v
        if (weight == 0 || node == v)
            continue;
        adjMatrix_kcut[v][node] += weight;
        if (adjMatrix_kcut[v][node] >= K && forceC)
            recordMerge(v, node);

        adjMatrix_kcut[node].erase(u);
        adjMatrix_kcut[node][v] += weight;
    }
    adjMatrix_kcut[u].clear();
    return v;
}

void Graph::recordMerge(const int u, const int v)
{
    if (findParentsAfterMerge(u) == findParentsAfterMerge(v))
        return;

    if (verbose > 2)
        cout << "record merge:\t" << v << " " << u << endl;
    mergeParentsAfterMerge(u, v);
    toMerge.emplace_back(u, v);
}

// void Graph::Ret2File(string filename)
// {
//     ofstream fout;
//     fout.open(filename.c_str());
//     if (fout.is_open())
//     {
//         fout << "Number of Components:\t" << kCC.size() << endl;
//         int ind = 0;
//         for (auto &v : kCC)
//         {
//             fout << "Comp. " << ind++ << " size:\t" << v.size() << endl;
//         }

//         for (auto &v : kCC)
//         {
//             sort(v.begin(), v.end());
//             // cout << "Components:\n";
//             for (auto &i : v)
//             {
//                 fout << i << " ";
//             }
//             fout << endl;
//         }
//         fout.close();
//     }
//     else
//     {
//         cout << "unable to open file" << endl;
//     }
//     return;
// }

void Graph::printGraph()
{
    cout << "Print Graph:" << endl;
    for (int i = 0; i < nNodes; i++)
    {
        for (auto &p : adjMatrix_graph[i])
        {
            cout << i << "<->" << p.first << ":" << p.second << endl;
        }
    }
}

void Graph::printGraphKCut()
{
    cout << "Print Graph kCut:" << endl;
    for (int i = 0; i < nNodes; i++)
    {
        for (auto &p : adjMatrix_kcut[i])
        {
            cout << i << "<->" << p.first << ":" << p.second << endl;
        }
    }
}
