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

Graph::Graph(int _nNode) : nNodes(_nNode)
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
    for (int i = 0; i < nNodes; i++)
    {
        parents[i] = i;
        parent2child[i].push_back(i);
    }

    node2degree.resize(nNodes);
    bfsMark.resize(nNodes, false);

    L = unordered_set<int>();
    u2wL.resize(nNodes);
    uStats.resize(nNodes);
    wL2u = new MaxHeap(nNodes);
    UFAfterMerge.preallocate(nNodes, verbose);
}

void Graph::UnionFindAfterMerge::preallocate(int nNodes, int _verbose)
{
    verbose = _verbose;
    toMerge.clear();
    parentsAfterMerge.resize(nNodes);
    for (int i = 0; i < nNodes; i++)
    {
        parentsAfterMerge[i] = i;
    }
}
void Graph::addEdge(const int u, const int v, const WTYPE weight)
{
    if (u != v && u >= 0 && v >= 0 && u < nNodes && v < nNodes && weight > 0)
    {
        adjMatrix_graph[u][v] += weight;
        adjMatrix_graph[v][u] += weight;
    }
}

Graph::Graph(vector<vector<int>> _edges)
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

Graph::Graph(string filename)
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

void Graph::KCoreOptimization(
    const int K,
    vector<unordered_map<int, WTYPE>> &adjMatrix_graph,
    vector<int> &node2degree,
    const int verbose)
{
    int rmCount = 0;
    Timer tm;
    int nNodes = adjMatrix_graph.size();
    if (nNodes <= 0)
        return;

    fixDegree(adjMatrix_graph, node2degree, verbose);

    // cout << "K\t" << K << endl;
    // for (int i = 0; i < 10; i++)
    //     cout << node2degree[i] << endl;
    vector<int> tmpNode2RM;

    for (int i = 0; i < nNodes; i++)
    {
        if (node2degree[i] < K && !adjMatrix_graph[i].empty())
            tmpNode2RM.push_back(i);
    }

    while (!tmpNode2RM.empty())
    {
        vector<int> newNode2RM;
        for (auto &nodeRm : tmpNode2RM)
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
            rmCount++;
        }
        tmpNode2RM = move(newNode2RM);
    }
    if (verbose)
        cout << "KCoreOptimization removal:\t" << rmCount << endl;
    if (verbose)
        cout << "Time to KCoreOptimization:\t" << tm.CheckTimer() << endl;
}

void Graph::fixDegree(
    const vector<unordered_map<int, WTYPE>> &adjMatrix_graph,
    vector<WTYPE> &node2degree,
    const int verbose)
{
    Timer tm;
    int nNodes = adjMatrix_graph.size();
    if (nNodes <= 0)
        return;
    if (node2degree.size() != nNodes)
        node2degree.assign(nNodes, 0);

    for (int i = 0; i < nNodes; i++)
    {
        node2degree[i] = 0;
        for (auto &p : adjMatrix_graph[i])
            node2degree[i] += p.second;
    }
    if (verbose)
        cout << "Time to fix degree:\t" << tm.CheckTimer() << endl;
}

void Graph::getConnectedComp(
    vector<pair<int, int>> &seedNSize,
    const vector<unordered_map<int, WTYPE>> &adjMatrix_graph,
    vector<bool> &bfsMark,
    vector<vector<int>> &kCC,
    const bool returnKCC,
    const int verbose)
{
    Timer tm;
    int nNodes = adjMatrix_graph.size();
    if (nNodes <= 0)
        return;
    if (returnKCC)
        kCC.clear();
    seedNSize.clear();
    fill(begin(bfsMark), end(bfsMark), false);

    for (int i = 0; i < nNodes; i++)
    {
        if (bfsMark[i] || adjMatrix_graph[i].empty())
        {
            bfsMark[i] = true;
        }
        else
        {
            if (returnKCC)
                kCC.push_back(vector<int>());
            int seed = i;
            bfsMark[i] = true;
            int ncomp = 1;
            stack<int> s;
            s.push(i);
            while (!s.empty())
            {
                int curr = s.top();
                if (returnKCC)
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
            seedNSize.emplace_back(seed, ncomp);
        }
    }
    if (verbose)
        cout << "Time to getConnected Components:\t" << tm.CheckTimer() << endl;
    return;
}

void Graph::getKCore(const int _K)
{
    K = _K;
    if (verbose)
        cout << "prepare K core" << endl;
    KCoreOptimization(K, adjMatrix_graph, node2degree, verbose);
    vector<pair<int, int>> tmp;
    getConnectedComp(tmp, adjMatrix_graph, bfsMark, kCC, true, verbose);
    if (verbose)
        cout << "return connected components" << endl;
    sortKCC();
}

void Graph::getKCC(const int _K)
{
    K = _K;
    Timer tm1;
    depth = 0;
    while (true)
    {
        UFAfterMerge.preallocate(nNodes, verbose);
        for (int i = 0; i < nNodes; i++)
        {
            parents[i] = i;
            parent2child[i].clear();
            parent2child[i].push_back(i);
        }
        fill(u2wL.begin(), u2wL.end(), 0);
        fill(uStats.begin(), uStats.end(), 0);

        depth++;
        KCoreOptimization(K, adjMatrix_graph, node2degree, verbose);

        // getConencted Components
        vector<pair<int, int>> seedNSize;
        getConnectedComp(seedNSize, adjMatrix_graph, bfsMark, kCC, false, verbose);
        if (verbose)
        {
            cout << "# of CC.:\t" << seedNSize.size() << endl;
        }

        if (seedNSize.empty())
            break;
        adjMatrix_kcut = adjMatrix_graph;
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

        UFAfterMerge.toMerge.clear();

        Timer tmForkMas;
        int status = kMas(seed, ncomp);
        timeForKMas += tmForkMas.CheckTimer();

        if (verbose)
            cout << "Early Merging Count:\t" << UFAfterMerge.toMerge.size()
                 << "\nStatus:\t" << status
                 << "\nCurrunt KCC. size:\t" << kCC.size() << endl;
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

        Timer tmForMerge;
        int tmpSeed = mergeAll(ncomp);
        timeForMerge += tmForMerge.CheckTimer();

        if (tmpSeed >= 0)
            seed = tmpSeed;
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
    unordered_set<int> nodeInL;
    getNodesInL(nodeInL);

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
    int seed = -1;
    while (!UFAfterMerge.toMerge.empty())
    {
        if (verbose > 2)
            cout << "Remaining pairs to merge:\t" << UFAfterMerge.toMerge.size() << endl;

        int status = mergeNodes();
        if (status >= 0)
        {
            ncomp--;
            seed = status;
        }
    }
    return seed;
}

int Graph::earlyStop(const WTYPE currCut,
                     const int currNNode,
                     const int nMerge)
{
    if (currCut < K && L.size() < currNNode)
    {
        // count number of merges in L
        // remove the cut L vs V/L
        if (nMerge + 1 == L.size() && L.size() > 1) // also add L to ret directly
            return -3;
        else
            return -2;
    }
    return 0;
}

// return 0  for normal exit
// return -1 for non-valid input
// return -2 for earlyStop and Remove Cut
// return -3 for earlyStop and add L to KCC
int Graph::kMas(int &prevNode, const int currNNode)
{
    if (currNNode <= 1)
        return -1;

    L.clear();
    // prevNode from a node
    int nMerge = 0;
    int oldStartNode = prevNode;
    u2wL[prevNode] = 0;

    // set<pair<int, int>> wL2u;
    wL2u->size = 0;
    L.insert(prevNode);
    uStats[prevNode] = 1;

    int currCut = 0;
    for (auto &p : adjMatrix_kcut[prevNode])
    {
        int node = p.first;
        WTYPE weight = p.second;
        currCut += weight;
        u2wL[node] = weight;
        wL2u->insert(weight, node + 1);
    }

    // early stop
    int earlyStopStatus;
    if ((earlyStopStatus = earlyStop(currCut, currNNode, nMerge)) != 0)
        return earlyStopStatus;

    vector<int> toAdd;
    while (L.size() < currNNode)
    {
        // find the next vertex to add;
        cntVisit++;
        int MA = wL2u->getMaxKey();
        int weight = wL2u->getMax();
        if (weight >= K)
            UFAfterMerge.recordMerge(MA, prevNode);

        if (verbose > 2)
        {
            cout << "\nprevNode:\t" << prevNode << endl;
            cout << "Cut Before:\t" << currCut << endl;
            cout << "|L|:\t" << L.size() << " of " << currNNode << endl;
            cout << "u:\t" << MA << "\nw(L,u):\t" << weight << endl;
        }

        toAdd.push_back(MA);
        uStats[MA] = 2;
        u2wL[MA] = 0;

        wL2u->removeMax();
        prevNode = MA;

        while (batchM && wL2u->size > 0 && wL2u->getMax() >= K)
        {
            MA = wL2u->getMaxKey();
            weight = wL2u->getMax();
            if (verbose > 2)
                cout << "kMas++: add:\t" << MA << endl;
            UFAfterMerge.recordMerge(MA, prevNode);
            nMerge++;
            prevNode = MA;

            toAdd.push_back(MA);
            uStats[MA] = 2;
            u2wL[MA] = 0;
            wL2u->removeMax();
        }

        // for (auto &&s : u2wL)
        //     cout << s << " ";
        // cout << endl;
        for (auto &MA : toAdd)
        {

            for (auto &p : adjMatrix_kcut[MA])
            {
                WTYPE weightTmp = p.second;
                int node = p.first;
                if (uStats[node] == 1)
                {
                    // cout << node << "-" << MA << endl;
                    currCut -= weightTmp;
                }
                else if (uStats[node] == 0)
                {
                    // cout << node << "+" << MA << endl;
                    currCut += weightTmp;
                    if (u2wL[node] > 0)
                        wL2u->delEle(node + 1);

                    u2wL[node] += weightTmp;
                    wL2u->insert(u2wL[node], node + 1);
                }
                // if (L.count(node))
                // {
                //     if (!u2wL[node])
                //         currCut -= weightTmp;
                // }
                // else
                // {
                //     currCut += weightTmp;
                //     if (u2wL[node] > 0)
                //         wL2u->delEle(node + 1);

                //     u2wL[node] += weightTmp;
                //     wL2u->insert(u2wL[node], node + 1);
                // }
            }
        }
        for (auto &MA : toAdd)
        {
            L.insert(MA);
            uStats[MA] = 1;
        }

        toAdd.clear();
        // early stop
        if ((earlyStopStatus = earlyStop(currCut, currNNode, nMerge)) != 0)
            return earlyStopStatus;
    }
    for (auto &&s : L)
        uStats[s] = 0;
    return 0;
}

// return -1; no merging happenned
// return other; the remaining node
int Graph::mergeNodes()
{
    auto top = UFAfterMerge.toMerge.back();
    UFAfterMerge.toMerge.pop_back();

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
        int tmpweight = (adjMatrix_kcut[v][node] += weight);
        if (tmpweight >= K && forceC)
            UFAfterMerge.recordMerge(v, node);

        adjMatrix_kcut[node].erase(u);
        adjMatrix_kcut[node][v] += weight;
    }
    adjMatrix_kcut[u].clear();
    return v;
}

void Graph::UnionFindAfterMerge::recordMerge(const int u, const int v)
{
    if (findParentsAfterMerge(u) == findParentsAfterMerge(v))
        return;

    if (verbose > 2)
        cout << "record merge:\t" << v << " " << u << endl;
    mergeParentsAfterMerge(u, v);
    toMerge.emplace_back(u, v);
}

void Graph::UnionFindAfterMerge::mergeParentsAfterMerge(const int v, const int u)
{
    int p1 = findParentsAfterMerge(v);
    int p2 = findParentsAfterMerge(u);
    if (p1 != p2)
    {
        parentsAfterMerge[p1] = p2;
    }
}

int Graph::UnionFindAfterMerge::findParentsAfterMerge(const int v)
{
    if (parentsAfterMerge[v] == v)
        return v;
    else
    {
        int p = parentsAfterMerge[v];
        return parentsAfterMerge[v] = findParentsAfterMerge(p);
    }
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
