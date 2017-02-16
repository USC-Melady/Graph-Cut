#pragma once

#include <vector>
#include <iostream>
#include <algorithm>
#include <set>
#include <map>
#include <list>
#include <unordered_set>
#include <unordered_map>
#include <climits>
#include <string>
#include <chrono>

using namespace std;

typedef int WTYPE;
const double EPS = 0.00000001;

class Graph
{
  public:
    // non-trival components to return
    vector<vector<int>> kCC;
    bool forceC = true;
    bool batchM = true;
    unsigned cntVisit = 0;
    // unsigned cntKMasSave = 0;

    double timeForKMas = 0;
    double timeForMerge = 0;

    void getKCC(const int _K);
    void getKCore(const int _K);

    void sortKCC();

    Graph(int _nNode);
    Graph(string filename);
    Graph(vector<vector<int>> _edges);
    ~Graph() { delete wL2u; }

    int verbose;

    static void KCoreOptimization(
        const int K,
        vector<unordered_map<int, WTYPE>> &adjMatrix_graph,
        vector<int> &node2degree,
        const int verbose);

    //   private:
    int nNodes;
    int K;
    // Algorithm Analytic
    int depth = 0;

    // Utils for KCore
    static void fixDegree(
        const vector<unordered_map<int, WTYPE>> &adjMatrix_graph,
        vector<WTYPE> &node2degree,
        const int verbose);

    // Utils for KCC
    static void getConnectedComp(
        vector<pair<int, int>> &seedNSize,
        const vector<unordered_map<int, WTYPE>> &adjMatrix_graph,
        vector<bool> &bfsMark,
        vector<vector<int>> &kCC,
        const bool returnKCC,
        const int verbose);

    int kCut(int &currNNode, int &prevNode);
    int kMas(int &seed, const int currNNode);

    int earlyStop(const WTYPE currCut, const int currNNode, const int nMerge);

    int mergeAll(int &ncomp);
    int findParent(const int v);
    int mergeNodes();
    void removeCutL();
    void addLtoRet();
    void getNodesInL(vector<int> &nodeInL);
    void getNodesInL(unordered_set<int> &nodeInL);

    void addEdge(const int u, const int v, const WTYPE weight = 1);

    // used for k-cut+
    vector<unordered_map<int, WTYPE>> adjMatrix_kcut;
    // used for kCC
    vector<unordered_map<int, WTYPE>> adjMatrix_graph;

    // Pre-allocated Variables
    void preallocate();
    vector<int> node2degree;
    vector<list<int>> parent2child;
    vector<int> parents;

    vector<bool> merged;
    unordered_set<int> L;
    vector<bool> bfsMark;
    vector<WTYPE> u2wL;
    vector<int> uStats;
    // note that this not consistent with the actual parenthood
    // it is only guaranteed to be consistent with the membership
    // after all pending merging operations complete.
    struct UnionFindAfterMerge
    {
        UnionFindAfterMerge(){};
        int verbose;
        void preallocate(int nNodes, int verbose);

        // variable for KCC
        vector<pair<int, int>> toMerge;
        vector<int> parentsAfterMerge;
        void recordMerge(const int u, const int v);

        void mergeParentsAfterMerge(const int v, const int u);
        int findParentsAfterMerge(const int v);

    } UFAfterMerge;

    // heap
    struct MaxHeap //miniheap
    {
        int *maxContents;      //from 1
        int *maxElementToHeap; //from 1
        int *maxHeapToElement; //from 1

        int size;
        MaxHeap(int MAXSIZE)
        {
            size = 0;
            maxContents = new int[MAXSIZE + 1];
            maxElementToHeap = new int[MAXSIZE + 1];
            maxHeapToElement = new int[MAXSIZE + 1];
        }
        ~MaxHeap()
        {
            delete[] maxContents;
            delete[] maxElementToHeap;
            delete[] maxHeapToElement;
        }

        void insert(int content, int order)
        {
            size++;
            maxContents[size] = content;
            maxElementToHeap[order] = size;
            maxHeapToElement[size] = order;
            siftUp(size);
        }

        WTYPE getKey(int node) { return maxContents[maxElementToHeap[node]]; }

        int getMax() { return maxContents[1]; }
        int getMaxKey()
        {
            return maxHeapToElement[1] - 1;
        }
        void removeMax()
        {
            swap(1, size);
            maxElementToHeap[maxHeapToElement[size]] = 0;
            maxHeapToElement[size] = 0;
            size--;
            siftDown(1);
        }
        void delEle(int eleP)
        {
            int heapP = maxElementToHeap[eleP];
            swap(heapP, size);
            maxElementToHeap[eleP] = 0;
            maxHeapToElement[size] = 0;
            size--;
            siftDown(heapP);
            siftUp(heapP);
        }
        void siftUp(int np)
        {
            int parent = np / 2;
            while (parent >= 1 && maxContents[np] > maxContents[parent])
            {
                swap(np, parent);

                np = parent;
                parent = np / 2;
            }
        }
        void siftDown(int np)
        {
            int left = 2 * np;
            int right = 2 * np + 1;
            while (1)
            {
                if (((left <= size) && (maxContents[left] > maxContents[np])) || ((right <= size) && (maxContents[right] > maxContents[np])))
                {
                    if (right > size || (maxContents[left] > maxContents[right]))
                    {
                        swap(left, np);
                        np = left;
                    }
                    else
                    {
                        swap(right, np);
                        np = right;
                    }
                    left = 2 * np;
                    right = 2 * np + 1;
                }
                else
                    break;
            }
        }
        void swap(int np, int parent)
        {
            //swap content
            int temp;
            temp = maxContents[parent];
            maxContents[parent] = maxContents[np];
            maxContents[np] = temp;

            //swap order
            int tp;
            tp = maxHeapToElement[parent];
            maxHeapToElement[parent] = maxHeapToElement[np];
            maxHeapToElement[np] = tp;

            //set another order
            maxElementToHeap[maxHeapToElement[parent]] = parent;
            maxElementToHeap[maxHeapToElement[np]] = np;
        }
    } * wL2u;

    void printGraph();
    void printGraphKCut();
};

class Timer
{
  public:
    chrono::system_clock::time_point startTime;
    chrono::duration<double> stepTime;
    Timer()
    {
        startTime = chrono::system_clock::now();
        stepTime = chrono::duration<double>(0);
    }

    double CheckTimer()
    {
        stepTime = chrono::system_clock::now() - startTime;
        return stepTime.count();
    }
};
