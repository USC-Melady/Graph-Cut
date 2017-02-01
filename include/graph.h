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

    void getKCC();
    void getKCore();

    void sortKCC();

    Graph(int _nNode, int _K);
    Graph(string filename, int _K);
    Graph(vector<vector<int>> _edges, int _K);
    ~Graph() { delete wL2u; }

    int verbose;
    //
    void KCoreOptimization();

    //   private:
    int nNodes;
    int K;
    // Algorithm Analytic
    int depth = 0;

    // Utils for KCore
    void fixDegree();
    void editDegree(const int node, const WTYPE delta);
    void returnConnectedComp();

    // Utils for KCC
    vector<pair<int, int>> getConnectedComp();
    int kCut(int &currNNode, int &prevNode);
    int kMas(int &seed, int &currNNode);
    void recordMerge(const int u, const int v);
    int mergeAll(int &ncomp);
    int findParent(const int v);
    int mergeNodes();
    void removeCutL();
    void addLtoRet();
    void getNodesInL(vector<int> &nodeInL);
    void getNodesInL(unordered_set<int> &nodeInL);

    // variable for KCC
    vector<pair<int, int>> toMerge;

    void addEdge(const int u, const int v, const WTYPE weight = 1);

    // used for k-cut+
    vector<unordered_map<int, WTYPE>> adjMatrix_kcut;
    // used for kCC
    vector<unordered_map<int, WTYPE>> adjMatrix_graph;

    // Pre-allocated Variables
    void preallocate();
    // set<pair<WTYPE, int>> weight_node;test
    vector<int> node2degree;
    vector<list<int>> parent2child;
    vector<int> parents;
    vector<int> parentsAfterMerge;
    void mergeParentsAfterMerge(const int v, const int u);
    int findParentsAfterMerge(const int v);
    vector<bool> merged;
    unordered_set<int> L;
    vector<bool> bfsMark;
    vector<WTYPE> u2wL;

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
