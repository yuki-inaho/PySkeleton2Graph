#ifndef PYSKELETON2GRAPH_INCLUDE_BICONNECTED_COMPONENT_H_
#define PYSKELETON2GRAPH_INCLUDE_BICONNECTED_COMPONENT_H_

#include <set>
#include <stack>
#include <vector>
#include <algorithm>
#include "graph.h"


template <typename ND, typename LD>
struct EdgeCompare
{
    bool operator()(const Edge<ND, LD> &e, const Edge<ND, LD> &f) const
    {
        if (std::min(e.src, e.dst) != std::min(f.src, f.dst))
            return std::min(e.src, e.dst) < std::min(f.src, f.dst);
        return std::max(e.src, e.dst) < std::max(f.src, f.dst);
    }
};

template <typename ND, typename LD>
using Edgeset = std::set<Edge<ND, LD>, EdgeCompare<ND, LD>>;

template <typename ND, typename LD>
void dfs(const Graph<ND, LD> &g, int v, int u,
           std::vector<int> &art, std::vector<Edgeset<ND, LD>> &bcomp,
           std::stack<Edge<ND, LD>> &S, std::vector<int> &ord, std::vector<int> &low, int &time)
{
    low[v] = ord[v] = ++time;
    for (Node<ND, LD> *n = g.firstNode; n; n = n->next)
    {
        int w = e->dst;
        if (ord[w] < ord[v])
            S.push(*e);
        if (ord[w] == 0)
        {
            dfs(g, w, v, art, bcomp, S, ord, low, time);
            low[v] = std::min(low[v], low[w]);
            if ((ord[v] == 1 && ord[w] != 2) || (ord[v] != 1 && low[w] >= ord[v]))
                art.push_back(v);
            if (low[w] >= ord[v])
            { // for bcomps
                bcomp.push_back(Edgeset<ND, LD>());
                while (1)
                {
                    Edge<ND, LD> f = S.top();
                    S.pop();
                    bcomp.back().insert(f);
                    if (f.src == v && f.dst == w)
                        break;
                }
            }
        }
        else{
            low[v] = std::min(low[v], ord[w]);
        }
    }
}

template <typename ND, typename LD>
void articulationPoint(const Graph<ND, LD> &g, std::vector<int> &art, std::vector<Edgeset<ND, LD>> &bcomp)
{
    const int n = g.size();
    std::vector<int> low(n), ord(n);
    std::stack<Edge<ND, LD>> S;
    for(int u=0;u<n;++u){
        if (ord[u] == 0)
        {
            int time = 0;
            dfs(g, u, -1, art, bcomp, S, ord, low, time);
        }

    }
}

#endif