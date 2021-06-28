#ifndef PYSKELETON2GRAPH_INCLUDE_BICONNECTED_COMPONENT_H_
#define PYSKELETON2GRAPH_INCLUDE_BICONNECTED_COMPONENT_H_

#include <set>
#include <stack>
#include <vector>
#include <algorithm>
#include <iterator>
#include "graph.h"
#include "graph_helper.h"

template <typename ND, typename LD>
struct EdgeCompare
{
    bool operator()(const Edge<ND, LD> *e, const Edge<ND, LD> *f) const
    {
        if (std::min(e->data.src, e->data.dst) != std::min(f->data.src, f->data.dst))
            return std::min(e->data.src, e->data.dst) < std::min(f->data.src, f->data.dst);
        return std::max(e->data.src, e->data.dst) < std::max(f->data.src, f->data.dst);
    }
};

template <typename ND, typename LD>
using Edgeset = std::set<Edge<ND, LD> *, EdgeCompare<ND, LD>>;

template <typename ND, typename LD>
void dfs(const std::shared_ptr<GraphHelper<ND, LD>> gh, Hash hash_v, Hash hash_parent,
         std::vector<Hash> &art, std::vector<Edgeset<ND, LD>> &bcomp, std::stack<Edge<ND, LD> *> &S,
         std::unordered_map<Hash, int32_t> &ord, std::unordered_map<Hash, int32_t> &low, int &time)
{
    low[hash_v] = ord[hash_v] = ++time;
    std::vector<Hash> neighbor_hash_list_v = gh->getNeighborHashList(hash_v);
    for (Hash hash_n : neighbor_hash_list_v)
    {
        if (ord[hash_n] < ord[hash_v])
            S.push(gh->getEdgePtr(hash_v, hash_n));

        if (ord[hash_n] == 0)
        {
            dfs(gh, hash_n, hash_v, art, bcomp, S, ord, low, time);
            low[hash_v] = std::min(low[hash_v], low[hash_n]);
            if ((ord[hash_v] == 1 && ord[hash_n] != 2) || (ord[hash_v] != 1 && low[hash_n] >= ord[hash_v]))
                art.push_back(hash_v);
            if (low[hash_n] >= ord[hash_v])
            {
                bcomp.push_back(Edgeset<ND, LD>());
                while (1)
                {
                    Edge<ND, LD> *f = S.top();
                    S.pop();
                    bcomp.back().insert(f);
                    if (f->data.src == hash_v && f->data.dst == hash_n)
                        break;
                }
            }
        }
        else if (hash_n != hash_parent)
        {
            low[hash_v] = std::min(low[hash_v], ord[hash_n]);
        }
    }
}

template <typename ND, typename LD>
void articulationPoint(const std::shared_ptr<GraphHelper<ND, LD>> gh, std::vector<Hash> &art, std::vector<std::vector<Hash>> &hash_list_clique_to_compress)
{
    std::vector<Edgeset<ND, LD>> bcomp;
    std::vector<Hash> hash_list_gh = gh->getHashList();
    std::unordered_map<Hash, int32_t> low, ord;

    low.insert(std::pair<Hash, int32_t>{-1, 0});
    ord.insert(std::pair<Hash, int32_t>{-1, 0});
    for (Hash hash : hash_list_gh)
    {
        low.insert(std::pair<Hash, int32_t>{hash, 0});
        ord.insert(std::pair<Hash, int32_t>{hash, 0});
    }

    std::stack<Edge<ND, LD> *> S;
    for (Hash hash_start : hash_list_gh)
    {
        if (ord[hash_start] == 0)
        {
            int time = 0;
            dfs(gh, hash_start, -1, art, bcomp, S, ord, low, time);
        }
    }

    std::vector<Hash> hash_list_union;
    for (Edgeset<ND, LD> edge_list_clique : bcomp)
    {
        if (edge_list_clique.size() > 1)
        {
            std::vector<Hash> hash_list_clique;
            for (Edge<ND, LD> *edge : edge_list_clique)
            {
                hash_list_clique.push_back(edge->data.src);
                hash_list_clique.push_back(edge->data.dst);
            }

            // Remove duplicated node hash
            std::sort(hash_list_clique.begin(), hash_list_clique.end());
            hash_list_clique.erase(std::unique(hash_list_clique.begin(), hash_list_clique.end()), hash_list_clique.end());

            std::vector<Hash> set_diff_result;
            std::set_difference(
                hash_list_clique.begin(), hash_list_clique.end(),
                hash_list_union.begin(), hash_list_union.end(),
                std::back_inserter(set_diff_result));

            std::copy(set_diff_result.begin(), set_diff_result.end(), std::back_inserter(hash_list_union));
            hash_list_clique_to_compress.push_back(set_diff_result);
        }
    }
}
#endif