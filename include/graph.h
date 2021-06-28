/*
This code is based on the implements of below repository.
https://github.com/6502/cppgraph

MIT License

Copyright (c) 2021 Andrea Griffini, Yuki Yoshikawa

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef PYSKELETON2GRAPH_INCLUDE_GRAPH_H_
#define PYSKELETON2GRAPH_INCLUDE_GRAPH_H_

#include <memory>
#include <unordered_map>
#include "typedef_pixel.h"

// TODO: use smart ptr
template <typename ND, typename LD> struct Node;
template <typename ND, typename LD> struct Edge;
template <typename ND, typename LD> struct Graph;

template <typename ND, typename LD> using NodePtr = Node<ND, LD> *;
template <typename ND, typename LD> using EdgePtr = Edge<ND, LD> *;

template <typename ND, typename LD>
struct Node
{
    ND data;
    Graph<ND, LD> &graph;
    NodePtr<ND, LD> prev, next;
    EdgePtr<ND, LD> firstIn, lastIn, firstOut, lastOut;

    Node(ND data, Graph<ND, LD> &graph)
        : data(std::move(data)),
          graph(graph),
          prev(graph.lastNode), next(nullptr),
          firstIn(nullptr), lastIn(nullptr),
          firstOut(nullptr), lastOut(nullptr)
    {
        if (prev)
            prev->next = this;
        else
            graph.firstNode = this;
        graph.lastNode = this;
        graph.m_num_nodes_++;
    }

    Node(Graph<ND, LD> &graph)
        : Node<ND, LD>(ND(), graph)
    {
        graph.m_num_nodes_++;
    }

    ~Node()
    {
        while (lastIn)
            delete lastIn;
        while (lastOut)
            delete lastOut;
        if (prev)
            prev->next = next;
        else
            graph.firstNode = next;
        if (next)
            next->prev = prev;
        else
            graph.lastNode = prev;
        graph.m_num_nodes_--;
    }

    /*
    This function is Outgoing-ward neighbor node getter function,
    But if graph is undirected, this function returns all neighbor list.
    */
    std::vector<NodePtr<ND, LD>> getNeighborNodes()
    {
        std::vector<NodePtr<ND, LD>> neighbor_node_list;
        for (EdgePtr<ND, LD> x = firstOut; x; x = x->nextInFrom)
        {
            neighbor_node_list.push_back(x->to);
        }
        std::unique(neighbor_node_list.begin(), neighbor_node_list.end());
        return neighbor_node_list;
    }
};

template <typename ND, typename LD>
struct Edge
{
    LD data;
    NodePtr<ND, LD> from, to;
    EdgePtr<ND, LD> prev, next, prevInFrom, nextInFrom, prevInTo, nextInTo;

    /*
    Assume from->graph == to->graph
    */
    Edge(LD data, NodePtr<ND, LD> from, NodePtr<ND, LD> to)
        : data(std::move(data)),
          from(from), to(to),
          prev(from->graph.lastEdge), next(nullptr),
          prevInFrom(from->lastOut), nextInFrom(nullptr),
          prevInTo(to->lastIn), nextInTo(nullptr)
    {
        if (&from->graph != &to->graph)
        {
            throw std::runtime_error("Cannot Edge nodes from different graphs");
        }
        if (prev)
            prev->next = this;
        else
            from->graph.firstEdge = this;
        from->graph.lastEdge = this;
        if (prevInFrom)
            prevInFrom->nextInFrom = this;
        else
            from->firstOut = this;
        from->lastOut = this;
        if (prevInTo)
            prevInTo->nextInTo = this;
        else
            to->firstIn = this;
        to->lastIn = this;
        from->graph.m_num_edges_++;
    }

    Edge(NodePtr<ND, LD> from, NodePtr<ND, LD> to)
        : Edge<ND, LD>(LD(), from, to)
    {
        from->graph.m_num_edges_++;
    }

    ~Edge()
    {
        if (prevInTo)
            prevInTo->nextInTo = nextInTo;
        else
            to->firstIn = nextInTo;
        if (nextInTo)
            nextInTo->prevInTo = prevInTo;
        else
            to->lastIn = prevInTo;
        if (prevInFrom)
            prevInFrom->nextInFrom = nextInFrom;
        else
            from->firstOut = nextInFrom;
        if (nextInFrom)
            nextInFrom->prevInFrom = prevInFrom;
        else
            from->lastOut = prevInFrom;
        if (prev)
            prev->next = next;
        else
            from->graph.firstEdge = next;
        if (next)
            next->prev = prev;
        else
            from->graph.lastEdge = prev;

        from->graph.m_num_edges_--;
    }

    Edge(const Edge<ND, LD> &) = delete;
    Edge(Edge<ND, LD> &&) = delete;
    Edge &operator=(const Edge<ND, LD> &) = delete;
};

template <typename ND, typename LD>
struct Graph
{
    int32_t m_num_nodes_;
    int32_t m_num_edges_;

    NodePtr<ND, LD> firstNode, lastNode;
    EdgePtr<ND, LD> firstEdge, lastEdge;

    Graph()
        : firstNode(nullptr), lastNode(nullptr),
          firstEdge(nullptr), lastEdge(nullptr), m_num_nodes_(0), m_num_edges_(0)
    {
    }

    ~Graph()
    {
        while (lastNode)
            delete lastNode;
    }

    Graph(const Graph<ND, LD> &) = delete;
    Graph(Graph<ND, LD> &&) = delete;
    Graph<ND, LD> &operator=(const Graph<ND, LD> &) = delete;

    NodePtr<ND, LD> addNode(ND data) { return new Node<ND, LD>(data, *this); }
    EdgePtr<ND, LD> addEdge(LD data, NodePtr<ND, LD> from, NodePtr<ND, LD> to) { return new Edge<ND, LD>(data, from, to); }
    NodePtr<ND, LD> addNode() { return new Node<ND, LD>(*this); }
    EdgePtr<ND, LD> addEdge(NodePtr<ND, LD> from, NodePtr<ND, LD> to) { return new Edge<ND, LD>(from, to); }

    int32_t get_number_of_nodes() const
    {
        return m_num_nodes_;
    }

    int32_t get_number_of_edges() const
    {
        return m_num_edges_;
    }
};

#endif // PYSKELETON2GRAPH_INCLUDE_GRAPH_H_
