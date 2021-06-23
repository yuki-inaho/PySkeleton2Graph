#ifndef PYSKELETON2GRAPH_INCLUDE_GRAPH_H_
#define PYSKELETON2GRAPH_INCLUDE_GRAPH_H_

#include <memory>
#include <unordered_map>
#include "typedef.h"

template <typename ND, typename LD> struct Node;
template <typename ND, typename LD> struct Edge;
template <typename ND, typename LD> struct Graph;

template <typename ND, typename LD>
struct Node
{
    ND data;
    Graph<ND, LD> &graph;
    Node<ND, LD> *prev, *next;
    Edge<ND, LD> *firstIn, *lastIn, *firstOut, *lastOut;

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
    }

    Node(Graph<ND, LD> &graph)
        : Node<ND, LD>(ND(), graph)
    {
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
    }

    /*
    This function is Outgoing-ward neighbor node getter function,
    But if graph is undirected, this function returns all neighbor list.
    */
    std::vector<Node<ND, LD> *> getNeighborNodes(){
        std::vector<Node<ND, LD> *> neighbor_node_list;
        for (Edge<ND, LD> *x=firstOut; x; x=x->nextInFrom) {
            neighbor_node_list.push_back(x->to);
        }
        std::unique(neighbor_node_list.begin(), neighbor_node_list.end());
        return neighbor_node_list;
    }

    template <typename CBack>
    bool forEachOutgoingEdge(CBack cb)
    {
        for (Edge<ND, LD> *x = firstOut; x; x = x->nextInFrom)
        {
            if (!cb(x))
                return false;
        }
        return true;
    }

    template <typename CBack>
    bool forEachIncomingEdge(CBack cb)
    {
        for (Edge<ND, LD> *x = firstIn; x; x = x->nextInTo)
        {
            if (!cb(x))
                return false;
        }
        return true;
    }
};

template <typename ND, typename LD>
struct Edge
{
    LD data;
    Node<ND, LD> *from, *to;
    Edge<ND, LD> *prev, *next, *prevInFrom, *nextInFrom, *prevInTo, *nextInTo;
    Edge(LD data, Node<ND, LD> *from, Node<ND, LD> *to)
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
    }

    Edge(Node<ND, LD> *from, Node<ND, LD> *to)
        : Edge<ND, LD>(LD(), from, to)
    {
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
    }

    Edge(const Edge<ND, LD> &) = delete;
    Edge(Edge<ND, LD> &&) = delete;
    Edge &operator=(const Edge<ND, LD> &) = delete;
};

template <typename ND, typename LD>
struct Graph
{
    Node<ND, LD> *firstNode, *lastNode;
    Edge<ND, LD> *firstEdge, *lastEdge;

    Graph()
        : firstNode(nullptr), lastNode(nullptr),
          firstEdge(nullptr), lastEdge(nullptr)
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

    Node<ND, LD> *addNode(ND data) { return new Node<ND, LD>(data, *this); }
    Edge<ND, LD> *addEdge(LD data, Node<ND, LD> *from, Node<ND, LD> *to) { return new Edge<ND, LD>(data, from, to); }
    Node<ND, LD> *addNode() { return new Node<ND, LD>(*this); }
    Edge<ND, LD> *addEdge(Node<ND, LD> *from, Node<ND, LD> *to) { return new Edge<ND, LD>(from, to); }

    template <typename CBack>
    bool forEachNode(CBack cb)
    {
        for (Node<ND, LD> *n = firstNode; n; n = n->next)
        {
            if (!cb(n))
                return false;
        }
        return true;
    }

    template <typename CBack>
    bool forEachEdge(CBack cb)
    {
        for (Edge<ND, LD> *L = firstEdge; L; L = L->next)
        {
            if (!cb(L))
                return false;
        }
        return true;
    }
};

#endif // PYSKELETON2GRAPH_INCLUDE_GRAPH_H_
