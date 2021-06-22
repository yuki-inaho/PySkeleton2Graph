#ifndef PYSKELETON2GRAPH_INCLUDE_GRAPH_H_
#define PYSKELETON2GRAPH_INCLUDE_GRAPH_H_

#include <memory>
#include "typedef.h"

template <typename ND, typename LD> struct Node;
template <typename ND, typename LD> struct Link;
template <typename ND, typename LD> struct Graph;

template <typename ND, typename LD>
struct Node
{
    ND data;
    Graph<ND, LD> &graph;
    Node<ND, LD> *prev, *next;
    Link<ND, LD> *firstIn, *lastIn, *firstOut, *lastOut;

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

    template <typename CBack>
    bool forEachOutgoingLink(CBack cb)
    {
        for (Link<ND, LD> *x = firstOut; x; x = x->nextInFrom)
        {
            if (!cb(x))
                return false;
        }
        return true;
    }

    template <typename CBack>
    bool forEachIncomingLink(CBack cb)
    {
        for (Link<ND, LD> *x = firstIn; x; x = x->nextInTo)
        {
            if (!cb(x))
                return false;
        }
        return true;
    }
};

template <typename ND, typename LD>
struct Link
{
    LD data;
    Node<ND, LD> *from, *to;
    Link<ND, LD> *prev, *next, *prevInFrom, *nextInFrom, *prevInTo, *nextInTo;
    Link(LD data, Node<ND, LD> *from, Node<ND, LD> *to)
        : data(std::move(data)),
          from(from), to(to),
          prev(from->graph.lastLink), next(nullptr),
          prevInFrom(from->lastOut), nextInFrom(nullptr),
          prevInTo(to->lastIn), nextInTo(nullptr)
    {
        if (&from->graph != &to->graph)
        {
            throw std::runtime_error("Cannot link nodes from different graphs");
        }
        if (prev)
            prev->next = this;
        else
            from->graph.firstLink = this;
        from->graph.lastLink = this;
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

    Link(Node<ND, LD> *from, Node<ND, LD> *to)
        : Link<ND, LD>(LD(), from, to)
    {
    }

    ~Link()
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
            from->graph.firstLink = next;
        if (next)
            next->prev = prev;
        else
            from->graph.lastLink = prev;
    }

    Link(const Link<ND, LD> &) = delete;
    Link(Link<ND, LD> &&) = delete;
    Link &operator=(const Link<ND, LD> &) = delete;
};

template <typename ND, typename LD>
struct Graph
{
    Node<ND, LD> *firstNode, *lastNode;
    Link<ND, LD> *firstLink, *lastLink;

    Graph()
        : firstNode(nullptr), lastNode(nullptr),
          firstLink(nullptr), lastLink(nullptr)
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
    Link<ND, LD> *addLink(LD data, Node<ND, LD> *from, Node<ND, LD> *to) { return new Link<ND, LD>(data, from, to); }
    Node<ND, LD> *addNode() { return new Node<ND, LD>(*this); }
    Link<ND, LD> *addLink(Node<ND, LD> *from, Node<ND, LD> *to) { return new Link<ND, LD>(from, to); }

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
    bool forEachLink(CBack cb)
    {
        for (Link<ND, LD> *L = firstLink; L; L = L->next)
        {
            if (!cb(L))
                return false;
        }
        return true;
    }
};

#endif // PYSKELETON2GRAPH_INCLUDE_GRAPH_H_
