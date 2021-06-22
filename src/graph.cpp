#include "graph.h"

template <typename ND>
struct Node
{
    ND data;
    Graph &graph;
    Node *prev, *next;
    Link *firstIn, *lastIn, *firstOut, *lastOut;
    Node(ND data, Graph &graph)
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

    Node(Graph &graph)
        : Node(ND(), graph)
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
        for (Link *x = firstOut; x; x = x->nextInFrom)
        {
            if (!cb(x))
                return false;
        }
        return true;
    }

    template <typename CBack>
    bool forEachIncomingLink(CBack cb)
    {
        for (Link *x = firstIn; x; x = x->nextInTo)
        {
            if (!cb(x))
                return false;
        }
        return true;
    }
};

template <typename LD>
struct Link
{
    LD data;
    Node *from, *to;
    Link *prev, *next, *prevInFrom, *nextInFrom, *prevInTo, *nextInTo;
    Link(LD data, Node *from, Node *to)
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

    Link(Node *from, Node *to)
        : Link(LD(), from, to)
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

    Link(const Link &) = delete;
    Link(Link &&) = delete;
    Link &operator=(const Link &) = delete;
};
