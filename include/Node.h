#ifndef NODE_
#define NODE_

#include <vector>
#include <iostream>

typedef std::vector<double> State;

class Node
{
    public:
        Node(const State c):q(c){};
        Node(const State c, Node* p):q(c),parent(p){};
        ~Node();
        
        State q;
        Node* parent;
};

#endif