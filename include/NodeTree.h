#ifndef NODETREE_
#define NODETREE_

#include <vector>
#include <algorithm>
#include "Node.h"

std::ostream& operator<<(std::ostream& out, Node & T);
std::ostream& operator<<(std::ostream& out, State & q);
State operator+(const State& v1, const State& v2);
State operator*(const State& v1, const double& k);
State operator*(State const& v1, State const& v2);
template <typename T> void MyPrint (T const& q);

class NodeTree
{
    private:
        std::vector<Node*> _nodes;
        State _weights;      
    public:
        NodeTree(){};
        NodeTree(Node* root, State weights): _nodes({root}), _weights(weights) {};
        ~NodeTree(){};
        void Append(Node* n) {_nodes.push_back(n);}
        void Delete();
        Node* GetLatestNode() { return _nodes[GetTreeSize()-1]; }
        Node* GetNodeByIdx(size_t idx){ return _nodes[idx]; }
        size_t GetTreeSize() { return _nodes.size();}
        size_t size() { return _nodes.size();}
        State at(size_t i) { return _nodes[i]->q*_weights; }
        std::vector<Node*> GetPath(Node* end_point)
        {
            std::vector<Node*> path;
            do{
                path.push_back(end_point);
                end_point = end_point->parent;
            }while(end_point!=nullptr);
            std::reverse(path.begin(), path.end());
            return path;
        }
};

#endif