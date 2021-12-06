#include <vector>
#include <algorithm>
#include "Node.h"

class NodeTree
{
    private:
        std::vector<Node*> _nodes;
    public:
        NodeTree(){};
        NodeTree(Node* root): _nodes({root}) {};
        ~NodeTree(){};
        void Append(Node* n) {_nodes.push_back(n);}
        void Delete();
        Node* GetLatestNode() { return _nodes[GetTreeSize()-1]; }
        Node* GetNodeByIdx(size_t idx){ return _nodes[idx]; }
        size_t GetTreeSize() { return _nodes.size();}
        std::vector<Node*> GetPath(Node* end_point);
};


std::vector<Node*> NodeTree::GetPath(Node* end_point)
{
    std::vector<Node*> path;
    do{
        path.push_back(end_point);
        end_point = end_point->parent;
    }while(end_point!=nullptr);
    std::reverse(path.begin(), path.end());
    return path;
}
