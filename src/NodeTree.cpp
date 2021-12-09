#include "NodeTree.h"

std::ostream& operator<<(std::ostream& out, Node & T)
{
    auto q = T.q;
    out << '(';
    for(std::size_t i=0;i<q.size();i++){
        out << q[i] << ',';
    }
    return out << ')' << std::endl;
}

std::ostream& operator<<(std::ostream& out, State & q)
{
    out << '(';
    for(std::size_t i=0;i<q.size();i++){
        out << q[i] << ',';
    }
    return out << ')' << std::endl;
}

template <typename T>
void MyPrint (T const& q) 
{ 
    std::cout << '(';
    for(std::size_t i=0;i<q.size();i++){
        std::cout << q[i] << ',';
    }
    std::cout << ')' << std::endl;
} 

State operator+(const State& v1, const State& v2) 
{
    State r;
    if(v1.size() == v2.size()){
        r.reserve(v1.size());
        for (size_t i = 0; i < v1.size(); ++i) {
            r.push_back(v1[i] + v2[i]);
        }
    }
    return r;
}

State operator*(const State& v1, const double& k) 
{
    State r;
    r.reserve(v1.size());
    for (size_t i = 0; i < v1.size(); ++i) {
        r.push_back(v1[i] * k);
    }
    return r;
}

State operator*(State const& v1, State const& v2)
{
    State r;
    if(v1.size() == v2.size()){
        r.reserve(v1.size());
        for (size_t i = 0; i < v1.size(); ++i) {
            r.push_back(v1[i] * v2[i]);
        }
    }
    return r;
}