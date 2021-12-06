#ifndef PLANNER_
#define PLANNER_

#include <math.h>
#include <boost/bind.hpp>
#include <stdlib.h>
#include <time.h> 
#include <iostream>
#include <numeric>
#include "NodeTree.h"
#include "KDTree.h"

struct Params{
        float goal_bias_probability;
        float goal_bias_probability_second;
        float goal_bias_mag;
        State upper_limit;
        State lower_limit;
        std::vector<int32_t> joint_index;
        double step_size;
        int max_sample_points;
        int draw_tree_point;
        int draw_tree_line;
        int draw_path_point;
        int draw_path_line;
        State weights;
        double total_wights;
        int smooth_max_iter;
};


class Planner
{
private:
    /* data */
public:
    Planner() {}
    ~Planner(){}
    virtual State SampleRandomConfig(State goal);
    Node* NearestNode(NodeTree* query_tree, std::vector<State>* node_list, KDTree* &kd_tree, State query_point);
    bool IsInWorkspace(State q);
    bool CheckDestination(State q, State goal);
    void ShortcutSmoothing(std::vector<Node*>& path);
    bool CheckLineCollision(State q1, State q2);
    double RandomNumber();
    double L2Norm(const State &a, const State &b);
    double L2Norm(const State &a);

    virtual bool CheckCollision(State q) = 0;
    
    Params _params;

};

State Planner::SampleRandomConfig(State goal)
{
    State q = goal;
    if(RandomNumber()<_params.goal_bias_probability){
        if(RandomNumber()<_params.goal_bias_probability_second) q = goal;
        else{
            do{
                q.clear();
                for(std::size_t i=0;i<goal.size();i++){
                    q.push_back((RandomNumber()-0.5)*2.0*_params.goal_bias_mag + goal[i]);
                }
            }while (!IsInWorkspace(q) || CheckCollision(q));
        }
    }
    else{
        do{
            q.clear();
            for(std::size_t i=0;i<goal.size();i++){
                q.push_back(RandomNumber()*(_params.upper_limit[i]-_params.lower_limit[i])+_params.lower_limit[i]);
            }
        }while (!IsInWorkspace(q) || CheckCollision(q));
    }
    return q;
}

Node* Planner::NearestNode(NodeTree* query_tree, std::vector<State>* node_list, KDTree* &kd_tree, State query_point)
{
    auto tree_size = query_tree->GetTreeSize();
    if( tree_size<200 || tree_size%200==0 )
    {
        if(tree_size<node_list->size()) node_list->clear();

        while (tree_size > node_list->size())
        {
            auto n = query_tree->GetNodeByIdx(node_list->size());
            node_list->push_back(n->q*_params.weights);
        }
        kd_tree =  new KDTree(*node_list);
    }
    
    auto index = kd_tree->nearest_index(query_point*_params.weights);
    return query_tree->GetNodeByIdx(index);
}

bool Planner::IsInWorkspace(State q)
{
    for(size_t i=0;i<q.size();i++)
    {
        if(q[i]<_params.lower_limit[i] || q[i]>_params.upper_limit[i])
            return false;
    }
    return true;
}

bool Planner::CheckDestination(State q, State goal)
{
    if (L2Norm(goal, q) <= _params.step_size*2) return true;
    else return false;
}

void Planner::ShortcutSmoothing(std::vector<Node*>& path)
{
    for(int i=0;i<_params.smooth_max_iter;i++){
        int path_size = path.size();
        int rand1 = int(RandomNumber()*path_size);
        int rand2 = int(RandomNumber()*path_size);
        int p1_idx = std::min(rand1, rand2);
        int p2_idx = std::max(rand1, rand2);
        auto p1 = path[p1_idx];
        auto p2 = path[p2_idx];
        if (p2-p1<2) continue;
        if (!CheckLineCollision(p1->q, p2->q)){
            path.erase(path.begin()+p1_idx+1, path.begin()+p2_idx-1);
        }
    }
}

bool Planner::CheckLineCollision(State q1, State q2)
{
    auto delta_q = q2 + q1*(-1);
    auto unit_step = delta_q*(1/L2Norm(delta_q));
    auto new_q = q1;
    while(L2Norm(q2, new_q) > _params.step_size*0.5)
    {
        new_q = new_q + unit_step*(_params.step_size*0.3);
        if (!IsInWorkspace(new_q)) continue;
        if (CheckCollision(new_q)) return true;
    }

    return false;
}

double Planner::RandomNumber()
{
    return double(rand()) / double(RAND_MAX);
}

double Planner::L2Norm(const State &a, const State &b) 
{
    double distc = 0;
    for (size_t i = 0; i < a.size(); i++) {
        double di = a.at(i) - b.at(i);
        distc += di * di;
    }
    return std::sqrt(distc);
}

double Planner::L2Norm(const State &a) 
{
    double distc = 0;
    for (size_t i = 0; i < a.size(); i++) {
        double di = a.at(i);
        distc += di * di;
    }
    return std::sqrt(distc);
}
#endif