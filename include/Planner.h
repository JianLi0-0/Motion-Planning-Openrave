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
        double stop_step_size;
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
    Node* NearestNode(NodeTree* query_tree, State query_point);
    bool IsInWorkspace(State q);
    bool CheckDestination(State q, State goal);
    void ShortcutSmoothing(std::vector<Node*>& path);
    void RandomShortcutSmoothing(std::vector<Node*>& path);
    bool CheckLineCollision(State q1, State q2);
    double RandomNumber();
    double L2Norm(const State &a, const State &b);
    double L2Norm(const State &a);
    bool Connect(NodeTree* rrt_tree, KDTree* &kd_tree, Node* nearest_node, State rand_q);
    bool Extend(NodeTree* rrt_tree, KDTree* &kd_tree, Node* nearest_node, State rand_q);


    virtual bool CheckCollision(State q) = 0;
    
    Params _params;

};

#endif