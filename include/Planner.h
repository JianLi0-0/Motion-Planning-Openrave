#include <math.h>
#include "NodeTree.h"
#include "KDTree.h"

class Planner
{
private:
    /* data */
public:
    Planner();
    ~Planner();
    State SampleRandomConfig();
    void Planning(State start, State goal);
    Node* NearestNode(NodeTree* query_tree, std::vector<State>* node_list, State query_point);
    bool LocalPlanner(NodeTree* rrt_tree, std::vector<State>* node_list, Node* nearest_node, State rand_q);
    bool CheckCollision(State q);
    bool IsInWorkspace(State q);
    bool CheckDestination(State q);
    void ShortcutSmoothing(std::vector<Node*>& path);
    bool CheckLineCollision(State q1, State q2);
    double RandomNumber();
    double L2Norm(const State &a, const State &b);
    double L2Norm(const State &a);
    
    struct{
        float goal_bias_probability = 0.20;
        float goal_bias_probability_second = 0.30;
        float goal_bias_mag = 60.0/180.0*M_PI;
        State upper_limit;
        State lower_limit;
        std::vector<int32_t> joint_index;
        double step_size = 15.0/180.0*M_PI;
        int max_sample_points = 80000;
        int draw_tree_point = 0;
        int draw_tree_line = 0;
        int draw_path_point = 5;
        int draw_path_line = 3;
        State weights = {6,6,6,4,4,3,2};
        double total_wights;
    }_params;
    State _start;
    State _goal;
    int _smooth_max_iter = 500;
    int _smooth_step_size = 3.0/180.0*M_PI;
};
