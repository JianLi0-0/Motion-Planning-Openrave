#ifndef BIRRT_
#define BIRRT_

#include <openrave/plugin.h>
#include "util.h"
#include "Planner.h"

class BiRRT : public ModuleBase, public Planner
{
public:
    BiRRT(EnvironmentBasePtr penv, std::istream& ss);
    virtual ~BiRRT() {}
    bool PlanningInterface(std::ostream& sout, std::istream& sinput);
    
    void Planning(State start, State goal);
    bool LocalPlanner(NodeTree* rrt_tree, std::vector<State>* node_list, KDTree* &kd_tree, Node* nearest_node, State rand_q);
    bool CheckCollision(State q);
    State SampleRandomConfig();

    void TestTime();

private:
    Params _params ={
        .goal_bias_probability = 0.20,
        .goal_bias_probability_second = 0.30,
        .goal_bias_mag = 60.0/180.0*M_PI,
        .step_size = 15.0/180.0*M_PI,
        .max_sample_points = 40000,
        .draw_tree_point = 0,
        .draw_tree_line = 0,
        .draw_path_point = 5,
        .draw_path_line = 3,
        .weights = {6,6,6,4,4,3,2},
        .smooth_max_iter = 200,
        };
    State _start;
    State _goal;

    OpenravePtr* openrave_ptr;
};


#endif