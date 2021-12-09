#include "RRTConnect.h"
using namespace std;

RRTconnect::RRTconnect(EnvironmentBasePtr penv, std::istream& ss)
:ModuleBase(penv) 
{
    RegisterCommand("PlanningInterface",boost::bind(&RRTconnect::PlanningInterface,this,_1,_2),
                    "This is an example command");
    std::srand((unsigned)time(NULL));
    openrave_ptr = new OpenravePtr();
    openrave_ptr->_env = GetEnv();
    openrave_ptr->_robot_ptr = GetEnv()->GetRobot("PR2");
    openrave_ptr->_leftarm_ptr = openrave_ptr->_robot_ptr->SetActiveManipulator("leftarm");
    openrave_ptr->_robot_ptr->GetActiveDOFLimits(_params.lower_limit, _params.upper_limit);
    _params.joint_index = openrave_ptr->_leftarm_ptr->GetArmIndices();
    for(size_t i=0;i<_params.joint_index.size();i++){
        _params.lower_limit[i] = _params.lower_limit[_params.joint_index[i]];
        _params.upper_limit[i] = _params.upper_limit[_params.joint_index[i]];
        if(_params.lower_limit[i]<-5) _params.lower_limit[i]=-4*M_PI;
        if(_params.upper_limit[i]> 5) _params.upper_limit[i]= 4*M_PI;
    }
    _params.total_wights = std::accumulate(_params.weights.begin(), _params.weights.end(), decltype(_params.weights)::value_type(0));
    _params.weights = _params.weights * (1.0/_params.total_wights);

    Planner::_params = _params;

}

bool RRTconnect::PlanningInterface(std::ostream& sout, std::istream& sinput)
{
    // std::string input;
    // sinput >> input;
    // sout << input;
    std::istreambuf_iterator<char> eos;
    std::string goal_str(std::istreambuf_iterator<char>(sinput), eos);
    State command;
    string_to_vector_double(goal_str, command);

    _start.assign(command.begin(), command.begin()+7);
    _goal.assign(command.begin()+7, command.end());

    Planning(_start, _goal);

    return true;
}

void RRTconnect::Planning(State start, State goal)
{
    _start = start;
    _goal = goal;

    DrawEndEffector(openrave_ptr, _start, 1, 10);
    DrawEndEffector(openrave_ptr,_goal, 1, 10);
    if(CheckCollision(_start) || CheckCollision(_goal)) {
        cout << "start or goal in collision" << endl; 
        return;
    }

    bool termination = false;
    NodeTree* rrt_tree = new NodeTree(new Node(_start, nullptr), _params.weights);
    rrt_tree->set_kd_tree( new KDTree(rrt_tree) );
    // KDTree* kd_tree;
    size_t count = 0;
    do{
        auto rand_q = SampleRandomConfig(_goal);
        auto nearest_node = NearestNode(rrt_tree, rand_q);
        termination = LocalPlanner(rrt_tree, nearest_node, rand_q);
        if (int(rrt_tree->GetTreeSize()) > _params.max_sample_points) {std::cout << "Maximum points are sampled !!!" << std::endl;break;}
        if(rrt_tree->GetTreeSize()>count) {
            count += 400;
            std::cout << "tree_size: " <<  rrt_tree->GetTreeSize() << std::endl;
        }
    }while (!termination);

    Node* end_point;
    if(termination) {
        rrt_tree->Append(new Node(_goal, rrt_tree->GetLatestNode()));
        end_point = rrt_tree->GetLatestNode();
    }
    else end_point = NearestNode(rrt_tree, _goal);
    std::cout << *end_point << std::endl;
    
    auto temp_end_point = end_point;
    do{
        VisualizeTree(openrave_ptr, temp_end_point, _params.draw_path_point, _params.draw_path_line, _green, _red);
        temp_end_point = temp_end_point->parent;
    }while(temp_end_point->parent!=nullptr);

    temp_end_point = end_point;
    auto path = rrt_tree->GetPath(temp_end_point);
    std::cout << "path: " <<  path.size() << std::endl;

    ShortcutSmoothing(path);
    std::cout << "path: " <<  path.size() << std::endl;
    for(size_t i=0;i<path.size()-1;i++){
        Node* child = new Node(path[i]->q,path[i+1]);
        VisualizeTree(openrave_ptr, child, _params.draw_path_point, _params.draw_path_line, _blue, _green);
    }

    delete rrt_tree;
    rrt_tree = nullptr;
    // delete kd_tree;
    // kd_tree = nullptr;

}

bool RRTconnect::LocalPlanner(NodeTree* rrt_tree, Node* nearest_node, State rand_q)
{
    auto current_q = nearest_node->q;
    auto delta_q = rand_q + current_q*(-1);
    auto unit_step = delta_q*(1/L2Norm(delta_q));

    auto new_q = current_q + unit_step*_params.step_size;

    while (IsInWorkspace(new_q) && !CheckCollision(new_q) && L2Norm(NearestNode(rrt_tree, new_q)->q, new_q)>0.9*_params.step_size) // 
    {
        Node* new_node = new Node(new_q, nearest_node);
        rrt_tree->Append(new_node);
        VisualizeTree(openrave_ptr, new_node, _params.draw_tree_point, _params.draw_tree_line, _green, _red);

        if (CheckDestination(new_q, _goal)) return true;
        // return false; // extend
    
        new_q = new_q + unit_step*_params.step_size;
        nearest_node = new_node;
    }

    return false;
}

bool RRTconnect::CheckCollision(State q)
{
    openrave_ptr->_robot_ptr->SetDOFValues(q, 1, openrave_ptr->_leftarm_ptr->GetArmIndices());
    // return (GetEnv()->CheckStandaloneSelfCollision(openrave_ptr->_robot_ptr) || GetEnv()->CheckCollision(openrave_ptr->_robot_ptr));
    return GetEnv()->CheckCollision(openrave_ptr->_robot_ptr);
}

// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "rrtconnect" ) {
        return InterfaceBasePtr(new RRTconnect(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Module].push_back("RRTconnect");
    
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}

