#include "BiRRT.h"
using namespace std;

BiRRT::BiRRT(EnvironmentBasePtr penv, std::istream& ss)
:ModuleBase(penv) 
{
    RegisterCommand("PlanningInterface",boost::bind(&BiRRT::PlanningInterface,this,_1,_2),
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

bool BiRRT::PlanningInterface(std::ostream& sout, std::istream& sinput)
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

void BiRRT::Planning(State start, State goal)
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
    NodeTree* rrt_tree_1 = new NodeTree(new Node(_start, nullptr));
    NodeTree* rrt_tree_2 = new NodeTree(new Node(_goal, nullptr));
    std::vector<State>* node_list_1 = new std::vector<State>();
    std::vector<State>* node_list_2 = new std::vector<State>();
    KDTree* kd_tree_1;
    KDTree* kd_tree_2;
    size_t count1 = 0, count2 = 0;
    // bool first_large;
    do{
        auto rand_q = SampleRandomConfig();
        auto tree_1_nearest_node = NearestNode(rrt_tree_1, node_list_1, kd_tree_1, rand_q);
        Connect(rrt_tree_1, node_list_1, kd_tree_1, tree_1_nearest_node, rand_q);

        State tree_goal = rrt_tree_1->GetLatestNode()->q;
        auto tree_2_nearest_node = NearestNode(rrt_tree_2, node_list_2, kd_tree_2, tree_goal);
        Extend(rrt_tree_2, node_list_2, kd_tree_2, tree_2_nearest_node, rand_q);

        if (L2Norm(tree_goal, rrt_tree_2->GetLatestNode()->q) < _params.step_size) {
            termination = true;
            std::cout << "termination = true" << std::endl;
        }

        if (rrt_tree_1->GetTreeSize() > rrt_tree_2->GetTreeSize()) {
            // std::cout << "SWAP" << std::endl;
            std::swap(rrt_tree_1, rrt_tree_2);
            std::swap(node_list_1, node_list_2);
            std::swap(kd_tree_1, kd_tree_2);
        }

        auto tree_size = int(rrt_tree_1->GetTreeSize())+int(rrt_tree_2->GetTreeSize());
        if (tree_size > _params.max_sample_points) {std::cout << "Maximum points are sampled !!!" << std::endl;break;}
        if(rrt_tree_1->GetTreeSize()>count1) {count1 += 500; std::cout << "tree_1_size: " <<  rrt_tree_1->GetTreeSize() << "  tree_2_size: " <<  rrt_tree_2->GetTreeSize() << std::endl; }
        if(rrt_tree_2->GetTreeSize()>count2) {count2 += 500; std::cout << "tree_1_size: " <<  rrt_tree_1->GetTreeSize() << "  tree_2_size: " <<  rrt_tree_2->GetTreeSize() << std::endl; }
    }while (!termination);

    Node* end_point_1 = rrt_tree_1->GetLatestNode();
    Node* end_point_2 = rrt_tree_2->GetLatestNode();

    // visualize the original path
    auto temp_end_point = end_point_1;
    do{
        VisualizeTree(openrave_ptr, temp_end_point, _params.draw_path_point, _params.draw_path_line, _green, _red);
        temp_end_point = temp_end_point->parent;
    }while(temp_end_point->parent!=nullptr);
    temp_end_point = end_point_2;
    do{
        VisualizeTree(openrave_ptr, temp_end_point, _params.draw_path_point, _params.draw_path_line, _green, _red);
        temp_end_point = temp_end_point->parent;
    }while(temp_end_point->parent!=nullptr);

    // concatenate two path
    std::cout << "path1 " << std::endl;
    auto path1 = rrt_tree_1->GetPath(end_point_1);
    auto path2 = rrt_tree_2->GetPath(end_point_2);
    std::vector<Node *> path;
    if( L2Norm(rrt_tree_1->GetNodeByIdx(0)->q, _goal)<_params.step_size ){
        std::reverse(path1.begin(), path1.end());
        path2.insert(path2.end(),path1.begin(),path1.end());
        path = path2;
    }
    else{
        std::reverse(path2.begin(), path2.end());
        path1.insert(path1.end(),path2.begin(),path2.end());
        path = path1;
    }
        
    std::cout << "path: " <<  path.size() << std::endl;

    ShortcutSmoothing(path);
    std::cout << "path: " <<  path.size() << std::endl;
    // visualize the short-cutting path
    for(size_t i=0;i<path.size()-1;i++){
        Node* child = new Node(path[i]->q,path[i+1]);
        VisualizeTree(openrave_ptr, child, _params.draw_path_point, _params.draw_path_line, _blue, _green);
    }

    delete rrt_tree_1;
    delete rrt_tree_2;
    rrt_tree_1 = nullptr;
    rrt_tree_2 = nullptr;
    delete node_list_1;
    delete node_list_2;
    node_list_1 = nullptr;
    node_list_2 = nullptr;


}

bool BiRRT::LocalPlanner(NodeTree* rrt_tree, std::vector<State>* node_list, KDTree* &kd_tree, Node* nearest_node, State rand_q)
{
    auto current_q = nearest_node->q;
    auto delta_q = rand_q + current_q*(-1);
    auto unit_step = delta_q*(1/L2Norm(delta_q));

    auto new_q = current_q + unit_step*_params.step_size;

    while (IsInWorkspace(new_q) && !CheckCollision(new_q) && L2Norm(NearestNode(rrt_tree, node_list, kd_tree, new_q)->q, new_q)>0.9*_params.step_size) // 
    {
        Node* new_node = new Node(new_q, nearest_node);
        rrt_tree->Append(new_node);
        VisualizeTree(openrave_ptr, new_node, _params.draw_tree_point, _params.draw_tree_line, _green, _red);

        // if (CheckDestination(new_q, _goal)) return true;
    
        new_q = new_q + unit_step*_params.step_size;
        nearest_node = new_node;
    }

    return false;
}

State BiRRT::SampleRandomConfig()
{
    State q = _goal;
    do{
        q.clear();
        for(std::size_t i=0;i<_goal.size();i++){
            q.push_back(RandomNumber()*(_params.upper_limit[i]-_params.lower_limit[i])+_params.lower_limit[i]);
        }
    }while (!IsInWorkspace(q) || CheckCollision(q));
    return q;
}

bool BiRRT::CheckCollision(State q)
{
    openrave_ptr->_robot_ptr->SetDOFValues(q, 1, openrave_ptr->_leftarm_ptr->GetArmIndices());
    // return (GetEnv()->CheckStandaloneSelfCollision(openrave_ptr->_robot_ptr) || GetEnv()->CheckCollision(openrave_ptr->_robot_ptr));
    return GetEnv()->CheckCollision(openrave_ptr->_robot_ptr);
}

void BiRRT::TestTime()
{
    CustomTimer timer;
    for(int j=0;j<1;j++){
        timer.tic();
        for(int i=0;i<32000;i++)
        {
            openrave_ptr->_robot_ptr->SetDOFValues(_start, 1, openrave_ptr->_leftarm_ptr->GetArmIndices());
            GetEnv()->CheckCollision(openrave_ptr->_robot_ptr);
        }
        timer.toc(true);
        timer.tic();
        for(int i=0;i<32000;i++)
        {
            openrave_ptr->_robot_ptr->SetDOFValues(_start, 1, openrave_ptr->_leftarm_ptr->GetArmIndices());
            GetEnv()->CheckStandaloneSelfCollision(openrave_ptr->_robot_ptr);
        }
        timer.toc(true);
    }
    
    timer.tic();
    timer.toc(true);
}

// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "birrt" ) {
        return InterfaceBasePtr(new BiRRT(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Module].push_back("BiRRT");
    
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}

