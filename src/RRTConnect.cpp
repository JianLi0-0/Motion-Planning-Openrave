#include "RRTConnect.h"
using namespace std;

RRTconnect::RRTconnect(EnvironmentBasePtr penv, std::istream& ss)
:ModuleBase(penv) 
{
    RegisterCommand("MyCommand",boost::bind(&RRTconnect::MyCommand,this,_1,_2),
                    "This is an example command");

    std::srand((unsigned)time(NULL));
    _env = GetEnv();
    _robot_ptr = GetEnv()->GetRobot("PR2");
    _leftarm_ptr = _robot_ptr->SetActiveManipulator("leftarm");
    _robot_ptr->GetActiveDOFLimits(_params.lower_limit, _params.upper_limit);
    _params.joint_index = _leftarm_ptr->GetArmIndices();
    for(size_t i=0;i<_params.joint_index.size();i++){
        _params.lower_limit[i] = _params.lower_limit[_params.joint_index[i]];
        _params.upper_limit[i] = _params.upper_limit[_params.joint_index[i]];
        if(_params.lower_limit[i]<-5) _params.lower_limit[i]=-4*M_PI;
        if(_params.upper_limit[i]> 5) _params.upper_limit[i]= 4*M_PI;
    }
}

void string_to_vector_double(string str, std::vector<double>& fea){
    stringstream ss(str);
    string buf;
    vector<double> vec;

    while(ss >> buf)
    {
        vec.push_back(atof(buf.c_str()));
        fea = vec;
    }
    
}

bool RRTconnect::MyCommand(std::ostream& sout, std::istream& sinput)
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

    _params.total_wights = std::accumulate(_params.weights.begin(), _params.weights.end(), decltype(_params.weights)::value_type(0));
    std::cout << "_params.total_wights: " << _params.total_wights<< std::endl;

    _params.weights = _params.weights * (1.0/_params.total_wights);
    std::cout << "_params.weights: " << _params.weights<< std::endl;

    // _params.draw_path_line = 0;
    // _params.draw_path_point=0;
    // _params.draw_tree_line=0;
    // _params.draw_tree_point=0;
    // for(int j=0;j<1;j++){
    //     for(int i=3;i<5;i++){
    //         _params.step_size = (20.0+i*2)/180.0*M_PI; // 18 works1
    //         Planning(_start, _goal);
    //     }
    // }

    Planning(_start, _goal);
    // std::cout << "_success_count:"<< std::endl;
    // MyPrint(_success_count);



    // State q, groundtruth;
    
    // double distance=100000000;

    // NodeTree* query_tree = new NodeTree(new Node({1,2,3,4,5,6,7}));

    // for(int i=0;i<5;i++)
    // {
    //     q.clear();
    //     for(std::size_t i=0;i<_goal.size();i++){
    //         q.push_back(RandomNumber()*(_params.upper_limit-_params.lower_limit)+_params.lower_limit);
    //     }
    //     Node* b = new Node(q);
    //     query_tree->Append(b);
    //     auto temp_d = L2Norm(_goal,q);
    //     if (temp_d<distance) {distance = temp_d;groundtruth=q;}
    // }
    // std::cout << "size: " << query_tree->GetTreeSize()<< std::endl;
    // auto nn = NearestNode(query_tree, _goal);
    // std::cout << "nearest test "; for (double b : nn->q) { std::cout << b << ", "; } std::cout << '\n';
    // std::cout << "groundtruth"; for (double b : groundtruth) { std::cout << b << ", "; } std::cout << std::endl;
    // std::cout << "dist: " << distance<< std::endl;

    
    // 

    // Planning({-0.15,0.075,0,-1.008,0,-0.11,0}, {0.449,-0.201,0,-0.151,0,-0.11,0});

    // DrawLine(_start, _goal, 1, 2);
    // Node* first_node = new Node(_start);
    // Node* latest_node = new Node(_goal, first_node);
    // VisualizeTree(latest_node, 6, 2);

    return true;
}

void RRTconnect::Planning(State start, State goal)
{
    _start = start;
    _goal = goal;

    DrawEndEffector(_start, 1, 10);
    DrawEndEffector(_goal, 1, 10);
    if(CheckCollision(_start) || CheckCollision(_goal)) {
        cout << "start or goal in collision" << endl; 
        return;
    }

    bool termination = false;
    NodeTree* rrt_tree = new NodeTree(new Node(_start, nullptr));
    std::vector<State>* node_list = new std::vector<State>();
    do{
        auto rand_q = SampleRandomConfig();
        // VisualizeRandSample(rand_q, 6, 1);
        auto nearest_node = NearestNode(rrt_tree, node_list, rand_q);
        termination = LocalPlanner(rrt_tree, node_list, nearest_node, rand_q);
        if (int(rrt_tree->GetTreeSize()) > _params.max_sample_points) {std::cout << "Maximum points are sampled !!!" << std::endl;break;}
        std::cout << "tree_size: " <<  rrt_tree->GetTreeSize() << std::endl;
    }while (!termination);

    Node* end_point;
    if(termination) {
        rrt_tree->Append(new Node(_goal, rrt_tree->GetLatestNode()));
        end_point = rrt_tree->GetLatestNode();
    }
    else end_point = NearestNode(rrt_tree, node_list, _goal);
    std::cout << *end_point << std::endl;
    _success_count.push_back(rrt_tree->GetTreeSize());
    
    
    auto temp_end_point = end_point;
    do{
        VisualizeTree(temp_end_point, _params.draw_path_point, _params.draw_path_line, _green, _red);
        temp_end_point = temp_end_point->parent;
    }while(temp_end_point->parent!=nullptr);

    temp_end_point = end_point;
    auto path = rrt_tree->GetPath(temp_end_point);
    std::cout << "path: " <<  path.size() << std::endl;

    ShortcutSmoothing(path);
    std::cout << "path: " <<  path.size() << std::endl;
    for(size_t i=0;i<path.size()-1;i++){
        Node* child = new Node(path[i]->q,path[i+1]);
        VisualizeTree(child, _params.draw_path_point, _params.draw_path_line, _blue, _green);
    }

    delete rrt_tree;
    rrt_tree = nullptr;
    delete node_list;
    node_list = nullptr;


}

State RRTconnect::SampleRandomConfig()
{
    State q = _goal;
    if(RandomNumber()<_params.goal_bias_probability){
        if(RandomNumber()<_params.goal_bias_probability_second) q = _goal;
        else{
            do{
                q.clear();
                for(std::size_t i=0;i<_goal.size();i++){
                    q.push_back((RandomNumber()-0.5)*2.0*_params.goal_bias_mag + _goal[i]);
                }
            }while (CheckCollision(q));
        }
    }
    else{
        do{
            q.clear();
            for(std::size_t i=0;i<_goal.size();i++){
                q.push_back(RandomNumber()*(_params.upper_limit[i]-_params.lower_limit[i])+_params.lower_limit[i]);
            }
        // }while (false);
        }while (CheckCollision(q));
    }
    return q;
}

Node* RRTconnect::NearestNode(NodeTree* query_tree, std::vector<State>* node_list, State query_point)
{
    // static std::vector<State>* node_list = new std::vector<State>();
    static KDTree* kd_tree;

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

bool RRTconnect::LocalPlanner(NodeTree* rrt_tree, std::vector<State>* node_list, Node* nearest_node, State rand_q)
{
    auto current_q = nearest_node->q;
    auto delta_q = rand_q + current_q*(-1);
    auto unit_step = delta_q*(1/L2Norm(delta_q));

    auto new_q = current_q + unit_step*_params.step_size;

    // cout << "current_q: " << current_q << endl;
    // cout << "rand_q: " << rand_q << endl;
    // cout << "unit_step: " << unit_step << endl;
    // cout << "new_q: " << new_q << endl;
    // cout << "CheckCollision(new_q): " << CheckCollision(new_q) << endl;
    // cout << "IsInWorkspace(new_q): " << IsInWorkspace(new_q) << endl;

    while (!CheckCollision(new_q) && IsInWorkspace(new_q) && L2Norm(NearestNode(rrt_tree, node_list, new_q)->q, new_q)>0.9*_params.step_size) // 
    {
        Node* new_node = new Node(new_q, nearest_node);
        rrt_tree->Append(new_node);
        VisualizeTree(new_node, _params.draw_tree_point, _params.draw_tree_line, _green, _red);

        if (CheckDestination(new_q)) return true;
        // return false; // extend
    
        new_q = new_q + unit_step*_params.step_size;
        nearest_node = new_node;
    }

    return false;
}

bool RRTconnect::CheckCollision(State q)
{
    _robot_ptr->SetDOFValues(q, 1, _leftarm_ptr->GetArmIndices());
    return (GetEnv()->CheckStandaloneSelfCollision(_robot_ptr) || GetEnv()->CheckCollision(_robot_ptr));
}

bool RRTconnect::IsInWorkspace(State q)
{
    for(size_t i=0;i<q.size();i++)
        if(q[i]<_params.lower_limit[i] || q[i]>_params.upper_limit[i]) {
            // cout << "i: " << i << " q[i]:" << q[i] << "_params.lower_limit[i]: " << _params.lower_limit[i] << "_params.upper_limit[i]: " << _params.upper_limit[i] << endl;
            return false;
        }
    return true;
}

bool RRTconnect::CheckDestination(State q)
{
    if (L2Norm(_goal, q) <= _params.step_size*2) return true;
    else return false;
}

void RRTconnect::ShortcutSmoothing(std::vector<Node*>& path)
{
    for(int i=0;i<_smooth_max_iter;i++){
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

bool RRTconnect::CheckLineCollision(State q1, State q2)
{
    auto delta_q = q2 + q1*(-1);
    auto unit_step = delta_q*(1/L2Norm(delta_q));
    auto new_q = q1;
    while(L2Norm(q2, new_q) > _params.step_size*0.1)
    {
        new_q = new_q + unit_step*(_params.step_size*0.1);
        if (CheckCollision(new_q)) return true;
    }

    return false;
}

double RRTconnect::RandomNumber()
{
    return double(rand()) / double(RAND_MAX);
}

double RRTconnect::L2Norm(const State &a, const State &b) 
{
    double distc = 0;
    for (size_t i = 0; i < a.size(); i++) {
        double di = a.at(i) - b.at(i);
        distc += di * di;
    }
    return std::sqrt(distc);
}

double RRTconnect::L2Norm(const State &a) 
{
    double distc = 0;
    for (size_t i = 0; i < a.size(); i++) {
        double di = a.at(i);
        distc += di * di;
    }
    return std::sqrt(distc);
}

// Get the endeffector position
std::vector<float> RRTconnect::GetEEPosition(State& config) 
{
    RobotBase::RobotStateSaver save(_robot_ptr);
    // _robot_ptr->SetActiveDOFValues(config);
    // _robot_ptr->SetActiveManipulator("leftarm");
    _robot_ptr->SetDOFValues(config, 1, _leftarm_ptr->GetArmIndices());
    RobotBase::ManipulatorPtr mani;
    mani = _robot_ptr->GetActiveManipulator();

    RaveVector<float> point = mani->GetEndEffectorTransform().trans;

    std::vector<float> endEffector;
    endEffector.push_back(point.x);
    endEffector.push_back(point.y);
    endEffector.push_back(point.z);
    // std::cout << point.x << ", " << point.y << ", " << point.z << std::endl;

    return endEffector;
}

// Draw a point at the end effector position
void RRTconnect::DrawEndEffector(State config, int color, int size) 
{
    std::vector<float> endEffector = GetEEPosition(config);

    // std::cout << endEffector[0] << ", " << endEffector[1] << ", " << endEffector[2] << std::endl;

    float red[4] = {1,0,0,1};
    float green[4] = {0,1,0,1};
    float blue[4] = {0,0,1,1};

    if(color == 1)
        _handler.push_back(_env->plot3(&endEffector[0],1,1,size,red,0,true));
    else if (color == 2)
        _handler.push_back(_env->plot3(&endEffector[0],1,1,size,green,0,true));
    else if (color == 3)
        _handler.push_back(_env->plot3(&endEffector[0],1,1,size,blue,0,true));
    else {
        RAVELOG_INFO("Unknown Drawing Color\n");
        abort();
    }
}

void RRTconnect::DrawLine(State q1, State q2, int color, int size)
{
    std::vector<float> p1 = GetEEPosition(q1);
    std::vector<float> p2 = GetEEPosition(q2);
    for(size_t i=0;i<p2.size();i++)
        p1.push_back(p2[i]);
    _handler.push_back(_env->drawlinelist(&p1[0],2,4*3,size));
}

void RRTconnect::VisualizeTree(Node* latest_node, int point_size, int line_width, float* point_color, float* line_color)
{
    auto ln = latest_node->q;
    auto pln = latest_node->parent->q;
    std::vector<float> p1 = GetEEPosition(ln);
    std::vector<float> p2 = GetEEPosition(pln);

    if(point_size!=0){
        _handler.push_back(_env->plot3(&p1[0],1,1,point_size,point_color,0,true));
    }

    if(line_width!=0){
        for(size_t i=0;i<p2.size();i++)
        p1.push_back(p2[i]);
        _handler.push_back(_env->drawlinelist(&p1[0],2,4*3,line_width,line_color));
    }
    
}

void RRTconnect::VisualizeRandSample(State q, int point_size, int color)
{
    std::vector<float> p1 = GetEEPosition(q);
    float blue[4] = {0,0,1,1};
    if(point_size!=0){
        _handler.push_back(_env->plot3(&p1[0],1,1,point_size,blue,0,true));
    }
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

