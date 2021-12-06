#include "Planner.h"

State Planner::SampleRandomConfig()
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

Node* Planner::NearestNode(NodeTree* query_tree, std::vector<State>* node_list, State query_point)
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

bool Planner::LocalPlanner(NodeTree* rrt_tree, std::vector<State>* node_list, Node* nearest_node, State rand_q)
{
    State current_q = nearest_node->q;
    State delta_q = rand_q + current_q*(-1);
    State unit_step = delta_q*(1/L2Norm(delta_q));
    State new_q = current_q + unit_step*_params.step_size;

    while (!CheckCollision(new_q) && IsInWorkspace(new_q) && L2Norm(NearestNode(rrt_tree, node_list, new_q)->q, new_q)>0.9*_params.step_size) // 
    {
        Node* new_node = new Node(new_q, nearest_node);
        rrt_tree->Append(new_node);
        // VisualizeTree(new_node, _params.draw_tree_point, _params.draw_tree_line, _green, _red);

        if (CheckDestination(new_q)) return true;
        // return false; // extend
    
        new_q = new_q + unit_step*_params.step_size;
        nearest_node = new_node;
    }

    return false;
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

bool Planner::CheckDestination(State q)
{
    if (L2Norm(_goal, q) <= _params.step_size*2) return true;
    else return false;
}

void Planner::ShortcutSmoothing(std::vector<Node*>& path)
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

bool Planner::CheckLineCollision(State q1, State q2)
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