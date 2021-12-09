#include "Planner.h"

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

Node* Planner::NearestNode(NodeTree* query_tree, State query_point)
{
    auto tree_size = query_tree->GetTreeSize();
    if( tree_size<50 || tree_size%50==0 )
        query_tree->set_kd_tree(new KDTree(query_tree));
    
    auto index = query_tree->get_kd_tree()->nearest_index(query_point*_params.weights);
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
    if (L2Norm(goal, q) <= _params.stop_step_size) return true;
    else return false;
}

void Planner::ShortcutSmoothing(std::vector<Node*>& path)
{
    auto point1 = path.begin();
    do{
        while(point1+2!=path.end())
        {
            auto point2 = point1 + 2;
            if (!CheckLineCollision((*point1)->q, (*point2)->q))
                path.erase(point1+1, point2);
            else
                break;
        }
        point1++;
    }while(point1!=path.end()-1);
}

void Planner::RandomShortcutSmoothing(std::vector<Node*>& path)
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
    while(L2Norm(q2, new_q) > _params.step_size*0.3)
    {
        new_q = new_q + unit_step*(_params.step_size*0.15);
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

bool Planner::Connect(NodeTree* rrt_tree, KDTree* &kd_tree, Node* nearest_node, State rand_q)
{
    auto current_q = nearest_node->q;
    auto delta_q = rand_q + current_q*(-1);
    auto unit_step = delta_q*(1/L2Norm(delta_q));

    auto new_q = current_q + unit_step*_params.step_size;

    while (IsInWorkspace(new_q) && !CheckCollision(new_q) && L2Norm(NearestNode(rrt_tree, new_q)->q, new_q)>0.9*_params.step_size) // 
    {
        Node* new_node = new Node(new_q, nearest_node);
        rrt_tree->Append(new_node);
        // if (CheckDestination(new_q, _goal)) return true;
        new_q = new_q + unit_step*_params.step_size;
        nearest_node = new_node;
    }

    return false;
}

bool Planner::Extend(NodeTree* rrt_tree, KDTree* &kd_tree, Node* nearest_node, State rand_q)
{
    auto current_q = nearest_node->q;
    auto delta_q = rand_q + current_q*(-1);
    auto unit_step = delta_q*(1/L2Norm(delta_q));

    auto new_q = current_q + unit_step*_params.step_size;

    if (IsInWorkspace(new_q) && !CheckCollision(new_q) && L2Norm(NearestNode(rrt_tree, new_q)->q, new_q)>0.9*_params.step_size) // 
    {
        Node* new_node = new Node(new_q, nearest_node);
        rrt_tree->Append(new_node);
    }

    return false;
}
