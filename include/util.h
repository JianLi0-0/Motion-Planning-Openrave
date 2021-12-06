#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <stdlib.h>
#include <time.h> 
#include <math.h>
#include <iostream>
#include <numeric>
#include "NodeTree.h"
using namespace OpenRAVE;

struct OpenravePtr{
    EnvironmentBasePtr _env;
    RobotBasePtr _robot_ptr;
    RobotBase::ManipulatorPtr _leftarm_ptr;
    std::vector<GraphHandlePtr> _handler;
    std::vector<GraphHandlePtr> _tree_handler;
    std::vector<GraphHandlePtr> _path_handler;
};

float _red[4] = {1,0,0,1};
float _green[4] = {0,1,0,1};
float _blue[4] = {0,0,1,1};

// Get the endeffector position
std::vector<float> GetEEPosition(OpenravePtr openrave_ptr, State& config) 
{
    RobotBase::RobotStateSaver save(openrave_ptr._robot_ptr);
    openrave_ptr._robot_ptr->SetDOFValues(config, 1, openrave_ptr._leftarm_ptr->GetArmIndices());
    RobotBase::ManipulatorPtr mani;
    mani = openrave_ptr._robot_ptr->GetActiveManipulator();

    RaveVector<float> point = mani->GetEndEffectorTransform().trans;

    std::vector<float> endEffector;
    endEffector.push_back(point.x);
    endEffector.push_back(point.y);
    endEffector.push_back(point.z);
    // std::cout << point.x << ", " << point.y << ", " << point.z << std::endl;

    return endEffector;
}

// Draw a point at the end effector position
void DrawEndEffector(OpenravePtr openrave_ptr, State config, int color, int size) 
{
    std::vector<float> endEffector = GetEEPosition(openrave_ptr, config);

    // std::cout << endEffector[0] << ", " << endEffector[1] << ", " << endEffector[2] << std::endl;

    float red[4] = {1,0,0,1};
    float green[4] = {0,1,0,1};
    float blue[4] = {0,0,1,1};

    if(color == 1)
        openrave_ptr._handler.push_back(openrave_ptr._env->plot3(&endEffector[0],1,1,size,red,0,true));
    else if (color == 2)
        openrave_ptr._handler.push_back(openrave_ptr._env->plot3(&endEffector[0],1,1,size,green,0,true));
    else if (color == 3)
        openrave_ptr._handler.push_back(openrave_ptr._env->plot3(&endEffector[0],1,1,size,blue,0,true));
    else {
        RAVELOG_INFO("Unknown Drawing Color\n");
        abort();
    }
}

void DrawLine(OpenravePtr openrave_ptr, State q1, State q2, int color, int size)
{
    std::vector<float> p1 = GetEEPosition(openrave_ptr, q1);
    std::vector<float> p2 = GetEEPosition(openrave_ptr, q2);
    for(size_t i=0;i<p2.size();i++)
        p1.push_back(p2[i]);
    openrave_ptr._handler.push_back(openrave_ptr._env->drawlinelist(&p1[0],2,4*3,size));
}

void VisualizeTree(OpenravePtr openrave_ptr, Node* latest_node, int point_size, int line_width, float* point_color, float* line_color)
{
    auto ln = latest_node->q;
    auto pln = latest_node->parent->q;
    std::vector<float> p1 = GetEEPosition(openrave_ptr, ln);
    std::vector<float> p2 = GetEEPosition(openrave_ptr, pln);

    if(point_size!=0){
        openrave_ptr._handler.push_back(openrave_ptr._env->plot3(&p1[0],1,1,point_size,point_color,0,true));
    }

    if(line_width!=0){
        for(size_t i=0;i<p2.size();i++)
        p1.push_back(p2[i]);
        openrave_ptr._handler.push_back(openrave_ptr._env->drawlinelist(&p1[0],2,4*3,line_width,line_color));
    }
    
}

void VisualizeRandSample(OpenravePtr openrave_ptr, State q, int point_size, int color)
{
    std::vector<float> p1 = GetEEPosition(openrave_ptr, q);
    float blue[4] = {0,0,1,1};
    if(point_size!=0){
        openrave_ptr._handler.push_back(openrave_ptr._env->plot3(&p1[0],1,1,point_size,blue,0,true));
    }
}