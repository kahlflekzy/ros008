#include "behaviortree_cpp_v3/bt_factory.h"
#include <ros/ros.h>
#include "tree_nodes_v3.h"
#include "tree_nodes.h"

using namespace BT;
using std::string;
using std::cout;
using std::endl;
using std::chrono::milliseconds;

int main(int argc, char** argv){
    ros::init(argc, argv, "behaviortree_node");
    ROS_INFO("Initialised BehaviorTree Node");

    char tmp[256];
    string path;
    getcwd(tmp, 256);
    string tree_file = "tree01.xml";
    path = string(tmp) + "/src/behavior_tree_navigation_v4/src/tree/"+tree_file;

    if (argc > 1)
    {
        path = string(argv[1]) + "/src/tree/"+tree_file;
    }

    ROS_INFO("Tree File: %s", path.c_str());
    
    BehaviorTreeFactory factory;

    factory.registerNodeType<behavior_tree_navigation_v3::LoadMap>("LoadMap");
    factory.registerNodeType<behavior_tree_navigation_v3::Localization>("Localization");
    factory.registerNodeType<behavior_tree_navigation_v3::TaskListener>("TaskListener");
    factory.registerNodeType<ExecuteTask>("ExecuteTask");
    factory.registerNodeType<MoveRobot>("MoveRobot");
    factory.registerNodeType<StopNode>("Stop");
    factory.registerNodeType<FinishTask>("FinishTask");
    factory.registerNodeType<behavior_tree_navigation_v3::MoveToBase>("MoveToBase");

    ROS_INFO("Creating Nodes.");
    auto tree = factory.createTreeFromFile(path);

    NodeStatus status = NodeStatus::RUNNING;
    ROS_INFO("Starting Tree.");

    while (status == NodeStatus::RUNNING or status == NodeStatus::SUCCESS){
        if (ros::ok()){
            status = tree.tickRoot();
            tree.sleep(milliseconds(100)); // recommended
        }else{
            tree.haltTree();
            status = NodeStatus::IDLE;
        }
    }
    if (status == NodeStatus::IDLE)
    {
        cout<<"Halted Tree Execution!"<<endl;
    }else {
        ROS_INFO("Finished Tree Execution!");
    }
    return 0;
}