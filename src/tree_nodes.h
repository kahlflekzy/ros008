#include <behavior_tree_navigation_v4/TaskAction.h>
#include <behavior_tree_navigation_v4/StopAction.h>
#include <behavior_tree_navigation_v4/MoveRobotAction.h>
#include <behavior_tree_navigation_v4/Stop.h>

#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include "behaviortree_cpp_v3/action_node.h"

using namespace BT;
using namespace behavior_tree_navigation_v4;
using std::cout;
using std::endl;
using std::string;

typedef actionlib::SimpleActionClient<TaskAction> TaskClient;
typedef actionlib::SimpleActionClient<StopAction> StopClient;
typedef actionlib::SimpleActionClient<MoveRobotAction> MoveRobotClient;

class ExecuteTask : public BT::StatefulActionNode {
    private:
        TaskClient client;
        TaskGoal goal;
        string pose = "";
        int task = 0;
        bool flag;
    public:
        ExecuteTask(const std::string& name, const BT::NodeConfiguration& config):
                    BT::StatefulActionNode(name, config), client("tasks_server", true){
                        ROS_INFO(">>> Waiting for ExecuteTask Server.");
                        flag = client.waitForServer(ros::Duration(3.0));
                    }
        
        static BT::PortsList providedPorts(){
            return {BT::InputPort<string>("message"),
                    BT::OutputPort<string>("goal"),
                    BT::InputPort<int>("new_task"), // read and write
                    BT::OutputPort<int>("new_task")};
        }

        NodeStatus onStart() override {
            if (!flag)
            {   
                ROS_INFO(">>> Failed to connect to tasks_server on time.");
                return NodeStatus::FAILURE;
            }
            int new_task;
            getInput("new_task", new_task);
            if (task == 0 or new_task == 1){
                task = 0;
                getInput("message", goal.task);
                setOutput("new_task", 0);
                ROS_INFO(">>> ExecuteTask: collected new tasks. task=[%i], new_task=[%i]", task, new_task);
                }
            else{
                goal.task = std::to_string(task); // send the goal index to server
                ROS_INFO(">>> ExecuteTask: executing goal [%i]", task);
                }
            client.sendGoal(goal,
                boost::bind(&ExecuteTask::onActionCompleted, this, _1, _2),
                TaskClient::SimpleActiveCallback(),
                TaskClient::SimpleFeedbackCallback()
            );
            return NodeStatus::RUNNING;
        }
        
        void onHalted() override {
            // notify the server that the operation has been aborted. NOT_IMPLEMENTED
            client.cancelAllGoals();
            cout<<">>> TasksListener was Halted"<<endl;
        }

        NodeStatus onRunning() override {
            if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                return NodeStatus::SUCCESS;
            }else{
                return NodeStatus::RUNNING;
            }
        }

        void onActionCompleted(const actionlib::SimpleClientGoalState& state,
                                const TaskResultConstPtr& result){
            string str = result->goal;
            auto parts = splitString(str, '>');
            if (parts[1] == "0"){
                task = 0;
            }else {
                task += 1;
            }
            ROS_INFO(">>> ExecuteTask: done with hint on next action _task [%i]", task);
            pose = string(parts[0]);
            setOutput("goal", str);
        }
};

class StopNode : public BT::SyncActionNode {
    private:
        StopClient client;
        StopGoal goal;
        bool flag;
        int START_STATE = -1;
        int RUN_STATE = -2;
        int value = START_STATE;
        int CONTINUE = Stop::CONTINUE;
    public:
        StopNode(const std::string& name, const BT::NodeConfiguration& config):
                    BT::SyncActionNode(name, config), client("stop_server", true){
                        ROS_INFO(">>> Waiting for StopActionServer.");
                        flag = client.waitForServer(ros::Duration(3.0));
                    }

        static BT::PortsList providedPorts(){
            return {BT::OutputPort<int>("preempt")};
        }

        NodeStatus onStart() {
            if (!flag){
                ROS_INFO(">>> Failed to connect to stop_server on time.");
                return NodeStatus::FAILURE;
            }
            // Not used
            goal.goal = 1;
//            ROS_INFO(">>> StopNode.onStart().");
            setOutput("preempt", CONTINUE);
            client.sendGoal(goal,
                boost::bind(&StopNode::onActionCompleted, this, _1, _2),
                StopClient::SimpleActiveCallback(),
                StopClient::SimpleFeedbackCallback()
            );
            return NodeStatus::RUNNING;
        }

        void hal1t() {
            client.cancelAllGoals();
            cout<<">>> StopClient was Halted"<<endl;
        }

        NodeStatus tick() override {
            while (value < 0){
                if (value == START_STATE){
                    value = RUN_STATE;
                    onStart();
                }
                // else {ROS_INFO(">>> StopNode.tick_running().");}
            }
            value = START_STATE;
            return NodeStatus::SUCCESS;
        }
        void onActionCompleted(const actionlib::SimpleClientGoalState& state, const StopResultConstPtr& result){
            /*
            value changes so fast, it can change before it is set as output
            so we use two different variables.
            */
            int _value = result->result;
            if (_value != CONTINUE){
                setOutput("preempt", _value);
                ROS_INFO(">>> StopNode.onActionCompleted(%i).", _value);
            }
            value = _value;
        }
};

class MoveRobot : public BT::StatefulActionNode {
    private:
        MoveRobotClient client;
        MoveRobotGoal goal;
        int status;
        int stop_code = Stop::CONTINUE;
        int end_task = Stop::CONTINUE;
        bool flag;
        bool new_goal = true;
    public:
        MoveRobot(const std::string& name, const BT::NodeConfiguration& config):
                    BT::StatefulActionNode(name, config), client("move_robot", true){
                        ROS_INFO(">>> Waiting for MoveToBase Server.");
                        flag = client.waitForServer(ros::Duration(3.0));
                    }

        static BT::PortsList providedPorts(){
            return {BT::InputPort<string>("message"),
                    BT::InputPort<int>("preempt"),
                    BT::OutputPort<int>("signal")};
        }

        NodeStatus onStart() override {
            if (!flag)
            {
                ROS_INFO(">>> Failed to connect to MoveToBase server on time.");
                return NodeStatus::FAILURE;
            }
            if (new_goal){
                string goal_str;
                getInput("message", goal_str);
                auto parts = splitString(goal_str, '>');
                if (parts[1] == "0"){
                    end_task = Stop::END_TASK; // stop entire task after this goal.
                }
                goal.goal = string(parts[0]);
                ROS_INFO(">>> MoveRobot.onStart()");
                client.sendGoal(goal,
                    boost::bind(&MoveRobot::onActionCompleted, this, _1, _2),
                    MoveRobotClient::SimpleActiveCallback(),
                    MoveRobotClient::SimpleFeedbackCallback()
                );
                new_goal = false;
            }
            return NodeStatus::RUNNING;
        }

        void onHalted() override {
            // notify the server that the operation has been aborted. NOT_IMPLEMENTED
            client.cancelAllGoals();
            cout<<">>> MoveRobot was Halted"<<endl;
        }

        NodeStatus onRunning() override {
            if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                if (status != 0) // Not really make a difference now.
                {
                    ROS_INFO("Failed to complete task execution. Returning FAILURE.");
                }
                new_goal = true;
                stop_code = Stop::CONTINUE;
                end_task = Stop::CONTINUE;
                return NodeStatus::SUCCESS;
            }else{
                if (stop_code == Stop::CONTINUE){
                    getInput("preempt", stop_code);
                    if (stop_code == Stop::STOP_CURRENT || stop_code == Stop::STOP_ALL)
                    {
                        /* preempt  */
                        ROS_INFO(">>> MoveRobot: preempting task with code: [%i].", stop_code);
                        client.cancelAllGoals();
                    }else if (stop_code == Stop::END_TASK){
                        ROS_INFO(">>> MoveRobot: got signal to prematurely terminate task. Code: [%i].", stop_code);
                    }
                }
                return NodeStatus::RUNNING;
            }
        }

        void onActionCompleted(const actionlib::SimpleClientGoalState& state,
                                const MoveRobotResultConstPtr& result){
            status = result->result;
            ROS_INFO(">>> MoveRobot Completed: END_TASK [%i], STOP_CODE [%i]", end_task, stop_code);
            if (end_task != Stop::CONTINUE){
                setOutput("signal", end_task);
                end_task = Stop::CONTINUE;
                }
            else{setOutput("signal", stop_code);}
        }
};

class FinishTask : public StatefulActionNode {
    public:
        FinishTask(const string& name, const NodeConfiguration& config):
            StatefulActionNode(name, config){}

        static PortsList providedPorts(){
            return {InputPort<int>("signal"),
                    BT::OutputPort<int>("signal"),
                    BT::OutputPort<int>("new_task")};
        }
        NodeStatus onStart() override {return checkStatus();}

        NodeStatus onRunning() override {return checkStatus();}

        NodeStatus checkStatus(){
            int status;
            getInput("signal", status);
            setOutput("new_task", 0);
            if (status == Stop::END_TASK or status == Stop::STOP_ALL){
                // so the loop will break
                int signal = Stop::CONTINUE;
                setOutput("signal", signal);
                setOutput("new_task", 1);
                return NodeStatus::SUCCESS;
            }
            // so the loop will restart
            return NodeStatus::RUNNING;
        }
        void onHalted() override {}
};