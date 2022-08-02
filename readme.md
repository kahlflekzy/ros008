# BehaviorTree Robot Services With Ability to Stop Goals Arbitrarily.
- [BehaviorTree Robot Services With Ability to Stop Goals Arbitrarily.](#behaviortree-robot-services-with-ability-to-stop-goals-arbitrarily)
  - [Introduction](#introduction)
  - [Start](#start)
    - [Workspace](#workspace)
    - [Install Dependencies](#install-dependencies)
  - [Create package](#create-package)
    - [Tree](#tree)
    - [Launch](#launch)
    - [Message](#message)
    - [Server Nodes](#server-nodes)
      - [Task](#task)
      - [Stop](#stop)
      - [MoveRobot](#moverobot)
      - [Stop Publisher](#stop-publisher)
    - [BehaviorTree (ActionLib clients)](#behaviortree-actionlib-clients)
      - [ExecuteTask](#executetask)
      - [StopNode](#stopnode)
      - [MoveRobot](#moverobot-1)
      - [FinishTask](#finishtask)
  - [Execution](#execution)
    - [Test](#test)
  - [Future Work](#future-work)
  - [References](#references)
## Introduction
In this work, I created a BT robot control implementation that moves the robot to different stages and loops to await more 
instructions. The novel addition in this project is the robot is able to stop at any point and at different stages of 
the task execution. 

## Start

### Workspace 
Created a workspace.

### Install Dependencies
Install `BehaviorTree` and `Turtblebot3 Navigation Simulations` from GitHub. Refer to 
[ros005](https://github.com/kahlflekzy/ros005) for details on this.  
Then run `catkin_make`

Also cloned `behavior_tree_navigation_v3` via
```
git clone https://github.com/kahlflekzy/ros007.git
```
then run `catkin_make`. You might get an error the first time because a message type used in the `BehaviorTree` node 
needs to be built first. Once it is built, the second run of `catkin_make` runs smoothly.

## Create package
```
catkin_create_pkg behavior_tree_navigation_v4 behavior_tree_navigation_v3 actionlib \
behaviortree_cpp_v3 rospy roscpp actionlib_msgs geometry_msgs nav_msgs std_msgs \
message_generation message_runtime
```
Run `catkin_make`

### Tree
I created a BT that uses reactive sequences. It is a bit involved. Particularly, the tree needs loop to get a 
Task `PoseTimeArray`, then executes goals `PoseTime` from the task sequentially. This involves a loop. Now while 
executing the goal, an instruction can be sent for it to terminate the goal, 
1. Terminate immediately, and move to the next goal,
2. Terminate Immediately and end entire task,
3. Complete current goal and end task.

The process of building this feature proved tricky, because of the way communication is done in BT. I used 
`StatefulActionNodes` however, these nodes don't function as expected in Reactive Sequences. I had to use 
`StatefulActionNodes` in order to properly communicate with `ROS`. And this is primarily what complicates the whole 
process. 

### Launch
I simply copied the `behaviortree_navigation.launch` launch file from `v3` and edited it to reflect the new nodes.

### Message
Create a `Stop` message to encapsulate the Stop signals as described above. See the file `Stop.msg` for details.  
Secondly, due to my new implementation, I had to create a `PoseTimeStamped` Message. It depends on _**v3**_ `PoseTime`
so I included `behavior_tree_navigation_v3`. I used this message type in `task_node` and `move_robot_node`. See those 
files for more details. 

Add the section in the `generate_messages` dependencies. And ran `catkin_make` twice or thrice.

Follow the instructions in [ros006](https://github.com/kahlflekzy/ros006) to build the messages.

I noticed however that using `CATKIN_DEPENDS` raised errors. Particularly I let all the dependencies show instead of 
just `message_runtime`. Commenting out the line ran successfully.

### Server Nodes
#### Task
I created a `Task` action. For the `execute_task_node`. It takes in a tasks (`PoseTimeArray` encoded) string and 
returns a goal (`PoseTimeStamped` encoded) string based on indexes passed through the goal. It had some hacks. The first time 
it gets an encoded `PoseTimeArray` and returns the goal at pose-index 0, but the next time it is called, it gets a 
string with an integer that indicates which goal to return. The execute method handles all these.

Furthermore, if all poses have been executed, it appends _0_ to the encoded pose it would return otherwise it appends a 
_1_ to show there are more goals.

Refer to [ros005](https://github.com/kahlflekzy/ros005) for building action files.

#### Stop
The stop node mainly creates a subscriber which listens for stop messages and communicates the appropriate `actionlib` 
client.

The current implementation stores stop messages in a list, which **isn't** a correct implementation. Of course, it only
communicates with the client when the client is active and makes a request. But the current implementation stores the 
codes, so a future request from the client can get a code that was stored in the past.

We will update this implementation in the future.

#### MoveRobot
The execute method takes in a `PoseTimeStamped` goal deserializes and converts it to 
a `PoseStamp` then it publishes this to move the robot. Now while the robot moves, it polls to listen for preemptive 
signals. A preempt signal, tells it to cancel the action, effectively stopping the robot motion. It achieves this by 
publishing the current goal to a cancel topic. See the file `move_robot_node.py` for details. Beyond that it's not much 
different from previous implementations which move the robot and wait for some seconds.

#### Stop Publisher
This node simply publishes stop messages for testing. See the file for details.

### BehaviorTree (ActionLib clients)
One of the main challenges in the BT implementation is that ReactiveSequences only repeat while a child returns failure 
or RUNNING. However, in the loop code we create to repeatedly tick the tree, this is difficult to achieve. A FAILURE 
status stops the tree. This is what we want in most cases, except for ReactiveSequences. But it's impossible to make 
this work, so we had to abandon the idea of using FAILURE status codes to make the `ReactiveSequence` repeat.

#### ExecuteTask
Here we use ports which get a `task`  (PoseTimeArray encoded string), then writes a goal (`PoseTimeStamped`), another 
port reads/writes if to get a new task or send goals from current task. The actionlib client for this node gets a reply
from the server, made up of two parts, the first is the encoded pose_time_stamped, while the second part tells if there
are more goals in the task or not this is written to a blackboard message that signals to end task or not. 

#### StopNode
This node is an action node that checks if there is a stop signal, so it signals move_robot to preempt the current goal.
It runs as some sort of condition node to move_robot.

#### MoveRobot
Gets the message from `ExecuteTask`, and moves the robot, also checks for stop messages from the StopNode to see if it 
should preempt goal or not.

#### FinishTask 
Aggregates the stop signals and determines what to do next. A simple SyncActionNode would do here, the current 
implementation uses a `StatefulActionNode`.

Remember to **ALWAYS** implement `onStart`, `onRunning` and `onHalted` for `StatefulActionNode`

## Execution
In one terminal, run the instructions below line by line, the bot model can be `burger`, `waffle` or `waffle_pi`
```
source devel/setup.bash
export TURTLEBOT3_MODEL=waffle
conda deactivate
roslaunch behavior_tree_navigation_v3 turtlebot3_navigation.launch
```
In another terminal, note that the `args` are optional.
```
source devel/setup.bash
roslaunch behavior_tree_navigation_v4 behaviortree_navigation.launch [map_file:=path_to_map.yaml] [init_pose_file:=path_to_init_pose_file.dat]
```
Then in yet another terminal
```
rosrun behavior_tree_navigation_v3 task_publisher.py
```
And finally,
```
rosrun behavior_tree_navigation_v4 stop_publisher.py
``` 

### Test
1. Execute a 1 goal task and return to base, successfully execute another 1 goal task and return to base.
2. Execute a 2 goal task and return to base, successfully execute a 1 goal task and return to base.
3. Execute a 3 goal task and return to base, successfully execute a 2 goal task and return to base.
4. Execute a 1 goal task, send Stop::1 should return to base.
5. Execute a 1 goal task, send Stop::2 should return to base.
6. Execute a 2 goal task, send Stop::1 should go to next goal then base.
7. Execute a 2 goal task, send Stop::2 should return to base.
8. Execute a 3 goal task, send Stop::2 while moving to first goal (or any other) robot should get to goal then return to base.

## Future Work
The future work would involve rewriting most of the entire implementation. The tree design is great, but the exact way 
it functions is quite a messy and complicated. This is due to the usage of actionlib and unnecessary 
`StatefulActionNode` nodes. A better way should be found of implementing the nodes that deal with movements primarily 
in the BT nodes, and receive messages directly from ROS. This will reduce the complication and logical errors.

Furthermore, I noticed the Navigation package was a bit buggy, sometimes the robot moves past the base other times, it 
first moves way from the base before getting to start moving to the base.

Maybe we should move the project to `ROS2`

## References
1. [Sequence Nodes](https://www.behaviortree.dev/sequencenode/)
2. [ActionLib API](https://docs.ros.org/en/api/actionlib/html/index.html)
3. [To String](https://stackoverflow.com/questions/5590381/easiest-way-to-convert-int-to-string-in-c)
4. [Pointers](https://stackoverflow.com/a/1481954)
5. [Reactive Sequences Ticks](https://github.com/BehaviorTree/BehaviorTree.CPP/issues/228#issuecomment-690124021)
