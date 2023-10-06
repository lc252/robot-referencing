#include <ros/ros.h>
#include <memory>
// MoveitCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>

#include <geometry_msgs/PointStamped.h>

#include <tf/transform_listener.h>

class target_planner_node
{
public:
    target_planner_node(moveit_cpp::PlanningComponentPtr in_planning_components):
    nh{},
    pose_sub(nh.subscribe("unity_objects/planning_axis", 1, &target_planner_node::pose_cb, this))
    {
        planning_components = in_planning_components;
    }

    ~target_planner_node()
    {
        ;
    }

    void timer_cb(const ros::TimerEvent&)
    {
                // set the start state of the plan to the current state of the robot
        planning_components->setStartStateToCurrentState();

        planning_components->setGoal(pose, "link_6");

        // Plan
        auto plan_solution = planning_components->plan();

        // Check if PlanningComponents succeeded in finding the plan
        if (plan_solution)
        {
            planning_components->execute(); // Execute the plan
        }
    }

private:
    // ros
    ros::NodeHandle nh;
    ros::Subscriber pose_sub;
    ros::Timer timer;

    // moveit
    moveit_cpp::PlanningComponentPtr planning_components;
    geometry_msgs::PoseStamped pose;

    void pose_cb(geometry_msgs::PoseStamped posemsg)
    {
        pose = posemsg;
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_planner");
    ros::NodeHandle nh("/target_planner");
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Duration(1.0).sleep();

    ROS_INFO("Starting MoveIt.");

    auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(nh);
    moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();

    const std::string PLANNING_GROUP = "manipulator";
    moveit_cpp::PlanningComponentPtr planning_components = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
    auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
    auto robot_start_state = planning_components->getStartState();
    auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);

    target_planner_node tpn(planning_components);
    ros::Timer timer = nh.createTimer(ros::Duration(10), &target_planner_node::timer_cb, &tpn);

    ros::waitForShutdown();
}
