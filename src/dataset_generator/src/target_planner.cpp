#include <ros/ros.h>
#include <memory>
// MoveitCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>

#include <geometry_msgs/PointStamped.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf/transform_listener.h>

#include <std_srvs/SetBool.h>
// #include <std_srvs/SetBoolRequest.h>

// namespace rvt = rviz_visual_tools;


class Planner
{
    public:

    moveit_cpp::PlanningComponentPtr planning_components;

    Planner(ros::NodeHandle &nh)
    {
        //ros::AsyncSpinner spinner(4);
        //spinner.start();
        static const std::string PLANNING_GROUP = "manipulator";

        // Otherwise robot with zeros joint_states
        ros::Duration(1.0).sleep();

        ROS_INFO("Starting MoveIt.");

        auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(nh);
        moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();

        planning_components = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
        auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
        auto robot_start_state = planning_components->getStartState();
        auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);

    }

    planning_interface::MotionPlanResponse plan_random_target(moveit_cpp::PlanningComponentPtr &planning_components)//, tf::StampedTransform transform)
    {
        // set the start state of the plan to the current state of the robot
        planning_components->setStartStateToCurrentState();

        // set a goal using PoseStamped
        geometry_msgs::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";

        target_pose.pose.orientation.x = random(2000, 1000);
        target_pose.pose.orientation.y = random(2000, 1000);
        target_pose.pose.orientation.z = random(2000, 1000);
        target_pose.pose.orientation.w = random(2000, 1000);

        target_pose.pose.position.x = random(1400, 700);
        target_pose.pose.position.y = random(1400, 700);
        target_pose.pose.position.z = random(1400, 700);
        planning_components->setGoal(target_pose, "link_6");

        ROS_INFO("Target:\n\tT:\n\t\tx:%f\n\t\ty:%f\n\t\tz:%f\n\tR:\n\t\tx:%f\n\t\ty:%f\n\t\tz:%f\n\t\tw:%f", 
            target_pose.pose.position.x, 
            target_pose.pose.position.y, 
            target_pose.pose.position.z, 
            target_pose.pose.orientation.x, 
            target_pose.pose.orientation.y, 
            target_pose.pose.orientation.z, 
            target_pose.pose.orientation.w);

        // PlanningComponents computes the plan and visualizes it.
        return planning_components->plan();
    }

    float random(const int &mod, const int &offset)
    {
        float ran = std::rand() % mod - offset;
        return ran/1000;
    }

    bool handle_req(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        auto solution = plan_random_target(planning_components);

        while(!solution)
        {
            solution = plan_random_target(planning_components);
        }

        if (solution)
        {
            res.success = true;
            res.message = "Success";
            planning_components->execute(); // Execute the plan
            // allow time to move
            ros::Duration(3).sleep();
            return true;
        }
        else 
        {
            res.success = false;
            res.message = "Failed";
            return false;
        }
    }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_planner");
    ros::NodeHandle nh("/target_planner");

    Planner planner(nh);

    ros::ServiceServer service = nh.advertiseService("plan_request", &Planner::handle_req, &planner);

    tf::TransformListener listener;

    while (nh.ok())
    {
        ros::spin();
    }

    ROS_INFO("Shutting down.");
    ros::waitForShutdown();
}
