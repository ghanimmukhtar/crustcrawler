#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64.h>
//#include <myrabot_arm_base_b/WriteServos.h>
//#include <myrabot_arm_base_b/ReadServos.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#define PI 3.14159265

class Crustcrawler_Arm
{
public:

    Crustcrawler_Arm(std::string name) :
        as_(nh_, name, false),
        action_name_(name)
    {
        as_.registerGoalCallback(boost::bind(&Crustcrawler_Arm::goalCB, this));
        as_.registerPreemptCallback(boost::bind(&Crustcrawler_Arm::preemptCB, this));

        sub_pose_arm_ = nh_.subscribe("/crustcrawler/joint_states", 1, &Crustcrawler_Arm::analysisCB, this);

        //ROS_WARN_STREAM("name space is: " << nh_.getNamespace());
        nh_.getParam("/simulation", simulation_);

        //pub_move_arm_ = nh_.advertise<myrabot_arm_base_b::WriteServos>("/move_arm", 1, this);
        pub_j_1_ = nh_.advertise<std_msgs::Float64>("/crustcrawler/joint_1_position_controller/command", 1, this);
        pub_j_2_ = nh_.advertise<std_msgs::Float64>("/crustcrawler/joint_2_position_controller/command", 1, this);
        pub_j_3_ = nh_.advertise<std_msgs::Float64>("/crustcrawler/joint_3_position_controller/command", 1, this);
        pub_j_4_ = nh_.advertise<std_msgs::Float64>("/crustcrawler/joint_4_position_controller/command", 1, this);
        pub_j_5_ = nh_.advertise<std_msgs::Float64>("/crustcrawler/joint_5_position_controller/command", 1, this);
        pub_j_6_ = nh_.advertise<std_msgs::Float64>("/crustcrawler/joint_6_position_controller/command", 1, this);

        as_.start();
    }

    ~Crustcrawler_Arm(void)
    {
    }

    void goalCB()
    {
        i_ = 0;

        goal_ = as_.acceptNewGoal()->trajectory;
    }

    void preemptCB()
    {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        as_.setPreempted();
    }

    void analysisCB(const sensor_msgs::JointState::ConstPtr &joint_states)
    {
        //ROS_WARN("********* I am at the start of the call back ********");
        if (!as_.isActive())
            return;

        ros::Time actual_time = ros::Time::now();

        feedback_.header.stamp = actual_time;

        feedback_.joint_names.resize(6);
        feedback_.actual.positions.resize(6);
        feedback_.actual.effort.resize(6);

        /*for simulated one where the order is correct*/
        if(simulation_){
            //ROS_WARN("I am at simulation");
            feedback_.joint_names = joint_states->name;
            feedback_.actual.positions = joint_states->position;
            feedback_.actual.effort = joint_states->effort;

        }
        /*for real robot where the order for now is: {'left_finger_joint', 'right_finger_joint', 'joint_6', 'joint_4', 'joint_5', 'joint_2', 'joint_3', 'joint_1'}*/
        else{
            //ROS_WARN("I am at the real robot");
            feedback_.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
            feedback_.actual.positions = {joint_states->position[7], joint_states->position[5], joint_states->position[6],
                                          joint_states->position[3], joint_states->position[4], joint_states->position[2]};
            feedback_.actual.effort = {joint_states->effort[7], joint_states->effort[5], joint_states->effort[6],
                                       joint_states->effort[3], joint_states->effort[4], joint_states->effort[2]};
        }


        points_ = goal_.points.size();

        if (i_ != points_)
        {
            //ROS_WARN_STREAM("********* I am trying to fill and publish commands and the size of trajectory is: ******** " << points_);

            //myrabot_arm_base_b::WriteServos move;

            if (i_ == 0
                    ||((feedback_.actual.positions[0] <= (goal_.points[i_-1].positions[0] + 0.1) && feedback_.actual.positions[0] >= (goal_.points[i_-1].positions[0] - 0.1))
                       && (feedback_.actual.positions[1] <= (goal_.points[i_-1].positions[1] + 0.1) && feedback_.actual.positions[1] >= (goal_.points[i_-1].positions[1] - 0.1))
                       && (feedback_.actual.positions[2] <= (goal_.points[i_-1].positions[2] + 0.1) && feedback_.actual.positions[2] >= (goal_.points[i_-1].positions[2] - 0.1))
                       && (feedback_.actual.positions[3] <= (goal_.points[i_-1].positions[3] + 0.1) && feedback_.actual.positions[3] >= (goal_.points[i_-1].positions[3] - 0.1))
                       && (feedback_.actual.positions[4] <= (goal_.points[i_-1].positions[4] + 0.1) && feedback_.actual.positions[4] >= (goal_.points[i_-1].positions[4] - 0.1))
                       && (feedback_.actual.positions[5] <= (goal_.points[i_-1].positions[5] + 0.1) && feedback_.actual.positions[5] >= (goal_.points[i_-1].positions[5] - 0.1))))
            {
                j_1_.data = goal_.points[i_].positions[0];
                j_2_.data = goal_.points[i_].positions[1];
                j_3_.data = goal_.points[i_].positions[2];
                j_4_.data = goal_.points[i_].positions[3];
                j_5_.data = goal_.points[i_].positions[4];
                j_6_.data = goal_.points[i_].positions[5];

                pub_j_1_.publish(j_1_);
                pub_j_2_.publish(j_2_);
                pub_j_3_.publish(j_3_);
                pub_j_4_.publish(j_4_);
                pub_j_5_.publish(j_5_);
                pub_j_6_.publish(j_6_);

                //for(int i = 0; i < goal_.points[i_].velocities.size(); i++)


                //ROS_WARN("********* I am publishing ********");

                t_initial = ros::Time::now();
            }
            else
            {
                i_ = i_ - 1;
            }

            ros::Time actual_time = ros::Time::now();

            ros::Duration duration(1.0);

            if (actual_time > (t_initial + duration))
            {
                result_.error_code = -1;
                as_.setAborted(result_);
            }

            feedback_.desired = goal_.points[i_];

            feedback_.error.positions.resize(6);

            for (int j = 0; j < 6; j++)
            {
                feedback_.error.positions[j] = feedback_.desired.positions[j] - feedback_.actual.positions[j];
            }

            i_ ++;
        }
        else
        {
            result_.error_code = 0;
            as_.setSucceeded(result_);
        }
        as_.publishFeedback(feedback_);

    }

protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
    std::string action_name_;
    int points_, i_;
    ros::Time t_initial;
    trajectory_msgs::JointTrajectory goal_;
    control_msgs::FollowJointTrajectoryResult result_;
    control_msgs::FollowJointTrajectoryFeedback feedback_;
    ros::Subscriber sub_pose_arm_;
    ros::Publisher pub_j_1_, pub_j_2_, pub_j_3_, pub_j_4_, pub_j_5_, pub_j_6_;
    std_msgs::Float64 j_1_, j_2_, j_3_, j_4_, j_5_, j_6_;
    bool simulation_ = true;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "follow_joint_trajectory");

    ROS_WARN("------------- initialized the joint trajectory node ---------------------");
    Crustcrawler_Arm crustcrawler(ros::this_node::getName());


    ros::spin();

    return 0;
}
