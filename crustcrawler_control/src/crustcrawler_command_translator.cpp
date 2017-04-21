#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <crustcrawler_core_msgs/JointCommand.h>
#include <std_msgs/Float64.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>

class Crustcrawler_JointCommand_Translator
{
public:
    Crustcrawler_JointCommand_Translator(std::string name){
        /*Subscribe to the status of the joint action server so that you update the joint commands when they is a goal to be executed*/
        joint_command_sub_ = nh_.subscribe("/crustcrawler/follow_joint_trajectory/goal", 1,
                                           &Crustcrawler_JointCommand_Translator::joint_trajectory_new_goal_cb, this);
        joint_state_sub_ = nh_.subscribe("/crustcrawler/joint_states", 1, &Crustcrawler_JointCommand_Translator::joint_state_cb, this);

        /*Those are the publisher which will be used to translate each command received from the joint action server to all joints*/
        pub_j_1_ = nh_.advertise<std_msgs::Float64>("/crustcrawler/joint_1_position_controller/command", 1, this);
        pub_j_2_ = nh_.advertise<std_msgs::Float64>("/crustcrawler/joint_2_position_controller/command", 1, this);
        pub_j_3_ = nh_.advertise<std_msgs::Float64>("/crustcrawler/joint_3_position_controller/command", 1, this);
        pub_j_4_ = nh_.advertise<std_msgs::Float64>("/crustcrawler/joint_4_position_controller/command", 1, this);
        pub_j_5_ = nh_.advertise<std_msgs::Float64>("/crustcrawler/joint_5_position_controller/command", 1, this);
        pub_j_6_ = nh_.advertise<std_msgs::Float64>("/crustcrawler/joint_6_position_controller/command", 1, this);
    }

    ~Crustcrawler_JointCommand_Translator(void)
    {

    }

    void joint_state_cb(const sensor_msgs::JointState::ConstPtr& jo_state){
        joint_state_ = *jo_state;
    }

    void joint_trajectory_new_goal_cb(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& new_goal){

        for(size_t i = 0; i < new_goal->goal.trajectory.points.size(); i++){
            j_1_.data = new_goal->goal.trajectory.points[i].positions[distance(new_goal->goal.trajectory.joint_names.begin(),
                                                                               find(new_goal->goal.trajectory.joint_names.begin(),
                                                                                    new_goal->goal.trajectory.joint_names.end(),
                                                                                    "joint_1"))];
            j_2_.data = new_goal->goal.trajectory.points[i].positions[distance(new_goal->goal.trajectory.joint_names.begin(),
                                                                               find(new_goal->goal.trajectory.joint_names.begin(),
                                                                                    new_goal->goal.trajectory.joint_names.end(),
                                                                                    "joint_2"))];
            j_3_.data = new_goal->goal.trajectory.points[i].positions[distance(new_goal->goal.trajectory.joint_names.begin(),
                                                                               find(new_goal->goal.trajectory.joint_names.begin(),
                                                                                    new_goal->goal.trajectory.joint_names.end(),
                                                                                    "joint_3"))];
            j_4_.data = new_goal->goal.trajectory.points[i].positions[distance(new_goal->goal.trajectory.joint_names.begin(),
                                                                               find(new_goal->goal.trajectory.joint_names.begin(),
                                                                                    new_goal->goal.trajectory.joint_names.end(),
                                                                                    "joint_4"))];
            j_5_.data = new_goal->goal.trajectory.points[i].positions[distance(new_goal->goal.trajectory.joint_names.begin(),
                                                                               find(new_goal->goal.trajectory.joint_names.begin(),
                                                                                    new_goal->goal.trajectory.joint_names.end(),
                                                                                    "joint_5"))];
            j_6_.data = new_goal->goal.trajectory.points[i].positions[distance(new_goal->goal.trajectory.joint_names.begin(),
                                                                               find(new_goal->goal.trajectory.joint_names.begin(),
                                                                                    new_goal->goal.trajectory.joint_names.end(),
                                                                                    "joint_6"))];
            /*Then publish them*/
            pub_j_1_.publish(j_1_);
            pub_j_2_.publish(j_2_);
            pub_j_3_.publish(j_3_);
            pub_j_4_.publish(j_4_);
            pub_j_5_.publish(j_5_);
            pub_j_6_.publish(j_6_);
            usleep(1e4);
        }
    }


protected:
    ros::NodeHandle nh_;
    ros::Subscriber joint_command_sub_, joint_action_server_sub_, joint_state_sub_;
    ros::Publisher pub_j_1_, pub_j_2_, pub_j_3_, pub_j_4_, pub_j_5_, pub_j_6_;
    std_msgs::Float64 j_1_, j_2_, j_3_, j_4_, j_5_, j_6_;
    sensor_msgs::JointState joint_state_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "crustcrawler_jointcommand_translator");
    ROS_WARN("------------- initialized crustcrawler joint command translator ---------------------");
    Crustcrawler_JointCommand_Translator crustcrawler_translator(ros::this_node::getName());

    ros::spin();

    return 0;
}
