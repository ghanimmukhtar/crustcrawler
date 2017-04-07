#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <crustcrawler_core_msgs/JointCommand.h>
#include <std_msgs/Float64.h>
#include <actionlib_msgs/GoalStatusArray.h>

class Crustcrawler_JointCommand_Translator
{
public:
    Crustcrawler_JointCommand_Translator(std::string name){
        /*Subscribe to the status of the joint action server so that you update the joint commands when they is a goal to be executed*/
        joint_command_sub_ = nh_.subscribe("/crustcrawler/follow_joint_trajectory/status", 1, &Crustcrawler_JointCommand_Translator::joint_trajectory_cb, this);

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

    void joint_trajectory_cb(const actionlib_msgs::GoalStatusArray::ConstPtr &goal_status){
        /*Only if there is a new goal to be executed then publish the joint commands*/
        if(!goal_status->status_list.empty()){
            /*Subscribe to joint command that will come from the joint action server*/
            joint_command_sub_ = nh_.subscribe("/crustcrawler/joint_command", 1, &Crustcrawler_JointCommand_Translator::joint_command_cb, this);
        }
    }

    void joint_command_cb(const crustcrawler_core_msgs::JointCommand::ConstPtr& joint_command){
        /*First fill the joint msgs to be pusblished*/
        for(size_t i = 0; i < joint_command->names.size(); i++){
            if (strcmp(joint_command->names[i].c_str(), "joint_1") == 0)
                j_1_.data = joint_command->command[i];
            else if (strcmp(joint_command->names[i].c_str(), "joint_2") == 0)
                j_2_.data = joint_command->command[i];
            else if (strcmp(joint_command->names[i].c_str(), "joint_3") == 0)
                j_3_.data = joint_command->command[i];
            else if (strcmp(joint_command->names[i].c_str(), "joint_4") == 0)
                j_4_.data = joint_command->command[i];
            else if (strcmp(joint_command->names[i].c_str(), "joint_5") == 0)
                j_5_.data = joint_command->command[i];
            else if (strcmp(joint_command->names[i].c_str(), "joint_6") == 0)
                j_6_.data = joint_command->command[i];
        }
        /*Then publish them*/
        pub_j_1_.publish(j_1_);
        pub_j_2_.publish(j_2_);
        pub_j_3_.publish(j_3_);
        pub_j_4_.publish(j_4_);
        pub_j_5_.publish(j_5_);
        pub_j_6_.publish(j_6_);
        //ROS_WARN("********************* I am publishing over here *************************");
    }

protected:
    ros::NodeHandle nh_;
    ros::Subscriber joint_command_sub_, joint_action_server_sub_;
    ros::Publisher pub_j_1_, pub_j_2_, pub_j_3_, pub_j_4_, pub_j_5_, pub_j_6_;
    std_msgs::Float64 j_1_, j_2_, j_3_, j_4_, j_5_, j_6_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "crustcrawler_jointcommand_translator");
    ROS_WARN("------------- initialized crustcrawler joint command translator ---------------------");
    Crustcrawler_JointCommand_Translator crustcrawler_translator(ros::this_node::getName());

    ros::spin();

    return 0;
}
