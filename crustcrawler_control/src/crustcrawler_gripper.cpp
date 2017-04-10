#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <crustcrawler_core_msgs/EndEffectorCommand.h>
#include <crustcrawler_core_msgs/EndEffectorState.h>
#include <std_msgs/Float64.h>
#include <actionlib_msgs/GoalStatusArray.h>

class Crustcrawler_Gripper
{
public:
    Crustcrawler_Gripper(std::string name){
        /*As soon as the class is instantiated a gripper state publisher start publishing a dummy state with right and left fingers state
         * First subsribe to joint_states and get fingers positions
        */
        joint_state_sub_ = nh_.subscribe("/crustcrawler/joint_states", 1, &Crustcrawler_Gripper::fingers_position_cb, this);
        gripper_state_pub_ = nh_.advertise<crustcrawler_core_msgs::EndEffectorState>("/crustcrawler/end_effector/gripper/state", 1, this);


        /*Create a subscriber to the gripper action server to execute the commands*/
        gripper_action_sub_ = nh_.subscribe("/crustcrawler/end_effector/gripper/command", 1, &Crustcrawler_Gripper::gripper_action_cb, this);

        /*Create the publisher to the fingers joints, so commands could be executed (openning and closing)*/
        right_finger_command_pub_ = nh_.advertise<std_msgs::Float64>("/crustcrawler/joint_right_finger_position_controller/command", 1, this);
        left_finger_command_pub_ = nh_.advertise<std_msgs::Float64>("/crustcrawler/joint_left_finger_position_controller/command", 1, this);

        /*Fill gripper joints names*/
        gripper_joints_names_.resize(2);
        fingers_positions_.resize(2);
        gripper_joints_names_ = {"left_finger_joint", "right_finger_joint"};

        nh_.getParam("/simulation", simulation_);
    }

    ~Crustcrawler_Gripper(void)
    {
    }

    /*As soon as you get fingers position fill a gripper state msg and publish it*/
    void fingers_position_cb(const sensor_msgs::JointState::ConstPtr& fingers_state){
        /*Extract right and left joints position */
        for(size_t i = 0; i < gripper_joints_names_.size(); i++)
            fingers_positions_[i] = fingers_state->position[distance(fingers_state->name.begin(),
                                                                     find(fingers_state->name.begin(),
                                                                          fingers_state->name.end(),
                                                                          gripper_joints_names_[i]))];
        left_finger_position_ = fingers_positions_[0];
        right_finger_position_ = fingers_positions_[1];
        gripper_state_msg_.id = 1;
        gripper_state_msg_.position = (left_finger_position_ + right_finger_position_) / 2.0;
        gripper_state_msg_.enabled = static_cast<uint8_t>(true);
        gripper_state_msg_.error = static_cast<uint8_t>(false);
        gripper_state_msg_.calibrated = static_cast<uint8_t>(true);
        gripper_state_msg_.ready = static_cast<uint8_t>(true);

        gripper_state_pub_.publish(gripper_state_msg_);
    }

    /*this is the call back for when the actions server wants to execute a new command (open/close)*/
    void gripper_action_cb(const crustcrawler_core_msgs::EndEffectorCommand::ConstPtr& gripper_command){
        if(gripper_command->args[13] == '1'){
            ROS_ERROR_STREAM("I am suppose to open the gripper, cause the command is: " << gripper_command->args);
            open_gripper();
        }
        if(gripper_command->args[13] == '0'){
            ROS_ERROR_STREAM("I am suppose to close the gripper, cause the command is: " << gripper_command->args);
            close_gripper();
        }
    }

    void open_gripper(){
        /*Simulation*/
        if(simulation_){
            left_cmd_.data = -0.8;
            right_cmd_.data = 0.8;
        }
        /*real robot*/
        else{
            left_cmd_.data = -0.5;
            right_cmd_.data = 0.5;
        }
        right_finger_command_pub_.publish(right_cmd_);
        left_finger_command_pub_.publish(left_cmd_);
    }

    void close_gripper(){
        /*Simulation*/
        if(simulation_){
            left_cmd_.data = 0.0;
            right_cmd_.data = 0.0;
        }
        /*real robot*/
        else{
            left_cmd_.data = 0.0;
            right_cmd_.data = 0.0;
        }
        right_finger_command_pub_.publish(right_cmd_);
        left_finger_command_pub_.publish(left_cmd_);

    }

protected:
    ros::NodeHandle nh_;
    ros::Subscriber joint_state_sub_, gripper_action_sub_;
    ros::Publisher gripper_state_pub_, gripper_command_pub_, right_finger_command_pub_, left_finger_command_pub_;
    crustcrawler_core_msgs::EndEffectorState gripper_state_msg_;
    double left_finger_position_, right_finger_position_;
    std::vector<double> fingers_positions_;
    std::vector<std::string> gripper_joints_names_;
    std_msgs::Float64 left_cmd_, right_cmd_;
    bool simulation_ = true;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "crustcrawler_gripper");
    ROS_WARN("------------- initialized crustcrawler gripper handler node ---------------------");
    Crustcrawler_Gripper crustcrawler_translator(ros::this_node::getName());

    ros::spin();

    return 0;
}
