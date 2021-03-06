#include <ros/ros.h>
#include <boost/timer.hpp>
#include <Eigen/Core>

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
        joint_state_sub_ = nh_.subscribe("/crustcrawler/joint_states", 1, &Crustcrawler_JointCommand_Translator::joint_states_cb, this);

        /*Those are the publisher which will be used to translate each command received from the joint action server to all joints*/
        pub_j_1_ = nh_.advertise<std_msgs::Float64>("/crustcrawler/joint_1_position_controller/command", 1, this);
        pub_j_2_ = nh_.advertise<std_msgs::Float64>("/crustcrawler/joint_2_position_controller/command", 1, this);
        pub_j_3_ = nh_.advertise<std_msgs::Float64>("/crustcrawler/joint_3_position_controller/command", 1, this);
        pub_j_4_ = nh_.advertise<std_msgs::Float64>("/crustcrawler/joint_4_position_controller/command", 1, this);
        pub_j_5_ = nh_.advertise<std_msgs::Float64>("/crustcrawler/joint_5_position_controller/command", 1, this);
        pub_j_6_ = nh_.advertise<std_msgs::Float64>("/crustcrawler/joint_6_position_controller/command", 1, this);

        /*time step size*/
        double rate;
        nh_.getParam("/time_step", dt_);
        nh_.getParam("/ros_rate", rate);
        ros_rate_.reset(new ros::Rate(rate));
    }

    ~Crustcrawler_JointCommand_Translator(void)
    {

    }

    void arranged_joints_values(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& new_goal){
        if(!joint_state_.position.empty()){
            arranged_joint_states_.clear();
            for(size_t i = 0; i < new_goal->goal.trajectory.joint_names.size(); i++){
                arranged_joint_states_.push_back(joint_state_.position[distance(joint_state_.name.begin(),
                                                                                find(joint_state_.name.begin(),
                                                                                     joint_state_.name.end(),
                                                                                     new_goal->goal.trajectory.joint_names[i]))]);
            }
        }
    }

    //get largest difference between elements of two vectors
    double largest_difference(std::vector<double> &first, std::vector<double> &second){
        Eigen::VectorXd difference(first.size());
        double my_max = 0;
        for(size_t j = 0; j < first.size(); ++j)
            difference(j) = fabs(first[j] - second[j]);
        for(size_t j = 0; j < first.size(); ++j){
            if(difference(j) > my_max)
                my_max = difference(j);
        }
        return my_max;
    }

    bool is_bigger(std::vector<double>& vector, double value, int& index){
        bool output = false;
        for(size_t i = 0; i < vector.size(); i++){
            if(fabs(vector[i]) > value){
                index = i;
                output = true;
            }
        }
        return output;
    }

    void joint_states_cb(const sensor_msgs::JointState::ConstPtr& joint_states){
        joint_state_ = *joint_states;
        joints_loads_ = joint_states->effort;
        int index;
        if(is_bigger(joints_loads_, 0.9, index)){
            stressed_ = true;
            ROS_WARN_STREAM("the joint under stress is: " << joint_states->name[index] << " with value of: " << joints_loads_[index]);
        }
        else
            stressed_ = false;
    }

    void joint_trajectory_new_goal_cb(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& new_goal){
        double start_time = ros::Time::now().toSec();
        the_timer_.restart();
        if(!stressed_){
            for(size_t i = 0; i < new_goal->goal.trajectory.points.size(); i++){
                double time_to_wait = new_goal->goal.trajectory.points[i].time_from_start.toSec();
                //ROS_ERROR_STREAM("time for this point is: " << time_to_wait);

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
                //target_joint_state_ = new_goal->goal.trajectory.points[i].positions;
                //arranged_joints_values(new_goal);
                /*Then publish them*/
                //while(largest_difference(target_joint_state_, arranged_joint_states_) > 0.03){
                //  arranged_joints_values(new_goal);
                pub_j_1_.publish(j_1_);
                pub_j_2_.publish(j_2_);
                pub_j_3_.publish(j_3_);
                pub_j_4_.publish(j_4_);
                pub_j_5_.publish(j_5_);
                pub_j_6_.publish(j_6_);

                /*}
            //while((ros::Time::now().toSec() - start_time) <= time_to_wait);
            //double time_to_sleep = ros::Time::now().toSec() - start_time;
            ROS_WARN_STREAM("I am publishing joints commands now !!!!!!!!! for point: " << i
                            << " which has time from start = " << time_to_wait
                            << " and the boost timer elapsed is: " << the_timer_.elapsed()
                            << "and time to sleep is: " << time_to_sleep);*/

                //ROS_WARN_STREAM("I will be waiting for: " << dt_*1e4);
                //usleep(dt_*1e4);
                ros_rate_->sleep();
            }
        }
        else{
            j_2_.data = 0.0;
            pub_j_2_.publish(j_2_);
            stressed_ = false;
            ROS_WARN("I am stressed and trying to releave myself !!!!!!!");
            ros_rate_->sleep();
        }
    }


protected:
    ros::NodeHandle nh_;
    ros::Subscriber joint_command_sub_, joint_action_server_sub_, joint_state_sub_;
    ros::Publisher pub_j_1_, pub_j_2_, pub_j_3_, pub_j_4_, pub_j_5_, pub_j_6_;
    std_msgs::Float64 j_1_, j_2_, j_3_, j_4_, j_5_, j_6_;
    sensor_msgs::JointState joint_state_;
    boost::timer the_timer_;
    std::vector<double> target_joint_state_, arranged_joint_states_, joints_loads_;
    std::vector<std::string> joint_names_;
    double dt_;
    std::shared_ptr<ros::Rate> ros_rate_;
    bool stressed_ = false;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "crustcrawler_jointcommand_translator");
    ROS_WARN("------------- initialized crustcrawler joint command translator ---------------------");
    Crustcrawler_JointCommand_Translator crustcrawler_translator(ros::this_node::getName());

    ros::spin();

    return 0;
}
