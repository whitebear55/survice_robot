#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>

#define WALL_PERCEPTION 0.3
#define LINEAR_VEL 0.05
#define ANGULAR_VEL 0.2
#define FRONT_NUM 0
#define LEFT_NUM 59
#define RIGHT_NUM 176
#define BACK_NUM 118
#define RIGHT_NUM_1 166
#define RIGHT_NUM_2 186
#define LEFT_NUM_1 69
#define LEFT_NUM_2 49
#define SLEEP 5

#define TARGET_COLOR_1 "red"
#define TARGET_COLOR_2 "blue"
#define TARGET_DISTANCE 36
#define TARGET_XPOS 0
#define TARGET_YPOS 1
#define TARGET_RADIUS 2
#define CENTER_XPOS 160

class Obstacle {
public:
    Obstacle() {
        _cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        _mani_pub = nh.advertise<std_msgs::String>("mani_pub", 1);
        _target_pub = nh.advertise<std_msgs::String>("target_pub", 1);

        obstacle();
    }

    void obstacle() {
        bool wall_parallel = true;
        bool target_parallel_1 = false;
        bool target_parallel_2 = false;
        bool target_closest = false;
        bool target_find = false;
        bool finish_flag = false;

        std::string path_method = "left_hand";
        std::vector<double> distances;
        std::vector<double> target_info;
        std::string finish;
        std::string figure;

        while(ros::ok()){
            distances.insert(distances.end(), get_scan().begin(), get_scan().end());
            target_info.insert(target_info.end(), get_target_info().begin(), get_target_info().end());

            if (target_find)
            {
                if (target_parallel_1)
                {
                    if (target_closest)
                    {
                        if (target_parallel_2)
                        {
                            if (finish_flag)
                            {
                                if (finish == "right_put_finish") {
                                    pub_cmd(0.0, 0.0);
                                    pub_target("object");
                                    finish_flag = false;
                                    target_parallel_1 = false;
                                    target_parallel_2 = false;
                                    target_find = false;
                                    finish = "";
                                    figure = "";
                                    path_method = "left_hand";
                                } 
                                else if (finish == "left_put_finish") {
                                    pub_cmd(0.0, 0.0);
                                    pub_target("object");
                                    finish_flag = false;
                                    target_parallel_1 = false;
                                    target_parallel_2 = false;
                                    target_find = false;
                                    finish = "";
                                    figure = "";
                                    path_method = "right_hand";
                                } 
                                else if (finish == "back_put_finish") {
                                    pub_cmd(0.0, 0.0);
                                    pub_target("object");
                                    finish_flag = false;
                                    target_parallel_1 = false;
                                    target_parallel_2 = false;
                                    target_find = false;
                                    finish = "";
                                    figure = "";
                                    path_method = "left_hand";
                                }
                                else if (finish == "right_hit_finish") {
                                    pub_cmd(0.0, 0.0);
                                    pub_target("object");
                                    finish_flag = false;
                                    target_parallel_1 = false;
                                    target_parallel_2 = false;
                                    target_find = false;
                                    finish = "";
                                    figure = "";
                                    path_method = "left_hand";
                                } 
                                else if (finish == "left_hit_finish") {
                                    pub_cmd(0.0, 0.0);
                                    pub_target("object");
                                    finish_flag = false;
                                    target_parallel_1 = false;
                                    target_parallel_2 = false;
                                    target_find = false;
                                    finish = "";
                                    figure = "";
                                    path_method = "right_hand";
                                } 
                                else if (finish == "parking_finish") {
                                    pub_cmd(0.0, 0.0);
                                    ROS_INFO("Project Finish!");
                                } 
                                else {
                                    finish_flag = false;
                                }
                            }
                            else
                            {
                                if (figure == "redcircle")
                                {
                                    pub_cmd(0.0, 0.0);
                                    pub_mani("right_put");
                                    ROS_INFO("\nThis is redcircle");
                                    finish = get_finish();
                                    finish_flag = true;
                                }
                                else if (figure == "bluecircle")
                                {
                                    pub_cmd(0.0, 0.0);
                                    pub_mani("left_put");
                                    ROS_INFO("\nThis is bluecircle");
                                    finish = get_finish();
                                    finish_flag = true;
                                }
                                else if (figure == "greencircle")
                                {
                                    pub_cmd(0.0, 0.0);
                                    pub_mani("back_put");
                                    ROS_INFO("\nThis is greencircle");
                                    finish = get_finish();
                                    finish_flag = true;
                                }
                                else if (figure == "redrectangle")
                                {
                                    pub_cmd(0.0, 0.0);
                                    pub_mani("right_hit");
                                    ROS_INFO("\nThis is redrectangle");
                                    finish = get_finish();
                                    finish_flag = true;
                                }
                                else if (figure == "bluerectangle")
                                {
                                    pub_cmd(0.0, 0.0);
                                    pub_mani("left_hit");
                                    ROS_INFO("\nThis is bluerectangle");
                                    finish = get_finish();
                                    finish_flag = true;
                                }
                                else if (figure == "greenrectangle") {
                                    pub_cmd(0.0, 0.0);
                                    pub_mani("mid_hit");
                                    ROS_INFO("\nThis is greenrectangle");
                                    finish = get_finish();
                                    pub_cmd(LINEAR_VEL, 0.0);
                                    sleep(SLEEP);
                                    pub_mani("parking");
                                    finish = get_finish();
                                    finish_flag = true;
                                }

                            }
                        }
                        else
                        {
                            if (target_info[TARGET_XPOS] >= CENTER_XPOS - 2 && target_info[TARGET_XPOS] <= CENTER_XPOS + 2)
                            {
                                pub_cmd(0.0, 0.0);
                                pub_target("What Figure");
                                figure = get_figure();
                                target_parallel_2 = true;
                            }
                            else if (target_info[TARGET_XPOS] == 0)
                            {
                                pub_cmd(0.0, 0.0);
                            }
                            else if (target_info[TARGET_XPOS] < CENTER_XPOS)
                            {
                                pub_cmd(0.0, ANGULAR_VEL * 0.1);
                            }
                            else if (target_info[TARGET_XPOS] > CENTER_XPOS)
                            {
                                pub_cmd(0.0, -ANGULAR_VEL * 0.1);
                            }
                        }
                    }
                    else
                    {
                        if (round(target_info[TARGET_RADIUS]) >= TARGET_DISTANCE - 1 && round(target_info[TARGET_RADIUS]) <= TARGET_DISTANCE + 1)
                        {
                            pub_cmd(0.0, 0.0);
                            target_closest = true;
                        }
                        else if (target_info[TARGET_RADIUS] == 0)
                        {
                            pub_cmd(0.0, 0.0);
                        }
                        else if (target_info[TARGET_RADIUS] < TARGET_DISTANCE)
                        {
                            pub_cmd(LINEAR_VEL * 0.1, 0.0);
                        }
                        else if (target_info[TARGET_RADIUS] > TARGET_DISTANCE)
                        {
                            pub_cmd(-LINEAR_VEL * 0.1, 0.0);
                        }
                    }
                }
                else
                {
                    if (target_info[TARGET_XPOS] >= CENTER_XPOS - 2 && target_info[TARGET_XPOS] <= CENTER_XPOS + 2)
                    {
                        pub_cmd(0.0, 0.0);
                        target_parallel_1 = true;
                    }
                    else if (target_info[TARGET_XPOS] == 0)
                    {
                        pub_cmd(0.0, 0.0);
                    }
                    else if (target_info[TARGET_XPOS] < CENTER_XPOS)
                    {
                        pub_cmd(0.0, ANGULAR_VEL * 0.1);
                    }
                    else if (target_info[TARGET_XPOS] > CENTER_XPOS)
                    {
                        pub_cmd(0.0, -ANGULAR_VEL * 0.1);
                    }
                }
            }
            else
            {
                if (target_info[TARGET_RADIUS] > 20.0)
                {
                    pub_cmd(0.0, 0.0);
                    target_find = true;
                }
                else
                {
                    if (wall_parallel && path_method == "left_hand")
                    {
                        if (distances[LEFT_NUM_2] > WALL_PERCEPTION + 0.5)
                        {
                            pub_cmd(LINEAR_VEL, (distances[LEFT_NUM_2] - distances[LEFT_NUM_1]) * 0.2);
                        }
                        else if (distances[FRONT_NUM] > WALL_PERCEPTION)
                        {
                            pub_cmd(LINEAR_VEL, (distances[LEFT_NUM_2] - distances[LEFT_NUM_1]) * 2.0 + (distances[LEFT_NUM] - WALL_PERCEPTION) * 0.5);
                        }
                        else
                        {
                            pub_cmd(0.0, -ANGULAR_VEL);
                            sleep(SLEEP);
                            wall_parallel = false;
                        }
                    }
                    else if (wall_parallel && path_method == "right_hand")
                    {
                        if (distances[RIGHT_NUM_2] > WALL_PERCEPTION + 0.5)
                        {
                            pub_cmd(LINEAR_VEL, (distances[RIGHT_NUM_1] - distances[RIGHT_NUM_2]) * 0.2);
                        }
                        else if (distances[FRONT_NUM] > WALL_PERCEPTION)
                        {
                            pub_cmd(LINEAR_VEL, (distances[RIGHT_NUM_1] - distances[RIGHT_NUM_2]) * 2.0 + (WALL_PERCEPTION - distances[RIGHT_NUM]) * 0.5);
                        }
                        else
                        {
                            pub_cmd(0.0, ANGULAR_VEL);
                            sleep(SLEEP);
                            wall_parallel = false;
                        }
                    }
                    else
                    {
                        if (path_method == "left_hand")
                        {
                            if (distances[LEFT_NUM_1] <= distances[LEFT_NUM_2])
                            {
                                pub_cmd(0.0, 0.0);
                                wall_parallel = true;
                            }
                            else
                            {
                                pub_cmd(0.0, -ANGULAR_VEL * 0.5);
                            }
                        }
                        else if (path_method == "right_hand")
                        {
                            if (distances[RIGHT_NUM_1] <= distances[RIGHT_NUM_2])
                            {
                                pub_cmd(0.0, 0.0);
                                wall_parallel = true;
                            }
                            else
                            {
                                pub_cmd(0.0, ANGULAR_VEL * 0.5);
                            }
                        }
                    }
                }
            }
            distances.clear();
            target_info.clear();
        }   
    }

    std::vector<double> get_scan() {
        sensor_msgs::LaserScanConstPtr scan = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan");
        std::vector<double> scan_filter;
        scan_filter.reserve(scan->ranges.size());
        scan_filter.insert(scan_filter.end(), scan->ranges.begin(), scan->ranges.end());

        for (int i = 0; i < scan_filter.size(); ++i) {
            if (std::isinf(scan_filter[i])) {
                scan_filter[i] = 3.5;
            } else if (std::isnan(scan_filter[i])) {
                scan_filter[i] = 0;
            }
        }

        return scan_filter;
    }

    std::string get_finish() {
        std_msgs::StringConstPtr finish_sub = ros::topic::waitForMessage<std_msgs::String>("finish_pub");
        std::string finish = finish_sub->data;

        return finish;
    }

    std::string get_figure() {
        std_msgs::StringConstPtr figure_sub = ros::topic::waitForMessage<std_msgs::String>("figure_pub");
        std::string figure = figure_sub->data;

        return figure;
    }

    std::vector<double> get_target_info() {
        std::vector<double> detect_target_info;
        
        std_msgs::Int64ConstPtr target_xpos = ros::topic::waitForMessage<std_msgs::Int64>("center_xpos");
        std_msgs::Int64ConstPtr target_ypos = ros::topic::waitForMessage<std_msgs::Int64>("center_ypos");
        std_msgs::Float64ConstPtr target_radius = ros::topic::waitForMessage<std_msgs::Float64>("center_radius");

        detect_target_info.push_back(target_xpos->data);
        detect_target_info.push_back(target_ypos->data);
        detect_target_info.push_back(target_radius->data);

        return detect_target_info;
    }

    void pub_cmd(double linear, double angular) {
        geometry_msgs::Twist twist;
        twist.linear.x = linear;
        twist.angular.z = angular;
        _cmd_pub.publish(twist);
    }

    void pub_target(const std::string& target_comm) {
        std_msgs::String string;
        string.data = target_comm;
        _target_pub.publish(string);
    }

    void pub_mani(const std::string& mani_comm) {
        std_msgs::String string;
        string.data = mani_comm;
        _mani_pub.publish(string);
    }
private:
    ros::NodeHandle nh;
    ros::Publisher _cmd_pub;
    ros::Publisher _mani_pub;
    ros::Publisher _target_pub;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "project_cpp");

    Obstacle obstacle;

    ros::spin();

    return 0;
}