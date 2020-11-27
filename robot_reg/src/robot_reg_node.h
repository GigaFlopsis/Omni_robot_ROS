#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/Marker.h>
#include <robot_reg/RobotRegConfig.h>
#include <std_msgs/Float32.h>

/*----Global params----*/

bool calc_vel_from_pos = true;

/*----Глобальные переменные----*/

// Коэффициенты регулятора
double_t hor_kp = 1.5;
double_t hor_kd = 0.3;
double_t max_hor_vel = 0.5;
double_t ver_kp = 1.5;
double_t ver_kd = 1.0;
double_t max_ver_vel = 1.0;
double_t angular_p = 2.5;
double_t angular_d = 0.1;
double_t prev_error = 0.0;

int ctr_type;                                       // по умолчанию используем управление по координатам
geometry_msgs::Twist current_vel;            // Текущая скорость дрона
geometry_msgs::Twist vel_field;

// topics
std::string goal_local_topic = "/goal_pose";             // напрямую
std::string goal_posestamped_local_topic = "/goal";             // напрямую

std::string map_frame = "odom";
std::string child_frame = "base_link";


// yaml
YAML::Node yaml_node;
std::string yaml_path = "~/robot_reg_params.yaml";

// таймеры
double goal_timer = 0.0;
double pose_timer = 0.0;
double goal_lost_time = 1.0;
double pose_lost_time = 0.5;
double print_delay = 3.0;
double print_timer = 0.;
const int queue_size = 10;

bool init_server = false;

/*----Функции подписки на топики----*/

void cfg_callback(robot_reg::RobotRegConfig &config, uint32_t level);

void goal_posestamped_cb(const geometry_msgs::PoseStamped &data);

void vel_field_cb(const geometry_msgs::TwistStampedConstPtr &data);


/*----Вспомогательные функции----*/

double_t angle_between(std::vector<double_t> vec1, std::vector<double_t> vec2);

std::vector<double_t> limit_vector(std::vector<double_t> r, double_t max_val);

std::vector<double_t> get_linear_vel_vec(std::vector<double_t> r, std::vector<double_t> vel);

double_t get_angular_vel(double_t ang, double_t vel, double_t k, double_t d);

visualization_msgs::Marker setup_marker(drone_msgs::DronePose point);

void set_server_value();

/*----Python function replacements----*/

double_t norm_d(std::vector<double_t> r);

double_t degToRad(double_t rad);

/*-------------------------------------*/
