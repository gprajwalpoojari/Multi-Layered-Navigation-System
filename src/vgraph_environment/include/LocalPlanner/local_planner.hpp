# include <iostream>
# include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <math.h>
#include "Robot/robot.hpp"
#include <queue>


class local_planner
{
private:
    std::vector<geometry_msgs::Point> global_path;
    std::vector<geometry_msgs::Point> vert_list;
    std::vector<std::vector<geometry_msgs::Point>> hull_verts
    double threshold;
    double robot_length;
    Robot robot;

public:
    local_planner(std::vector<geometry_msgs::Point> global_path,  std::vector<geometry_msgs::Point> vert_list , double threshold = 10 , double robot_length = 2, std::vector<std::vector<geometry_msgs::Point>> hull_verts)
                : global_path{global_path}, vert_list{vert_list}, threshold{threshold}, robot_length{robot_length}, hull_verts{hull_verts}
                {
                    double radius = 0.5                         // radius added randomly. Need to add correct. 
                    this->robot(this->global_path[0], 0, radius);
                }

    double degree_to_rad(double angle_degree)
    {
        return (angle_degree*M_PI/180);
    }

    // heuristic (APF)

    std::vector<geometry_msgs::Point> get_path(geometry_msgs::Point from, geometry_msgs::Point to)
    {   
        double x_curr = from.x;
        double y_curr = from.y;
        double theta_curr = 0;                        // Need to add the current theta of the robot.


        double heuristic = 0; // Implement APF.
        // Ackerman drive
        std::vector<int> U_s = {-1, 1};
        std::vector<int> Theta = {-15, 0, 15}; 
        double delta_t = 1;
        // std::pair<double, std::pair<geometry_msgs::Point, double>>
        std::priority_queue<std::pair<double, std::pair<geometry_msgs::Point, double>>, std::vector<std::pair<double, std::pair<geometry_msgs::Point, double>>>,
                             std::greater<std::pair<double, std::pair<geometry_msgs::Point, double>>> possible_states;
        
        for(auto u_s : U_s)
        {
            for(auto theta : Theta)
            {
                double theta_next =  theta_curr + (u_s*tan(this->degree_to_rad(theta))/this->robot_length)* delta_t ;
                double x_next = x_curr + (us*cos(theta_next))*delta_t;
                double y_next = y_curr + (us*sin(theta_next))*delta_t;
                geometry_msgs::Point point;
                point.x = x_next;
                point.y = y_next;
                point.z = 0;
                this->robot.update_robot_pose(point, theta_next);
                bool status = false;
                for (auto obstacle : hull_verts)
                {
                    status = this->robot.check_collision(obstacle);
                    if (status) break;
                    
                }
                if (status == false)
                {
                    std::pair<geometry_msgs::Point, double> possible_state = std::make_pair(point, theta_next);
                    double heuristic = this->cal_heuristic(possible_state);
                    possible_states.push(heuristic, possible_state);   // add heuristic function
                }
                
            }
        }

    }

    
};

