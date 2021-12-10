# include <iostream>
# include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <math.h>
#include "Robot/robot.hpp"
#include <queue>

typedef std::pair<geometry_msgs::Point, double> robot_state;
typedef std::priority_queue<std::pair<double, robot_state>, std::vector<std::pair<double, robot_state>>, std::greater<std::pair<double, robot_state>> pq_type;

class local_planner
{
private:
    std::vector<geometry_msgs::Point> global_path;
    std::vector<geometry_msgs::Point> vert_list;
    std::vector<std::vector<geometry_msgs::Point>> hull_verts
    double threshold;
    double robot_length;
    Robot robot;
     // Ackerman drive
    std::vector<int> U_s;
    std::vector<int> Theta;
    double delta_t;
    geometry_msgs::Point from;
    geometry_msgs::Point to;


public:
    local_planner(std::vector<geometry_msgs::Point> global_path,  std::vector<geometry_msgs::Point> vert_list , double threshold = 10 , double robot_length = 2, std::vector<std::vector<geometry_msgs::Point>> hull_verts)
                : global_path{global_path}, vert_list{vert_list}, threshold{threshold}, robot_length{robot_length}, hull_verts{hull_verts}
                {
                    double radius = 0.5                         // radius added randomly. Need to add correct. 
                    this->robot(this->global_path[0], 0, radius);
                    std::vector<int> this->U_s = {-1, 1};
                    std::vector<int> this->Theta = {-15, 0, 15}; 
                    double this->delta_t = 1;
                    robot->center = this->global_path[0];
                    robot->orientation = 0;
                }

    double degree_to_rad(double angle_degree)
    {
        return (angle_degree*M_PI/180);
    }

    double euc_dist(geometry_msgs::Point from, geometry_msgs::Point to) {
            double x_diff = from.x - to.x;
            double y_diff = from.y - to.y;
            return (sqrt(pow(x_diff, 2) + pow(y_diff, 2)));
        }

    double calc_heuristic(robot_state possible_state) {

    }

    pq_type get_possible_states(robot_state current_state) {

        pq_type possible_states;
        double x_curr = current_state.first.x;
        double y_curr = current_state.first.y;
        double theta_curr = current_state.second;

         for(auto u_s : this->U_s)
        {
            for(auto theta : this->Theta)
            {
                double theta_next =  theta_curr + (u_s*tan(this->degree_to_rad(theta))/this->robot_length)* this->delta_t ;
                double x_next = x_curr + (us*cos(theta_next))*this->delta_t;
                double y_next = y_curr + (us*sin(theta_next))*this->delta_t;
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
                    robot_state possible_state = std::make_pair(point, theta_next);
                    double heuristic = this->calc_heuristic(possible_state);
                    possible_states.push(heuristic, possible_state);   // add heuristic function
                }
                
            }
        }

        return possible_states;

     }

    std::vector<robot_state> get_local_path(geometry_msgs::Point from, geometry_msgs::Point to)
    {   
        this->from = from;
        this->to = to;
        double theta_curr = robot->orientation;             ///////////////////////// Need to add the current theta of the robot.
        robot_state current_state = std::make_pair(this->from, theta_curr);
        std::vector<robot_state> path_points;
        path_points.push_back(current_state);
        while (this->euc_dist(current_state.first, this->to) > 10) {
            pq_type possible_states = this->get_possible_states(current_state);
            std::pair<double, robot_state> heuristic_state_pair = possible_states.top();
            robot_state next_best_state = heuristic_state_pair.second;
            path_points.push_back(next_best_state);
            current_state = next_best_state;
        }
        this->to = current_state.first;
        return path_points;

    }

    std::vector<robot_state> get_path(){
        std::vector<std::vector<robot_state>> local_paths;
        for (int i = 0; i < this->global_path.size() - 1; i++) {
            std::vector<robot_state> local_path = get_local_path(this->global_path[i], this->global_path[i+1]);
            local_paths.insert(local_paths.end(), local_path.begin(), local_path.end());
            this->global_path[i+1] = this->to;
        }
    }

    
};

