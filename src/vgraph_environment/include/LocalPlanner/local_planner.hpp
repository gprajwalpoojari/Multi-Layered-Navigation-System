# include <iostream>
# include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <math.h>
#include "Robot/robot.hpp"
#include <queue>
#include <limits>

typedef std::pair<geometry_msgs::Point, double> robot_state;
typedef std::pair<double, robot_state> pq_data_type;
class myComparator
{
public:
    int operator() (const pq_data_type& d1, pq_data_type& d2)
    {
        return d1.first > d2.first;
    }
};
typedef std::priority_queue<pq_data_type, std::vector<pq_data_type>, myComparator> pq_type;


double get_perp_distance(geometry_msgs::Point robot_center, geometry_msgs::Point vertice_1, geometry_msgs::Point vertice_2)    
{ 
    geometry_msgs::Point vec_vertices;
    vec_vertices.x = vertice_1.x - vertice_2.x;
    vec_vertices.y = vertice_1.y - vertice_2.y;
    vec_vertices.z = vertice_1.z - vertice_2.z;
    double dist_bet_vertices = (sqrt(pow(vec_vertices.x, 2) + pow(vec_vertices.y, 2) + pow(vec_vertices.z, 2)));
    
    geometry_msgs::Point rob_cen_vert;
    rob_cen_vert.x = vertice_1.x - robot_center.x;
    rob_cen_vert.y = vertice_1.y - robot_center.y;
    rob_cen_vert.z = vertice_1.z - robot_center.z;
    double dist_cen_vert = (sqrt(pow(rob_cen_vert.x, 2) + pow(rob_cen_vert.y, 2) + pow(rob_cen_vert.z, 2)));

    double dot_product = (vec_vertices.x*rob_cen_vert.x) +  (vec_vertices.y*rob_cen_vert.y) + (vec_vertices.z*rob_cen_vert.z);
    double cos_theta = dot_product/(dist_bet_vertices*dist_cen_vert);
    double theta = acos(cos_theta);

    double perp_dist = dist_cen_vert*sin(theta);
    return perp_dist;
}

class Local_Planner
{
private:
    std::vector<geometry_msgs::Point> global_path;
    // std::vector<geometry_msgs::Point> vert_list;
    std::vector<std::vector<geometry_msgs::Point>> hull_verts;
    // double threshold;
    double robot_length;
    Robot robot;
     // Ackerman drive
    std::vector<double> U_s;
    std::vector<double> Theta;
    double delta_t;
    geometry_msgs::Point from;
    geometry_msgs::Point to;

    // std::vector<double> U_l;
    // std::vector<double> U_r; 
    // double rob_wheel_rad;


public:
    Local_Planner(std::vector<geometry_msgs::Point> global_path,  /*std::vector<geometry_msgs::Point> vert_list,*/ std::vector<std::vector<geometry_msgs::Point>> hull_verts, /*double threshold = 10 ,*/ double robot_length = 0.36)
                : global_path{global_path}, /*vert_list{vert_list}, threshold{threshold},*/  hull_verts{hull_verts}, robot_length{robot_length}, robot(global_path[0], 0, robot_length/2)
                {
                    this->U_s = {-0.1, 0.1};
                    this->Theta = {-45, 0, 45}; 
                    this->delta_t = 1;    

                    // this->rob_wheel_rad = 0.033;   // 33mm
                    // this->U_l = {-0.1, 0, 0.1};
                    // this->U_r = {-0.1, 0, 0.1};

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


    double get_shortest_dist(std::vector<geometry_msgs::Point> hull_vert)
    {   
        double shortest_dist = std::numeric_limits<double>::max(); 
        for (int i=0; i < hull_vert.size(); i++)
        {   
            double dist;
            if (i == hull_vert.size()-1)
            {
                dist = get_perp_distance(this->robot.get_center(), hull_vert[i],hull_vert[0]);
            }
            dist = get_perp_distance(this->robot.get_center(), hull_vert[i],hull_vert[i+1]);
            // std::cout << dist << " "<< std::endl;
            if (dist < shortest_dist)
            {
                shortest_dist = dist;
            }
        }
        // std::cout << shortest_dist << " "<< std::endl;
        return shortest_dist;
    }

    double get_attractive_potential()
    {
        double threshold = 5;
        double constant = 1;
        double U_att;
        double dist = this->euc_dist(this->robot.get_center(), this->to); 
        if (dist > threshold)
        {
            U_att = threshold*constant*dist - 0.5*constant*pow(threshold,2);
        }   
        else
        {
            U_att = 0.5*constant*pow(dist,2);
        }
        return U_att;
    }

    double get_repulsive_potential()
    {
        double threshold = 2*this->robot.get_radius();
        double constant = 1/2;
        double U_rep = 0;
        for (int i=0; i< this->hull_verts.size(); i++)
        {
            double dist = this->get_shortest_dist(this->hull_verts[i]);
            // std::cout << 1/dist << " " << threshold << std::endl;
            if (dist < threshold)
            {
                U_rep += 0.5*constant*pow((1/dist - 1/threshold),2);
            }
        }
        return U_rep; 
    }

    double calc_heuristic()  // robot_state possible_state (arg) 
    {
        double U_final;
        double U_att = this->get_attractive_potential();
        double U_rep = this->get_repulsive_potential();
        // std::cout<<"a "<<U_att<<" r "<<U_rep<<std::endl;
        U_final = U_att + U_rep;
        return U_final;
    }


    pq_type get_possible_states(robot_state current_state) {

        pq_type possible_states;
        double x_curr = current_state.first.x;
        double y_curr = current_state.first.y;
        double theta_curr = current_state.second;
        // std::cout << x_curr << " " << y_curr << std::endl;


         for(auto u_s : this->U_s)
        {
            for(auto theta : this->Theta)
            {
                double theta_next =  theta_curr + (u_s*tan(this->degree_to_rad(theta))/this->robot_length)* this->delta_t ;
                double x_next = x_curr + (u_s*cos(theta_next))*this->delta_t;
                double y_next = y_curr + (u_s*sin(theta_next))*this->delta_t;
                 
                // double theta_next = theta_curr + (u_l - u_r)*(this->rob_wheel_rad/this->robot_length)*this->delta_t;
                // double x_next = x_curr + (this->rob_wheel_rad/2)*(u_l+u_r)*cos(theta_next)*this->delta_t;
                // double y_next = y_curr + (this->rob_wheel_rad/2)*(u_l+u_r)*sin(theta_next)*this->delta_t;


                // std::cout<<"x "<<x_next<< " y "<<y_next<<std::endl;
                geometry_msgs::Point point;
                point.x = x_next;
                point.y = y_next;
                point.z = 0;
                this->robot.update_robot_pose(point, theta_next);
                bool status = false;
                // int count = 0;
                for (auto obstacle : hull_verts)
                {   
                    // std::cout << count << std::endl;
                    status = this->robot.check_collision(obstacle);
                    // std::cout<<status<<std::endl;
                    if (status) {
                        std::cout << "Collision!"<<std::endl;
                        std::cout << this->robot.get_center() << std::endl;
                        // std::cout << robot.get_center()<<std::endl;
                        // std::vector<geometry_msgs::Point> corners = robot.get_corners();
                        // for (auto i : corners){
                        //     std::cout <<i.x << " " << i.y <<std::endl;
                        // }
                        break;
                    }
                    // count++;
                }
                if (status == false)
                {
                    robot_state possible_state = std::make_pair(point, theta_next);
                    this->robot.update_robot_pose(point, theta_next);
                    // std::vector<geometry_msgs::Point> corners = this->robot.get_corners();
                    //     for (auto i : corners){
                    //         std::cout <<i.x << " " << i.y <<std::endl;
                    //     }
                    // std::cout<< "Next" << std::endl;
                    
                    double heuristic = this->calc_heuristic();    // possible_state (arg)
                    // std::cout<<"h "<< heuristic<<std::endl;
                    pq_data_type data = std::make_pair(heuristic, possible_state);
                    possible_states.push(data);   // add heuristic function
                }
                
            }
        }

        return possible_states;

     }

    std::vector<robot_state> get_local_path(geometry_msgs::Point from, geometry_msgs::Point to)
    {   
        this->from = from;
        this->to = to;
        // std::cout<<"from "<<from.x<<" "<<from.y<<std::endl;
        // std::cout<<"to "<<to.x<<" "<<to.y<<std::endl;
        double theta_curr = this->robot.get_orientation();             ///////////////////////// Need to add the current theta of the robot.
        robot_state current_state = std::make_pair(this->from, theta_curr);
        std::vector<robot_state> path_points;
        path_points.push_back(current_state);
        while (this->euc_dist(current_state.first, this->to) > 0.4) {
            // std::cout<<"c "<<current_state.first.x<<" "<<current_state.first.y<<std::endl;
            pq_type possible_states = this->get_possible_states(current_state);

            // std::cout<< possible_states.size()<<std::endl;
            pq_data_type heuristic_state_pair = possible_states.top();
            // std::cout << heuristic_state_pair.second.first.x << " " << heuristic_state_pair.second.first.y<< std::endl;
            robot_state next_best_state = heuristic_state_pair.second;
            path_points.push_back(next_best_state);
            current_state = next_best_state;
            // while (!possible_states.empty()) {
            //     pq_data_type temp = possible_states.top();
            //     possible_states.pop();
            //     std::cout<<temp.first << " " << temp.second.first.x << " "<<temp.second.first.y<< std::endl;
            // }
        }
        std::cout<<"Done" <<std::endl;
        this->to = current_state.first;
        return path_points;

    }

    std::vector<robot_state> get_path(){
        std::vector<robot_state> local_paths;
        for (int i = 0; i < this->global_path.size() - 1; i++) {
            // std::cout << this->global_path[i].x <<" "<<global_path[i].y <<std::endl;
            std::vector<robot_state> local_path = get_local_path(this->global_path[i], this->global_path[i+1]);
            // std::cout<<local_path.size()<<std::endl;
            local_paths.insert(local_paths.end(), local_path.begin(), local_path.end());
            this->global_path[i+1] = this->to;
        }
        return local_paths;
    }

    
};

