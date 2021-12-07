#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <queue>
#include <math.h>
#include <limits>

class AStar {
    private:
        geometry_msgs::Point start;
        geometry_msgs::Point goal;
        std::vector<geometry_msgs::Point> vertex_list;
        std::vector<std::vector<int>> connectivity;
        std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>,
                             std::greater<std::pair<double, int>>> pq;
        std::vector<int> previous;
        std::vector<geometry_msgs::Point> final_path;
        std::vector<double> distance;

    public:
        AStar(geometry_msgs::Point start, geometry_msgs::Point goal, 
                std::vector<geometry_msgs::Point> vertex_list, std::vector<std::vector<int>> connectivity)

                : start{start}, goal{goal}, vertex_list{vertex_list}, connectivity{connectivity},
                     previous(vertex_list.size()), distance(vertex_list.size(), std::numeric_limits<int>::max()) 
                     {
                }
        
        void show_connectivity(){
            for (int i = 0; i < this->connectivity.size(); i++){
                for (int j = 0; j < this->connectivity[i].size(); j++){
                    std::cout << this->connectivity[i][j] << "  ";
                }
                std::cout << std::endl;
            }
        }
        
        double euc_dist(geometry_msgs::Point from, geometry_msgs::Point to) {
            double x_diff = from.x - to.x;
            double y_diff = from.y - to.y;
            return (sqrt(pow(x_diff, 2) + pow(y_diff, 2)));
        }

        std::vector<geometry_msgs::Point> get_final_path() {
             return this->final_path;
        }

        void trace_path(){
            int goal_index = this->vertex_list.size() - 1;
            int start_index = this->vertex_list.size() - 2;
            this->final_path.push_back(this->vertex_list[goal_index]);
            while(goal_index != start_index){
                goal_index = previous[goal_index];
                this->final_path.push_back(this->vertex_list[goal_index]);
            }
            this->final_path.push_back(this->vertex_list[goal_index]);
            reverse(this->final_path.begin(), this->final_path.end());
        }

        void calculate_path() {
            geometry_msgs::Point start_node = this->start;
            geometry_msgs::Point goal_node = this->goal;
            int n = this->vertex_list.size();
            double heuristic = this->euc_dist(start_node, goal_node);
            distance[n - 2] = 0 + heuristic;
            int start_node_index = n - 2;
            this->pq.push(std::make_pair(heuristic, start_node_index));
            while(!this->pq.empty()){
                // ROS_INFO(pq.size());
                // std::cout << pq.size() << std::endl;
                std::pair<double, int> current = this->pq.top();
                this->pq.pop();
                int i = current.second;
                geometry_msgs::Point u = this->vertex_list[i];
                std::cout << this->connectivity[i].size();
                for (int j = 0; j < this->connectivity[i].size(); j++){
                    int k = this->connectivity[i][j];
                    geometry_msgs::Point v = this->vertex_list[k];
                    if (this->distance[k] > this->distance[i] + this->euc_dist(u, v)) {
                        this->distance[k] = this->distance[i] + this->euc_dist(u, v);
                        this->previous[k] = i;
                        this->pq.push(std::make_pair(this->distance[k] + this->euc_dist(v, this->goal), k));
                    }
                }
                if (u == this->goal) {
                    ROS_INFO("Path found. Tracing Path...");
                    this->trace_path();
                    return;
                }
            }
            if (this->pq.empty()){
                ROS_INFO("Path not found.");
            }
        }


};