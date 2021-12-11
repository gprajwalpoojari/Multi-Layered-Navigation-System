
// Creates a robot polygon to check collision between polygons.  

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <iostream>
#include <math.h>

class  Robot
{
private:
    geometry_msgs::Point center;
    double orientation;
    double radius;
    std::vector<geometry_msgs::Point> initial_points; 
    std::vector<geometry_msgs::Point> robot_final_points;

public:
    Robot(geometry_msgs::Point center, double orientation, double radius)
        : center{center}, orientation{orientation}, radius{radius}, robot_final_points(4)
        {
            geometry_msgs::Point p1, p2, p3, p4;
            p1.x = this->radius;
            p1.y = this->radius;
            p1.z = 0;

            p2.x = this->radius;
            p2.y = -this->radius;
            p2.z = 0;

            p3.x = -this->radius;
            p3.y = -this->radius;
            p3.z = 0;

            p4.x = -this->radius;
            p4.y = this->radius;
            p4.z = 0;

            this->initial_points = {p1, p2, p3, p4};
        }

    void rotate()
    {
        for(int i=0; i<this->initial_points.size(); i++)
        {
            geometry_msgs::Point point = this->initial_points[i];
            double x = cos(this->orientation)*point.x - sin(this->orientation)*point.y;
            double y = sin(this->orientation)*point.x + cos(this->orientation)*point.y; 
            geometry_msgs::Point rotated_point;
            rotated_point.x = x;
            rotated_point.y = y;
            rotated_point.z = 0;
            this->robot_final_points[i] = rotated_point;
        }
    }

    void translate()
    {
        for(auto point : this->robot_final_points)
        {
            point.x = point.x + this->center.x;
            point.y = point.y + this->center.y;
            point.z = point.x + this->center.z;
        }
    }

    geometry_msgs::Point get_center(){
        return this->center;
    }

    double get_orientation(){
        return this->orientation;
    }

    double get_radius(){
        return this->radius;
    }
    void update_robot_pose(geometry_msgs::Point center, double orientation)
    {
        this->center = center;
        this->orientation = orientation;
        this->rotate();
        this->translate();
    }

    std::vector<geometry_msgs::Point> get_robot_polygon()
    {
        return this->robot_final_points;
    }

    double cross_product(std::pair<geometry_msgs::Point, geometry_msgs::Point> line, geometry_msgs::Point point)
    {
        double x1, y1, x2, y2;
        x1 = line.second.x - line.first.x;
        y1 = line.second.y - line.first.y;
        x2 = point.x - line.first.x;
        y2 = point.y - line.first.y;
        //cross product formula
        double cross = x1 * y2 - x2 * y1;
        return cross;
    }

    bool check_collision(std::vector<geometry_msgs::Point> obstacle)
    {
        std::vector<std::vector<geometry_msgs::Point>> polygon = {obstacle, this->robot_final_points};
    
        bool COLLISION = false;
        //polygon collision algorithm
        for (int i = 0; i < polygon[0].size(); i++){
            for (int j = 0; j < polygon[1].size(); j++){
            bool probability[2] = {false}, exception[2] = {false};
            std::vector<std::pair<geometry_msgs::Point, geometry_msgs::Point>> line(2);
            int a = (i==polygon[0].size() - 1)? 0 : i;

            line[0] = std::make_pair(polygon[0][i],polygon[0][a]);
            line[1] = std::make_pair(polygon[1][i],polygon[1][a]);
            //for each line l in [l1, l2]
            for (int k = 0; k < 2; k++)
            {
                int m = (k==0)? 1: 0;
                //for each point p in the other line
                double cross[2];
                for (int l = 0; l < 2; l++)
                {
                    geometry_msgs::Point point = (l==0) ? line[m].first : line[m].second;
                    cross[l] = cross_product(line[k],point);
                }
                //If the two cross products have opposite sign, or one of them is zero
                if (cross[0] * cross[1] <= 0)
                {
                if (cross[0] != 0 || cross[1] != 0)
                {
                    probability[k] = true;
                    //store true value if only one of them is zero
                    if (cross[0] == 0 || cross[1] == 0)
                    {
                    exception[k] = true;
                    }
                    else{
                    exception[k] = false;
                    }
                }
                else
                {
                    probability[k] = false;
                }
                }
            }
            //If both cross product checks above indicated opposite signs,
            if (probability[0] && probability[1] == true){
                //check for atleast one zero cross product in both cases
                if (!(exception[0] && exception[1])){
                COLLISION = true;
                break;
                }
            }
            }
        }

    return COLLISION;
}

};

