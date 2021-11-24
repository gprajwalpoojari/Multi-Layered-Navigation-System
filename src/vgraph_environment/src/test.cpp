#include <string>

class Vgraph {
  public :
    Vgraph() {
      ros::init(int argc, char* argv, "vgraph_environment");
      ros::NodeHandle n;
      ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("vgraph_marekerarr", 10);
      ros::Publisher this->cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
      ros::Rate loop_rate(30);

      // tf::TransformListener this->tf_listener;
      // loop_rate.sleep(2);
      //
      // this->odom_frame = "/odom";
      // tf::StampedTransform transform;
      // try
      // {
      //     this->tf_listener.lookupTransform(this->odom_frame, "/base_footprint",ros::Time(0), transform);
      //     this->base_frame = '/base_footprint';
      // }
      // catch(tf::TransformException ex)
      // {
      //     try
      //     {
      //         this->tf_listener.lookupTransform(this->odom_frame, "/base_link",ros::Time(0), transform);
      //         this->base_frame = '/base_link';
      //     }
      //     catch(tf::TransformException ex)
      //     {
      //         ROS_ERROR("%s",ex.what());
      //         ros::Duration(1.0).sleep();
      //     }
      // }

      // float aabb_sidelen = 36;
      // float half = aabb_sidelen/2;


      std::string object_path = "obstacles.txt";
      float scale_factor = 100;

      std::vector<std::vector<std::pair<int, int>>> obstacles = load_obstacles(object_path);

      std::vector<std::vector<std::pair<int, int>>> grown_obstacles = grow_obstacles(obstacles);

      std::string goal_path = "goal.txt";
      std::pair<int, int> goal = load_goal(goal_path);
      std::pair<int, int> start {0,0};

      visualization_msgs::MarkerArray marker_arr;
      int marker_id = 0;

      std::vector<geometry_msgs::Point> verts;
      std::vector<std::vector<geometry_msgs::Point>> hull_verts;
      //  std::vector<std::pair<geometry_msgs::Point>> edges;
      <std::vector<std::pair<geometry_msgs::Point>> hull_edges;
      std::vector<geometry_msgs::Point> points;
      visualization_msgs::Marker m = init_marker(marker_id, visualization_msgs::Marker LINE_LIST);
      marker_id ++;

      // Draw convex hull around obstacles
      // some initialization as per .py file
      for (int i=0; i < grown_obstacles.size(), i++) {
        std::vector<geometry_msgs::Point> verts = convexHull(grown_obstacles[i]);
          for (int i = 0; i < verts.size(); i++) {
            geometry_msgs::Point p1;
            geometry_msgs::Point p2;
            p1.x = verts[i].x / scale_factor;
            p1.y = verts[i].y / scale_factor;
            p1.z = verts[i].z / scale_factor;
            if (i = verts.size() - 1) {
              p2.x = verts[0].x / scale_factor;
              p2.y = verts[0].y / scale_factor;
              p2.z = verts[0].z / scale_factor;
            }
            else {
              p2.x = verts[i+1].x / scale_factor;
              p2.y = verts[i+1].y / scale_factor;
              p2.z = verts[i+1].z / scale_factor;
            }
            verts.push_back(p1);
            points.push_back(p1);
            points.push_back(p2);
            std::pair<geometry_msgs::Point> edge = {p1, p2};
            hull_edges.push_back(edge);
          }
          hull_verts.push_back(verts);
          hull_edges.push_back(edges);
      }

    }
}
