#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <algorithm>
#include <cmath>

class HighlevelController {
public:
    HighlevelController(ros::NodeHandle& nh) {
        // Initialize publishers and subscribers
        laser_sub_ = nh.subscribe("/scan", 10, &HighlevelController::LaserCallback, this);
        vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        
        // Initialize other variables
        pillar_pos_[0] = 0.0;
        pillar_pos_[1] = 0.0;
    }

private:
    ros::Subscriber laser_sub_;
    ros::Publisher vel_pub_;
    float pillar_pos_[2];  // Position of the pillar (x, y)

    void LaserCallback(const sensor_msgs::LaserScan& msg) {
        int cluster_size_threshold = 10;    
        float dist_threshold = 0.2;
        float width_threshold = 0.07;  
        float curvature_high_threshold = 30.`0;
        float curvature_low_threshold = 0.05;   
        bool pillar_found = false;
        float min_dist = std::numeric_limits<float>::max();
        int min_index = -1;

        for (size_t i = 1; i < msg.ranges.size() - 1; ++i) {
            float dist = msg.ranges[i];
            if (!std::isfinite(dist) || dist <= 0.0) continue;  

            float prev_dist = msg.ranges[i - 1];
            float next_dist = msg.ranges[i + 1];
            float curvature = std::fabs(prev_dist - 2 * dist + next_dist);

            if (curvature > curvature_low_threshold && curvature < curvature_high_threshold) {
                int cluster_size = 0;
                float cluster_width = 0.0;

                for (int j = i; j < msg.ranges.size() && std::fabs(msg.ranges[j] - dist) < dist_threshold; ++j) {
                    cluster_size++;
                    cluster_width += msg.angle_increment * msg.ranges[j];
                }

                if (cluster_size < cluster_size_threshold && cluster_width < width_threshold && dist < min_dist) {
                    pillar_found = true;
                    min_dist = dist;
                    min_index = i;
                }
            }
        }

        if (pillar_found && min_index >= 0) {
            float ang = msg.angle_min + msg.angle_increment * min_index;

            pillar_pos_[0] = min_dist * std::cos(ang);
            pillar_pos_[1] = min_dist * std::sin(ang);

            ROS_INFO_STREAM("Pillar detected at " << min_dist << "m and "
                            << ang / M_PI * 180.0 << " degrees");
            ROS_INFO_STREAM("Pillar's coordinate to Turtlebot is [" << pillar_pos_[0]
                            << ", " << pillar_pos_[1] << "]");

            // DriveTurtlebot();
        } else {
            ROS_INFO_STREAM("No pillar detected.");
        }
    }

    void DriveTurtlebot() {
        // Combine heading and speed adjustments
        geometry_msgs::Twist cmd;
        cmd.linear.x = std::min(0.5f, (pillar_pos_[0] - 0.2f));  // Ensure safe approach and stop at 0.2m
        cmd.angular.z = 0.8 * std::atan2(pillar_pos_[1], pillar_pos_[0]);
        vel_pub_.publish(cmd);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtlebot_homing_task");
    ros::NodeHandle nh;
    HighlevelController controller(nh);
    ros::spin();
    return 0;
}
