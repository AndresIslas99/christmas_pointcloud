/**
 * @file christmas_cloud_publisher.cpp
 * @brief ROS2 node that publishes Christmas-themed point clouds
 * 
 * Features:
 * - 3D Christmas tree with ornaments
 * - Glowing star on top
 * - Falling snowflakes animation
 * - RGB coloring for festive visualization
 * 
 * @author Andres - Orza Technologies
 * @date December 2024
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <random>
#include <vector>

using namespace std::chrono_literals;

class ChristmasCloudPublisher : public rclcpp::Node
{
public:
    ChristmasCloudPublisher() : Node("christmas_cloud_publisher")
    {
        // Declare parameters
        this->declare_parameter("tree_height", 3.0);
        this->declare_parameter("tree_base_radius", 1.5);
        this->declare_parameter("num_tree_points", 15000);
        this->declare_parameter("num_ornaments", 50);
        this->declare_parameter("num_snowflakes", 500);
        this->declare_parameter("animation_speed", 0.05);
        this->declare_parameter("snowflake_area", 8.0);
        
        // Get parameters
        tree_height_ = this->get_parameter("tree_height").as_double();
        tree_base_radius_ = this->get_parameter("tree_base_radius").as_double();
        num_tree_points_ = this->get_parameter("num_tree_points").as_int();
        num_ornaments_ = this->get_parameter("num_ornaments").as_int();
        num_snowflakes_ = this->get_parameter("num_snowflakes").as_int();
        animation_speed_ = this->get_parameter("animation_speed").as_double();
        snowflake_area_ = this->get_parameter("snowflake_area").as_double();
        
        // Publisher
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "christmas_cloud", 10);
        
        // Timer for animation (30 Hz)
        timer_ = this->create_wall_timer(
            33ms, std::bind(&ChristmasCloudPublisher::publishCloud, this));
        
        // Initialize random generators
        std::random_device rd;
        gen_ = std::mt19937(rd());
        
        // Initialize snowflakes
        initializeSnowflakes();
        
        RCLCPP_INFO(this->get_logger(), 
            "🎄 Christmas Point Cloud Publisher Started! 🎄");
        RCLCPP_INFO(this->get_logger(), 
            "   Tree Height: %.1f m, Ornaments: %d, Snowflakes: %d",
            tree_height_, num_ornaments_, num_snowflakes_);
    }

private:
    // Publishers and timers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Parameters
    double tree_height_;
    double tree_base_radius_;
    int num_tree_points_;
    int num_ornaments_;
    int num_snowflakes_;
    double animation_speed_;
    double snowflake_area_;
    
    // Animation state
    double time_ = 0.0;
    std::vector<std::array<double, 3>> snowflake_positions_;
    std::vector<double> snowflake_speeds_;
    
    // Random generators
    std::mt19937 gen_;
    
    void initializeSnowflakes()
    {
        std::uniform_real_distribution<> pos_dist(-snowflake_area_/2, snowflake_area_/2);
        std::uniform_real_distribution<> height_dist(0.0, tree_height_ + 2.0);
        std::uniform_real_distribution<> speed_dist(0.02, 0.08);
        
        snowflake_positions_.resize(num_snowflakes_);
        snowflake_speeds_.resize(num_snowflakes_);
        
        for (int i = 0; i < num_snowflakes_; ++i) {
            snowflake_positions_[i] = {pos_dist(gen_), pos_dist(gen_), height_dist(gen_)};
            snowflake_speeds_[i] = speed_dist(gen_);
        }
    }
    
    void updateSnowflakes()
    {
        std::uniform_real_distribution<> pos_dist(-snowflake_area_/2, snowflake_area_/2);
        std::uniform_real_distribution<> wobble(-0.01, 0.01);
        
        for (int i = 0; i < num_snowflakes_; ++i) {
            // Fall down
            snowflake_positions_[i][2] -= snowflake_speeds_[i];
            // Add wobble
            snowflake_positions_[i][0] += wobble(gen_);
            snowflake_positions_[i][1] += wobble(gen_);
            
            // Reset if below ground
            if (snowflake_positions_[i][2] < 0.0) {
                snowflake_positions_[i][0] = pos_dist(gen_);
                snowflake_positions_[i][1] = pos_dist(gen_);
                snowflake_positions_[i][2] = tree_height_ + 2.0;
            }
        }
    }
    
    void generateChristmasTree(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
    {
        std::uniform_real_distribution<> angle_dist(0, 2.0 * M_PI);
        std::uniform_real_distribution<> height_dist(0.0, 1.0);
        std::uniform_real_distribution<> radius_noise(-0.05, 0.05);
        
        // Generate tree foliage (cone shape with layers)
        for (int i = 0; i < num_tree_points_; ++i) {
            double h = height_dist(gen_);
            double angle = angle_dist(gen_);
            
            // Cone radius decreases with height
            double max_radius = tree_base_radius_ * (1.0 - h * 0.9);
            
            // Add "layered" look like a real tree
            double layer_factor = std::sin(h * 15.0) * 0.1 + 1.0;
            double r = max_radius * std::sqrt(std::uniform_real_distribution<>(0, 1)(gen_)) * layer_factor;
            r += radius_noise(gen_);
            
            double x = r * std::cos(angle);
            double y = r * std::sin(angle);
            double z = h * tree_height_;
            
            pcl::PointXYZRGB point;
            point.x = x;
            point.y = y;
            point.z = z;
            
            // Green with slight variation
            int green_base = 100 + static_cast<int>(h * 80);
            point.r = 0;
            point.g = std::min(255, green_base + static_cast<int>(radius_noise(gen_) * 100));
            point.b = 0;
            
            cloud.push_back(point);
        }
        
        // Generate tree trunk
        std::uniform_real_distribution<> trunk_radius_dist(0, 0.15);
        for (int i = 0; i < 500; ++i) {
            double angle = angle_dist(gen_);
            double r = trunk_radius_dist(gen_);
            double z = std::uniform_real_distribution<>(-0.5, 0.2)(gen_);
            
            pcl::PointXYZRGB point;
            point.x = r * std::cos(angle);
            point.y = r * std::sin(angle);
            point.z = z;
            point.r = 101;
            point.g = 67;
            point.b = 33;
            
            cloud.push_back(point);
        }
    }
    
    void generateStar(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
    {
        double star_z = tree_height_ + 0.1;
        double star_size = 0.3;
        int num_star_points = 1000;
        
        // Twinkling effect
        double brightness = 0.7 + 0.3 * std::sin(time_ * 5.0);
        
        for (int i = 0; i < num_star_points; ++i) {
            double angle = (static_cast<double>(i) / num_star_points) * 2.0 * M_PI;
            
            // 5-pointed star shape
            double r_factor = (i % 2 == 0) ? 1.0 : 0.4;
            double r = star_size * r_factor;
            
            // Add some thickness
            std::uniform_real_distribution<> noise(-0.03, 0.03);
            
            pcl::PointXYZRGB point;
            point.x = r * std::cos(angle * 5) + noise(gen_);
            point.y = r * std::sin(angle * 5) + noise(gen_);
            point.z = star_z + noise(gen_);
            
            // Golden/yellow with twinkling
            point.r = static_cast<uint8_t>(255 * brightness);
            point.g = static_cast<uint8_t>(215 * brightness);
            point.b = 0;
            
            cloud.push_back(point);
        }
        
        // Star core (bright center)
        for (int i = 0; i < 200; ++i) {
            std::uniform_real_distribution<> core_dist(-0.05, 0.05);
            pcl::PointXYZRGB point;
            point.x = core_dist(gen_);
            point.y = core_dist(gen_);
            point.z = star_z + core_dist(gen_);
            point.r = 255;
            point.g = 255;
            point.b = static_cast<uint8_t>(200 * brightness);
            
            cloud.push_back(point);
        }
    }
    
    void generateOrnaments(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
    {
        // Ornament colors: red, gold, blue, silver
        std::vector<std::array<uint8_t, 3>> colors = {
            {255, 0, 0},      // Red
            {255, 215, 0},    // Gold
            {0, 100, 255},    // Blue
            {192, 192, 192},  // Silver
            {255, 0, 255},    // Magenta
            {0, 255, 255}     // Cyan
        };
        
        std::uniform_real_distribution<> angle_dist(0, 2.0 * M_PI);
        
        for (int i = 0; i < num_ornaments_; ++i) {
            // Position on tree surface
            double h = std::uniform_real_distribution<>(0.1, 0.9)(gen_);
            double angle = angle_dist(gen_);
            double max_radius = tree_base_radius_ * (1.0 - h * 0.9);
            double r = max_radius * 0.9; // Slightly inside the tree surface
            
            double ornament_x = r * std::cos(angle);
            double ornament_y = r * std::sin(angle);
            double ornament_z = h * tree_height_;
            
            // Select color
            auto& color = colors[i % colors.size()];
            
            // Generate spherical ornament
            double ornament_radius = 0.08;
            int points_per_ornament = 50;
            
            // Pulsing effect
            double pulse = 0.8 + 0.2 * std::sin(time_ * 3.0 + i);
            
            for (int j = 0; j < points_per_ornament; ++j) {
                double theta = std::uniform_real_distribution<>(0, M_PI)(gen_);
                double phi = std::uniform_real_distribution<>(0, 2 * M_PI)(gen_);
                double rr = ornament_radius * std::uniform_real_distribution<>(0.8, 1.0)(gen_);
                
                pcl::PointXYZRGB point;
                point.x = ornament_x + rr * std::sin(theta) * std::cos(phi);
                point.y = ornament_y + rr * std::sin(theta) * std::sin(phi);
                point.z = ornament_z + rr * std::cos(theta);
                
                point.r = static_cast<uint8_t>(color[0] * pulse);
                point.g = static_cast<uint8_t>(color[1] * pulse);
                point.b = static_cast<uint8_t>(color[2] * pulse);
                
                cloud.push_back(point);
            }
        }
    }
    
    void generateLights(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
    {
        // Spiral lights around the tree
        int num_lights = 200;
        double spirals = 5.0;
        
        for (int i = 0; i < num_lights; ++i) {
            double t = static_cast<double>(i) / num_lights;
            double h = t;
            double angle = t * spirals * 2.0 * M_PI;
            double max_radius = tree_base_radius_ * (1.0 - h * 0.85);
            
            double x = max_radius * std::cos(angle);
            double y = max_radius * std::sin(angle);
            double z = h * tree_height_;
            
            // Blinking effect - each light blinks at different phase
            double blink = std::sin(time_ * 4.0 + i * 0.5) > 0 ? 1.0 : 0.3;
            
            // Generate small glow around each light
            for (int j = 0; j < 10; ++j) {
                std::uniform_real_distribution<> glow(-0.02, 0.02);
                
                pcl::PointXYZRGB point;
                point.x = x + glow(gen_);
                point.y = y + glow(gen_);
                point.z = z + glow(gen_);
                
                // Alternating warm white and colored lights
                if (i % 3 == 0) {
                    point.r = static_cast<uint8_t>(255 * blink);
                    point.g = static_cast<uint8_t>(200 * blink);
                    point.b = static_cast<uint8_t>(100 * blink);
                } else if (i % 3 == 1) {
                    point.r = static_cast<uint8_t>(255 * blink);
                    point.g = static_cast<uint8_t>(50 * blink);
                    point.b = static_cast<uint8_t>(50 * blink);
                } else {
                    point.r = static_cast<uint8_t>(50 * blink);
                    point.g = static_cast<uint8_t>(255 * blink);
                    point.b = static_cast<uint8_t>(50 * blink);
                }
                
                cloud.push_back(point);
            }
        }
    }
    
    void generateSnowflakes(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
    {
        updateSnowflakes();
        
        for (int i = 0; i < num_snowflakes_; ++i) {
            // Each snowflake is a small cluster of points
            for (int j = 0; j < 5; ++j) {
                std::uniform_real_distribution<> noise(-0.02, 0.02);
                
                pcl::PointXYZRGB point;
                point.x = snowflake_positions_[i][0] + noise(gen_);
                point.y = snowflake_positions_[i][1] + noise(gen_);
                point.z = snowflake_positions_[i][2] + noise(gen_);
                
                // White/light blue snowflakes
                point.r = 240;
                point.g = 248;
                point.b = 255;
                
                cloud.push_back(point);
            }
        }
    }
    
    void generateGround(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
    {
        // Snowy ground
        std::uniform_real_distribution<> pos_dist(-snowflake_area_/2, snowflake_area_/2);
        std::uniform_real_distribution<> height_dist(-0.1, 0.05);
        
        for (int i = 0; i < 3000; ++i) {
            pcl::PointXYZRGB point;
            point.x = pos_dist(gen_);
            point.y = pos_dist(gen_);
            point.z = height_dist(gen_);
            
            // White snow with slight variation
            int snow_val = 230 + static_cast<int>(height_dist(gen_) * 100);
            point.r = std::min(255, snow_val);
            point.g = std::min(255, snow_val);
            point.b = 255;
            
            cloud.push_back(point);
        }
    }
    
    void generatePresents(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
    {
        // Gift boxes under the tree
        std::vector<std::array<double, 3>> present_positions = {
            {0.8, 0.5, 0.15},
            {-0.7, 0.6, 0.1},
            {0.3, -0.9, 0.12},
            {-0.5, -0.7, 0.18},
            {1.0, -0.3, 0.1}
        };
        
        std::vector<std::array<uint8_t, 3>> present_colors = {
            {255, 0, 0},
            {0, 128, 0},
            {0, 0, 255},
            {255, 165, 0},
            {128, 0, 128}
        };
        
        for (size_t p = 0; p < present_positions.size(); ++p) {
            auto& pos = present_positions[p];
            auto& color = present_colors[p];
            double size = pos[2];
            
            // Generate box
            for (int i = 0; i < 200; ++i) {
                std::uniform_real_distribution<> box(-size, size);
                
                pcl::PointXYZRGB point;
                point.x = pos[0] + box(gen_);
                point.y = pos[1] + box(gen_);
                point.z = std::abs(box(gen_));
                
                // Check if on ribbon (gold stripe)
                bool on_ribbon = (std::abs(point.x - pos[0]) < size * 0.15) ||
                                (std::abs(point.y - pos[1]) < size * 0.15);
                
                if (on_ribbon) {
                    point.r = 255;
                    point.g = 215;
                    point.b = 0;
                } else {
                    point.r = color[0];
                    point.g = color[1];
                    point.b = color[2];
                }
                
                cloud.push_back(point);
            }
        }
    }
    
    void publishCloud()
    {
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cloud.header.frame_id = "world";
        
        // Generate all Christmas elements
        generateGround(cloud);
        generateChristmasTree(cloud);
        generateStar(cloud);
        generateOrnaments(cloud);
        generateLights(cloud);
        generateSnowflakes(cloud);
        generatePresents(cloud);
        
        // Convert to ROS message
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(cloud, output);
        output.header.stamp = this->now();
        output.header.frame_id = "world";
        
        cloud_pub_->publish(output);
        
        // Update animation time
        time_ += animation_speed_;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChristmasCloudPublisher>();
    
    RCLCPP_INFO(node->get_logger(), "");
    RCLCPP_INFO(node->get_logger(), "  🎄 Merry Christmas! 🎄");
    RCLCPP_INFO(node->get_logger(), "  ⭐        *        ⭐");
    RCLCPP_INFO(node->get_logger(), "       *** ***");
    RCLCPP_INFO(node->get_logger(), "      *********");
    RCLCPP_INFO(node->get_logger(), "     ***********");
    RCLCPP_INFO(node->get_logger(), "    *************");
    RCLCPP_INFO(node->get_logger(), "         |||");
    RCLCPP_INFO(node->get_logger(), "  🎁 🎁 🎁 🎁 🎁 🎁");
    RCLCPP_INFO(node->get_logger(), "");
    RCLCPP_INFO(node->get_logger(), "Publishing on topic: /christmas_cloud");
    RCLCPP_INFO(node->get_logger(), "Visualize with: ros2 launch christmas_pointcloud christmas.launch.py");
    RCLCPP_INFO(node->get_logger(), "");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
