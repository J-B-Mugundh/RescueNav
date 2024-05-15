#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <gnc_functions.hpp> // This file contains the necessary function prototypes from intelligent quads

// Define a struct to represent a state-action pair
struct QEntry
{
    int state;     // State index
    int action;    // Action index
    double qValue; // Q-value
};

// Global variables
std::vector<QEntry> qTable;             // Global Q-table
geometry_msgs::Point currentCoordinate; // Current drone coordinates
geometry_msgs::Point takeoffPoint;      // Takeoff point

// Function to load the Q-table from CSV
void loadQTable(const std::string &filename)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Error: Unable to open Q-table file " << filename << std::endl;
        return;
    }
    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string stateStr, actionStr, qValueStr;
        if (!(iss >> stateStr >> actionStr >> qValueStr))
        {
            continue;
        }
        int state = std::stoi(stateStr);
        int action = std::stoi(actionStr);
        double qValue = std::stod(qValueStr);
        qTable.push_back({state, action, qValue});
    }
    file.close();
}

// Functions to execute the optimal trajectory based on the Q-table

// Find the closest state in the Q-table to the current state
int findClosestState(float signal_strength)
{
    double minDistance = std::numeric_limits<double>::max();
    int closestStateIndex = -1;

    for (size_t i = 0; i < qTable.size(); ++i)
    {
        double distance = sqrt(pow(qTable[i].state - currentCoordinate.x, 2) +
                               pow(qTable[i].action - currentCoordinate.y, 2));
        if (distance < minDistance)
        {
            minDistance = distance;
            closestStateIndex = i;
        }
    }

    return closestStateIndex;
}

// Execute optimal trajectory using Q-table
void executeOptimalTrajectory()
{
    // Load zones from CSV file
    std::vector<std::vector<geometry_msgs::Point>> zones;
    std::ifstream zones_file("zones.csv");
    std::string line;
    while (std::getline(zones_file, line))
    {
        std::istringstream iss(line);
        std::vector<geometry_msgs::Point> zone;
        double x, y, z;
        while (iss >> x >> y >> z)
        {
            geometry_msgs::Point point;
            point.x = x;
            point.y = y;
            point.z = z;
            zone.push_back(point);
        }
        zones.push_back(zone);
    }

    // Load depth matrix from CSV file
    std::vector<std::vector<int>> depth_matrix;
    std::ifstream depth_file("depthmatrix.csv");
    while (std::getline(depth_file, line))
    {
        std::vector<int> row;
        std::istringstream iss(line);
        int value;
        while (iss >> value)
        {
            row.push_back(value);
        }
        depth_matrix.push_back(row);
    }

    // Publish waypoints for the drone to follow
    ros::NodeHandle nh;
    ros::Publisher waypointPub = nh.advertise<geometry_msgs::Point>("waypoints", 1);
    ros::Rate rate(1); // Adjust the rate based on your requirement

    while (ros::ok())
    {
        // Get current position from the drone
        getDroneCoordinates()
        // Assuming currentCoordinate is set elsewhere in the code

        // Calculate LoRa signal strength for current position
        float signal_strength = calculateLoraStrength(takeoffPoint, currentCoordinate, depth_matrix);

        // Find the closest state in the Q-table to the current state based on signal strength
        int closestStateIndex = findClosestState(signal_strength);

        // Check if a valid closest state is found
        if (closestStateIndex != -1)
        {
            // Get the action corresponding to the closest state
            double nextX = qTable[closestStateIndex].state;  // Action x-coordinate
            double nextY = qTable[closestStateIndex].action; // Action y-coordinate

            // Calculate altitude based on signal strength from Q-table
            double nextAltitude = qTable[closestStateIndex].qValue; // Action altitude

            // Create the next waypoint based on the action
            geometry_msgs::Point nextWaypoint;
            nextWaypoint.x = nextX;
            nextWaypoint.y = nextY;
            nextWaypoint.z = nextAltitude;

            // Adjust waypoint to avoid obstacles
            adjustWaypointForObstacle(nextWaypoint);

            // Publish the next waypoint
            waypointPub.publish(nextWaypoint);
        }

        rate.sleep();
        ros::spinOnce(); // Handle ROS callbacks
    }
    
    // Initiate landing sequence
    initiateLandingSequence();
    rate.sleep();
    ros::spinOnce(); // Handle ROS callbacks
    }
}

// Function to adjust the waypoint to avoid obstacles
void adjustWaypointForObstacle(geometry_msgs::Point &waypoint)
{
    // Define obstacle avoidance parameters
    double obstacleRangeThreshold = 2.0; // Threshold distance to consider an obstacle (adjust as needed)
    double deviationDistance = 1.0;      // Distance to deviate from the obstacle (adjust as needed)

    // Subscribe to laser scan data
    ros::NodeHandle nh;
    ros::Subscriber laserScanSub = nh.subscribe<sensor_msgs::LaserScan>("laser_scan_topic", 1,
                                                                        [&](const sensor_msgs::LaserScan::ConstPtr &msg)
                                                                        {
                                                                            // Loop through laser scan data
                                                                            for (size_t i = 0; i < msg->ranges.size(); ++i)
                                                                            {
                                                                                double range = msg->ranges[i];
                                                                                double angle = msg->angle_min + i * msg->angle_increment;

                                                                                // Check if obstacle is within threshold range
                                                                                if (range < obstacleRangeThreshold)
                                                                                {
                                                                                    // Calculate deviation angle
                                                                                    double deviationAngle = atan2(waypoint.y - currentCoordinate.y, waypoint.x - currentCoordinate.x);
                                                                                    // Calculate new waypoint coordinates by deviating from the obstacle
                                                                                    waypoint.x = currentCoordinate.x + (deviationDistance * cos(deviationAngle));
                                                                                    waypoint.y = currentCoordinate.y + (deviationDistance * sin(deviationAngle));
                                                                                    ROS_INFO("Adjusted waypoint to avoid obstacle.");
                                                                                    return; // Exit loop once adjustment is made
                                                                                }
                                                                            }
                                                                        });

    // Spin ROS to process laser scan data
    ros::spinOnce();
}

// Function to get the drone's current coordinates
void getDroneCoordinates(const geometry_msgs::Point::ConstPtr &msg)
{
    currentCoordinate = *msg;
}

// Function to store the takeoff point
void storeTakeoffPoint(const geometry_msgs::Point::ConstPtr &msg)
{
    takeoffPoint = *msg;
}

// Function to calculate LoRa strength based on obstacles between takeoff point and current position
float calculateLoraStrength(const geometry_msgs::Point &takeoff_point, const geometry_msgs::Point &current_position, const std::vector<std::vector<int>> &depth_matrix)
{
    // Implementation of calculateLoraStrength function
}

void initiateLandingSequence()
{
    // Publish landing command or call a landing function
    ros::NodeHandle nh;
    ros::Publisher landingPub = nh.advertise<std_msgs::String>("landing_command", 1);

    // Create a message for landing command
    std_msgs::String landingMsg;
    landingMsg.data = "land"; // Command to initiate landing

    // Publish the landing command
    landingPub.publish(landingMsg);

    // Print message to indicate landing initiation
    ROS_INFO("Landing sequence initiated.");
}

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "trajectory_execution_node");
    ros::NodeHandle nh;

    // Load the Q-table from CSV
    std::string qTableFilename = "q_table.csv";
    loadQTable(qTableFilename);

    // Subscribe to the topic publishing the drone's coordinates
    ros::Subscriber coordSub = nh.subscribe<geometry_msgs::Point>("drone_coordinates", 1, getDroneCoordinates);

    // Subscribe to the topic publishing the takeoff point
    ros::Subscriber takeoffSub = nh.subscribe<geometry_msgs::Point>("takeoff_point", 1, storeTakeoffPoint);

    // Execute the optimal trajectory based on the Q-table
    executeOptimalTrajectory();

    // Spin ROS
    ros::spin();

    return 0;
}