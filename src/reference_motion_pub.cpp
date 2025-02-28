#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64MultiArray.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <nlohmann/json.hpp>

#include <vector>
#include <Eigen/Dense>

using json = nlohmann::json;

int main(int argc, char **argv) {
    ros::init(argc, argv, "reference_motion_publisher");
    ros::NodeHandle nh;

    ros::Publisher reference_motion_pub = nh.advertise<std_msgs::Float64MultiArray>("/reference_motion", 1);
    std_msgs::Float64MultiArray reference_motion_msg;

    std::string motion_dir = ros::package::getPath("reference_motion_publisher") + "/motion_data";

    Eigen::MatrixXd motion_data;
    std::vector<int> starting_indices;
    starting_indices.push_back(0);
    motion_data.resize(0, 16);

    if (!boost::filesystem::exists(motion_dir)) {
        ROS_ERROR("The directory does not exist.");
        return -1;
    }

    for (const auto &entry : boost::filesystem::directory_iterator(motion_dir)) {
        std::string file = entry.path().string();
        std::ifstream motion_data_json(file);
        motion_data_json.open(file);
        if (!motion_data_json.is_open()) {
            ROS_ERROR("Failed to open the file.");
            return -1;
        }

        json json_data;
        motion_data_json >> json_data;
        motion_data_json.close();

        Eigen::MatrixXd json_mat(json_data[0].size(), size_t(16));
        for (size_t i = 0; i < json_data[0].size(); i++) {
            const auto &item = json_data[0][i];
            for (size_t j = 0; j < item.size(); j++) {
                json_mat(i, j) = item[j];
            }
        }
        motion_data.conservativeResize(motion_data.rows() + json_mat.rows(), Eigen::NoChange);
        motion_data.bottomRows(json_mat.rows()) = json_mat;
        starting_indices.push_back(motion_data.rows());
    }


    ros::Rate loop_rate(60);    // publish at 50Hz
    // ROS_INFO_STREAM(starting_indices);
    int start = rand() % starting_indices.size();
    int ind = starting_indices[start];
    while (ros::ok()) {
        reference_motion_msg.data.clear();
        for (int i = 0; i < 16; i++) {
            reference_motion_msg.data.push_back(motion_data(ind, i));
        }
        ind++;
        if (ind >= starting_indices[start + 1] || ind >= motion_data.rows()) {
            ROS_INFO("New motion");
            start = rand() % starting_indices.size();
            ind = starting_indices[start];
            // ROS_INFO_STREAM(ind);
        }

        reference_motion_pub.publish(reference_motion_msg);
        ros::spinOnce();
        loop_rate.sleep();
        
    }

    return 0;
}
