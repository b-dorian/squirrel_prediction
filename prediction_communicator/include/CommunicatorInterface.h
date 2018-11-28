#include "ros/ros.h"

#include "CSVStateGenerator.h"

#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

#include "prediction_communicator/CSVService.h"

#ifndef KCL_communicator_interface
#define KCL_communicator_interface

/**
 * This file contains an interface to the communicator.
 */

namespace KCL_rosplan {

    class CommunicatorInterface
    {
    private:

        ros::NodeHandle* node_handle;

        /* params */
        std::string csv_path;
        std::string csv_name;

        CSVStateGenerator csv_generator;

    public:

        CommunicatorInterface(ros::NodeHandle& nh);
        virtual ~CommunicatorInterface();

        bool runCommunicatorServer(std::string csvPath);
        bool runCommunicatorServerDefault(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        bool runCommunicatorServerParams(prediction_communicator::CSVService::Request &req, prediction_communicator::CSVService::Response &res);

        /* ROS interface */
        ros::Publisher communicator_publisher;
    };

} // close namespace

#endif