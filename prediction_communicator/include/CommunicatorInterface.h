#include "ros/ros.h"

#include "CSVStateGenerator.h"
#include "CSVStateImporter.h"

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
        std::string csv_import_path;
        std::string csv_export_path;
        std::string csv_name;


        CSVStateGenerator csv_state_generator;
        CSVStateImporter csv_state_importer;

    public:

        CommunicatorInterface(ros::NodeHandle& nh);
        virtual ~CommunicatorInterface();

        bool runCSVStateGeneratorServer(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        bool runCSVStateImporterServer(prediction_communicator::CSVService::Request &req, prediction_communicator::CSVService::Response &res);

        /* ROS interface */
        ros::Publisher communicator_publisher;
    };

} // close namespace

#endif