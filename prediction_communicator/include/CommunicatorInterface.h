#include "ros/ros.h"

#include "CSVStateGenerator.h"
#include "CSVStateImporter.h"

#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

#include "prediction_communicator/CSVImporterService.h"
#include "prediction_communicator/CSVGeneratorService.h"

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

        bool runCSVStateGeneratorServer(prediction_communicator::CSVGeneratorService::Request &req, prediction_communicator::CSVGeneratorService::Response &res);
        bool runCSVStateImporterServer(prediction_communicator::CSVImporterService::Request &req, prediction_communicator::CSVImporterService::Response &res);

        /* ROS interface */
        ros::Publisher communicator_publisher;
    };

} // close namespace

#endif