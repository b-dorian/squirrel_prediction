#include "CommunicatorInterface.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <string>
#include <streambuf>

namespace KCL_rosplan {

    /*-------------*/
    /* constructor */
    /*-------------*/

    CommunicatorInterface::CommunicatorInterface(ros::NodeHandle& nh)
    {
        node_handle = &nh;

        // connecting to KB
        std::string kbG = "knowledge_base";
        node_handle->getParam("generator_knowledge_base", kbG);
        csv_state_generator.knowledge_base = kbG;

        std::string kbI = "knowledge_base";
        node_handle->getParam("importer_knowledge_base", kbI);
        csv_state_importer.knowledge_base = kbI;

        // publishing generated csv
        std::string communicator_instance = "communicator_instance";
        node_handle->getParam("communicator_topic", communicator_instance);
        communicator_publisher = node_handle->advertise<std_msgs::String>(communicator_instance, 1, true);
    }

    CommunicatorInterface::~CommunicatorInterface()
    {

    }

    /*--------------------*/
    /* Communicator interface */
    /*--------------------*/

    /**
     * problem generation service method (1)
     * loads parameters from param server
     */
    bool CommunicatorInterface::runCSVStateGeneratorServer(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

        // defaults
        csv_export_path = "common/problem.csv";

        // load params
        node_handle->getParam("csv_export_path", csv_export_path);

        // set problem name for ROS_INFO
        std::size_t lastDivide = csv_export_path.find_last_of("/\\");
        if(lastDivide != std::string::npos) {
            csv_name = csv_export_path.substr(lastDivide+1);
        } else {
            csv_name = csv_export_path;
        }

        ROS_INFO("KCL: (%s) (%s) Exporting state to CSV.", ros::this_node::getName().c_str(), csv_name.c_str());
        csv_state_generator.generateCSVFile(csv_export_path);
        ROS_INFO("KCL: (%s) (%s) State was exported.", ros::this_node::getName().c_str(), csv_name.c_str());

        // publish problem
        std::ifstream problemIn(csv_export_path.c_str());
        if(problemIn) {
            std_msgs::String problemMsg;
            problemMsg.data = std::string(std::istreambuf_iterator<char>(problemIn), std::istreambuf_iterator<char>());
            communicator_publisher.publish(problemMsg);
        }

        return true;
    }


    bool CommunicatorInterface::runCSVStateImporterServer(prediction_communicator::CSVService::Request &req, prediction_communicator::CSVService::Response &res) {

        // defaults
        csv_import_path = "common/problem.csv";

        // load params
        node_handle->getParam("csv_import_path", csv_import_path);


        // set problem name for ROS_INFO
        std::size_t lastDivide = csv_import_path.find_last_of("/\\");
        if(lastDivide != std::string::npos) {
            csv_name = csv_import_path.substr(lastDivide+1);
        } else {
            csv_name = csv_import_path;
        }

        ROS_INFO("KCL: (%s) (%s) Importing state from CSV.", ros::this_node::getName().c_str(), csv_name.c_str());
        csv_state_importer.importFromCSVFile(csv_import_path, req.csv_import_threshold);
        ROS_INFO("KCL: (%s) (%s) State was imported.", ros::this_node::getName().c_str(), csv_name.c_str());


        return true;
    }

} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

    srand (static_cast <unsigned> (time(0)));

    ros::init(argc,argv,"prediction_communicator_interface");
    ros::NodeHandle nh("~");

    KCL_rosplan::CommunicatorInterface CommunicatorInterface(nh);

    // start the planning services
    ros::ServiceServer service1 = nh.advertiseService("csv_state_generator_server", &KCL_rosplan::CommunicatorInterface::runCSVStateGeneratorServer, &CommunicatorInterface);
    ros::ServiceServer service2 = nh.advertiseService("csv_state_importer_server", &KCL_rosplan::CommunicatorInterface::runCSVStateImporterServer, &CommunicatorInterface);

    ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());
    ros::spin();

    return 0;
}