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
        std::string kb = "knowledge_base";
        node_handle->getParam("knowledge_base", kb);
        csv_generator.knowledge_base = kb;

        // publishing "problem"
        std::string communicator_instance = "communicator_instance";
        node_handle->getParam("communicator_topic", communicator_instance);
        communicator_publisher = node_handle->advertise<std_msgs::String>(communicator_instance, 1, true);
    }

    CommunicatorInterface::~CommunicatorInterface()
    {

    }

    /*--------------------*/
    /* Problem interface */
    /*--------------------*/

    /**
     * problem generation service method (1)
     * loads parameters from param server
     */
    bool CommunicatorInterface::runCommunicatorServerDefault(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

        // defaults
        csv_path = "common/problem.csv";

        // load params
        node_handle->getParam("csv_path", csv_path);

        // call problem server
        return runCommunicatorServer(csv_path);
    }

    /**
     * problem generation service method (2)
     * loads parameters from service request
     */
    bool CommunicatorInterface::runCommunicatorServerParams(prediction_communicator::CSVService::Request &req, prediction_communicator::CSVService::Response &res) {
        // call problem server
        bool success = runCommunicatorServer(req.csv_path);
        if(req.csv_string_response) {
            std::ifstream problemIn(req.csv_path.c_str());
            if(problemIn) res.csv_string = std::string(std::istreambuf_iterator<char>(problemIn), std::istreambuf_iterator<char>());
        }
        return success;
    }

    /**
     * planning system; prepares planning; calls planner; parses plan.
     */
    bool CommunicatorInterface::runCommunicatorServer(std::string csvPath) {

        ros::NodeHandle nh("~");

        // save parameter
        csv_path = csvPath;

        // set problem name for ROS_INFO
        std::size_t lastDivide = csv_path.find_last_of("/\\");
        if(lastDivide != std::string::npos) {
            csv_name = csv_path.substr(lastDivide+1);
        } else {
            csv_name = csv_path;
        }

        ROS_INFO("KCL: (%s) (%s) Exporting state to CSV.", ros::this_node::getName().c_str(), csv_name.c_str());
        csv_generator.generateCSVFile(csv_path);
        ROS_INFO("KCL: (%s) (%s) State was exported.", ros::this_node::getName().c_str(), csv_name.c_str());

        // publish problem
        std::ifstream problemIn(csv_path.c_str());
        if(problemIn) {
            std_msgs::String problemMsg;
            problemMsg.data = std::string(std::istreambuf_iterator<char>(problemIn), std::istreambuf_iterator<char>());
            communicator_publisher.publish(problemMsg);
        }

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
    ros::ServiceServer service1 = nh.advertiseService("prediction_communicator_server", &KCL_rosplan::CommunicatorInterface::runCommunicatorServerDefault, &CommunicatorInterface);
    ros::ServiceServer service2 = nh.advertiseService("prediction_communicator_server_params", &KCL_rosplan::CommunicatorInterface::runCommunicatorServerParams, &CommunicatorInterface);

    ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());
    ros::spin();

    return 0;
}