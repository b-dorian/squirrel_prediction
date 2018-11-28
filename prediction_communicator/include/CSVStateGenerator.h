/**
 * This class is responsible for generating the CSV export.
 * This is done by using the objects requested from Knowledge services.
 */
#include "ros/ros.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>

#include "rosplan_knowledge_msgs/GetDomainNameService.h"
#include "rosplan_knowledge_msgs/GetDomainTypeService.h"
#include "rosplan_knowledge_msgs/GetDomainAttributeService.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"
#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "rosplan_knowledge_msgs/GetMetricService.h"

#ifndef KCL_CSVStateGenerator
#define KCL_CSVStateGenerator

namespace KCL_rosplan {

    class CSVStateGenerator {
    private:
        std::vector<std::string> instances;
        std::vector<std::string> instances_types;
        std::vector<std::string> instances_super_types;
        std::vector<std::string> types;
        std::vector<std::string> super_types;

        std::string domain_type_service;
        std::string domain_predicate_service;

        std::string state_instance_service;
        std::string state_proposition_service;

        std::string getTypeParent(std::string child);
        bool areTypesRelated(std::string possibleParent, std::string possibleChild);
        void makePredicateNames(std::ofstream &pFile, ros::ServiceClient getDomainPropsClient);
        void makeInstanceCombinations(ros::ServiceClient getTypesClient,ros::ServiceClient getInstancesClient);
        void makeFacts(std::ofstream &pFile, ros::ServiceClient getDomainPropsClient, ros::ServiceClient getPropsClient);

    public:
        std::string knowledge_base;

        void generateCSVFile(std::string &csvPath);
    };
} // close namespace

#endif
