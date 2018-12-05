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

#include "rosplan_knowledge_msgs/GetDomainTypeService.h"
#include "rosplan_knowledge_msgs/GetDomainAttributeService.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"
#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateServiceArray.h"


#ifndef KCL_CSVStateImporter
#define KCL_CSVStateImporter

namespace KCL_rosplan {

    class CSVStateImporter
        {

    private:
        std::string domain_type_service;
        std::string domain_predicate_service;

        std::string state_instance_service;
        std::string state_proposition_service;

        std::string update_service;
        std::string update_array_service;

        float threshold;


        void parseCSVFile(std::ifstream &str);
        void checkThresholdandUpdate(std::vector<std::string> separatedLine);
        void updateProposition(std::string object1, std::string object2, int predicateNumber);

    public:
        std::string knowledge_base;

        void importFromCSVFile(std::string &csvPath, float &inputThreshold);

        };

} // close namespace

#endif