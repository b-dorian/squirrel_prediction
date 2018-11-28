
#include <CSVStateImporter.h>

#include "CSVStateImporter.h"

namespace KCL_rosplan {




    void CSVStateImporter::importFromCSVFile(std::string &csvPath, float &inputThreshold){

        threshold = inputThreshold;

        std::stringstream ss;

        ss << "/" << knowledge_base << "/domain/types";
        domain_type_service = ss.str();
        ss.str("");

        ss << "/" << knowledge_base << "/domain/predicates";
        domain_predicate_service = ss.str();
        ss.str("");

        ss << "/" << knowledge_base << "/state/instances";
        state_instance_service = ss.str();
        ss.str("");

        ss << "/" << knowledge_base << "/state/propositions";
        state_proposition_service = ss.str();
        ss.str("");

        ss << "/" << knowledge_base << "/update";
        update_service = ss.str();
        ss.str("");

        ss << "/" << knowledge_base << "/update_array";
        update_array_service = ss.str();
        ss.str("");


        std::ifstream iFile;
        iFile.open((csvPath).c_str());


        // setup service calls
        ros::NodeHandle nh;
        ros::ServiceClient getTypesClient = nh.serviceClient<rosplan_knowledge_msgs::GetDomainTypeService>(domain_type_service);
        ros::ServiceClient getInstancesClient = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>(state_instance_service);
        ros::ServiceClient getDomainPropsClient = nh.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>(domain_predicate_service);
        ros::ServiceClient getPropsClient = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(state_proposition_service);

        ros::ServiceClient updateClient = nh.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>(update_service);
        ros::ServiceClient updateArrayClient = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(update_array_service);


        std::string line;
        std::getline(iFile,line);

        parseCSVFile(iFile);
        std::cout << threshold << "\n";

    }


    void CSVStateImporter::parseCSVFile(std::ifstream &str) {
        std::string line;

        while(std::getline(str,line)){
            std::vector<std::string> separatedLine;
            std::stringstream lineStream(line);
            std::string cell;


            while(std::getline(lineStream,cell, ',')) {
                separatedLine.push_back(cell);
            }

            if (!lineStream && cell.empty()) {
                separatedLine.push_back("");
            }

            checkThresholdandUpdate(separatedLine);
        }
    }

    // compare to threshold and update kb if equal or above
    void CSVStateImporter::checkThresholdandUpdate(std::vector<std::string> separatedLine){

        int predNumber = (separatedLine.size() - 2) / 2;

        for (size_t t = 2; t < (predNumber + 2); t++) {

            if (std::stof (separatedLine[t+predNumber]) >= threshold){
                updateProposition(separatedLine[0].substr(1, separatedLine[0].size() -2),separatedLine[1].substr(1, separatedLine[1].size() -2),stoi(separatedLine[t]));
            }

        }
    }

    // update propostion
    void CSVStateImporter::updateProposition(std::string object1 , std::string object2, int positive){

        // doublcheck logic for adding 0 predicates

    }










} // close namespace