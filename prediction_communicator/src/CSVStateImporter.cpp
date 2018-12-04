
#include <CSVStateImporter.h>

#include "CSVStateImporter.h"

namespace KCL_rosplan {




    void CSVStateImporter::importFromCSVFile(std::string &csvPath, float &inputThreshold){

        threshold = inputThreshold;

        std::stringstream ss;

        ss << "/" << knowledge_base << "/domain/predicates";
        domain_predicate_service = ss.str();
        ss.str("");

        ss << "/" << knowledge_base << "/update";
        update_service = ss.str();
        ss.str("");

        std::ifstream iFile;
        iFile.open((csvPath).c_str());

        std::string line;
        std::getline(iFile,line);

        parseCSVFile(iFile);


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
                updateProposition(separatedLine[0].substr(1, separatedLine[0].size() -2),separatedLine[1].substr(1, separatedLine[1].size() -2),t-2,stoi(separatedLine[t]));
            }

        }
    }

    // update propostion
    // Note !!! doublecheck logic for adding 0 predicates
    // positive (1) definietly add
    // negative (0) add as isNegative = true or do not add at all?

    // Propositions which do not have a domain equivalent should be imported? ( ex: in dorian dorian) - currently true
    // Can be used for domain modification?

    // allow addition to kb of
    void CSVStateImporter::updateProposition(std::string object1 , std::string object2, int predicateNumber, int positive){

        // setup service calls
        ros::NodeHandle nh;
        ros::ServiceClient getDomainPropsClient = nh.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>(domain_predicate_service);
        ros::ServiceClient updateClient = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>(update_service);


        rosplan_knowledge_msgs::GetDomainAttributeService domainAttrSrv;
        if (!getDomainPropsClient.call(domainAttrSrv)) {
            ROS_ERROR("KCL: (CSVStateImporter) Failed to call service %s", domain_predicate_service.c_str());
        } else {

            int count = 0;
            std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator ait = domainAttrSrv.response.items.begin();
            for (; ait != domainAttrSrv.response.items.end(); ait++) {
                if (predicateNumber == count){

                    rosplan_knowledge_msgs::KnowledgeItem prop;
                    prop.knowledge_type = 1;

                    prop.attribute_name = ait->name;

                    diagnostic_msgs::KeyValue param;
                    param.key = ait->typed_parameters[0].key;
                    param.value = object1;
                    prop.values.push_back(param);
                    param.key = ait->typed_parameters[1].key;
                    param.value = object2;
                    prop.values.push_back(param);

                    rosplan_knowledge_msgs::KnowledgeUpdateService updateService;
                    updateService.request.update_type = 0;
                    updateService.request.knowledge = prop;
                    if (!updateClient.call(updateService)){
                        ROS_ERROR("KCL: (CSVStateImporter) Failed to call service %s", update_service.c_str());
                    } else{
                        ROS_INFO("KCL: (%s) (%s) (%s) Proposition was imported.", ait->name.c_str(), object1.c_str(), object2.c_str());
                    }
                    ;
                }
                else{
                    ++count;
                }

            }

        };




    }










} // close namespace