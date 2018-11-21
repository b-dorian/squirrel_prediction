
#include <CSVGenerator.h>

#include "CSVGenerator.h"

namespace KCL_rosplan {

    /**
	 * export propositions to a CSV file
	 * This file is later read by predictor.
	 */
    void CSVGenerator::generateCSVFile(std::string &csvPath) {

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

        std::ofstream pFile;
        pFile.open((csvPath).c_str());


        // setup service calls
        ros::NodeHandle nh;
        ros::ServiceClient getTypesClient = nh.serviceClient<rosplan_knowledge_msgs::GetDomainTypeService>(domain_type_service);
        ros::ServiceClient getInstancesClient = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>(state_instance_service);
        ros::ServiceClient getDomainPropsClient = nh.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>(domain_predicate_service);
        ros::ServiceClient getPropsClient = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(state_proposition_service);


        //write label row
        pFile << "'object1','object2'";

        //write csv
        makePredicateNames(pFile, getDomainPropsClient);
        makeInstanceCombinations(getTypesClient,getInstancesClient);
        makeFacts(pFile, getDomainPropsClient, getPropsClient);

    }

    // get the parent of a type (it is mandatory every type in the PDDL domain has a super-type defined, with "object" being the root of the type tree)
    std::string CSVGenerator::getTypeParent(std::string child){
        std::string parent = child;
        for(size_t t=0; t<super_types.size(); t++) {
            if (child == types[t]) parent = super_types[t];
        }
        return parent;
    }


    // check if two types are part of the same branch of the type tree
    bool CSVGenerator::areTypesRelated(std::string possibleParent, std::string possibleChild) {
        bool typesRelated = false;
        std::string tempChild = possibleChild;
        if (possibleChild != possibleParent) {
            for (size_t t = 0; t < super_types.size(); t++) {
                tempChild = getTypeParent(tempChild);
                if (tempChild == possibleParent) {
                    typesRelated = true;
                    break;
                }
            }
            return typesRelated;
        }
    }

    //write label row
    void CSVGenerator::makePredicateNames(std::ofstream &pFile, ros::ServiceClient getDomainPropsClient){
        rosplan_knowledge_msgs::GetDomainAttributeService domainAttrSrv;
        if (!getDomainPropsClient.call(domainAttrSrv)) {
            ROS_ERROR("KCL: (CSVGenerator) Failed to call service %s", domain_predicate_service.c_str());
        } else {

            std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator ait = domainAttrSrv.response.items.begin();
            for (; ait != domainAttrSrv.response.items.end(); ait++) {
                pFile << ",'" + ait->name+"'";
            }
            pFile << std::endl;
        };
    }

    //write instance combinations and gather associated types informations
    void CSVGenerator::makeInstanceCombinations(ros::ServiceClient getTypesClient, ros::ServiceClient getInstancesClient){
        rosplan_knowledge_msgs::GetDomainTypeService typeSrv;
        if (!getTypesClient.call(typeSrv)) {
            ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service %s", domain_type_service.c_str());
        }
        for(size_t t=0; t<typeSrv.response.types.size(); t++) {
            rosplan_knowledge_msgs::GetInstanceService instanceSrv;
            instanceSrv.request.type_name = typeSrv.response.types[t];

            types.push_back(typeSrv.response.types[t]);
            super_types.push_back(typeSrv.response.super_types[t]);

            if (!getInstancesClient.call(instanceSrv)) {
                ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service %s: %s", state_instance_service.c_str(),
                          instanceSrv.request.type_name.c_str());
            } else {
                if (instanceSrv.response.instances.size() == 0) continue;

                for (size_t i = 0; i < instanceSrv.response.instances.size(); i++) {
                    instances.push_back(instanceSrv.response.instances[i]);
                    instances_types.push_back(instanceSrv.request.type_name);
                    instances_super_types.push_back(typeSrv.response.super_types[t]);
                }
            }
        }

    }

    //write predicates vs propositions evaluation
    void CSVGenerator::makeFacts(std::ofstream &pFile, ros::ServiceClient getDomainPropsClient, ros::ServiceClient getPropsClient){
        rosplan_knowledge_msgs::GetDomainAttributeService domainAttrSrv;
        for(size_t i=0; i<instances.size(); i++) {
            for(size_t j=0; j<instances.size(); j++) {

                pFile << "'" + instances[i] + "','" + instances[j] + "'";

                if (!getDomainPropsClient.call(domainAttrSrv)) {
                    ROS_ERROR("KCL: (CSVGenerator) Failed to call service %s", domain_predicate_service.c_str());
                } else {

                    std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator ait = domainAttrSrv.response.items.begin();
                    for (; ait != domainAttrSrv.response.items.end(); ait++) {
                        std:: string output = ",";

                        rosplan_knowledge_msgs::GetAttributeService attrSrv;

                        if (!getPropsClient.call(attrSrv)) {
                            ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service %s: %s",
                                      state_proposition_service.c_str(), attrSrv.request.predicate_name.c_str());
                        } else {

                            bool notFound = true;
                            for (size_t k = 0; k<attrSrv.response.attributes.size(); k++) {

                                rosplan_knowledge_msgs::KnowledgeItem attr = attrSrv.response.attributes[k];
                                if (areTypesRelated(ait->typed_parameters[0].value, instances_types[i]) && areTypesRelated(ait->typed_parameters[1].value, instances_types[j]))
                                {

                                    if ((attr.attribute_name == ait->name) && (attr.values[0].value == instances[i]) && (attr.values[1].value == instances[j])) {
                                        output = ",1";
                                        notFound = false;
                                    }
                                    else{
                                        if (notFound == true)
                                        {
                                            output = ",0";
                                        }
                                    }
                                }
                            }
                        }
                        pFile << output;
                    }
                    pFile << std::endl;
                }
            }
        }
    }




} // close namespace
