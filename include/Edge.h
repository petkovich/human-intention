#pragma once

#include <Conflict.h>
#include <string>
#include <vector>
#include <tinyxml2.h>
#include <iostream>

class Edge2
{
  private:
    int id;
    int startNodeId;
    int endNodeId;

    double startAngle;
    double endAngle;
    double N1;
    double N2;
    double maxSpeedLoaded;
    double maxSpeedUnloaded;
    double costFactor;
    std::string startNodeDescription;
    std::string endNodeDescription;
    std::vector<Conflict> conflicts;

  public:
    Edge2() {}
    int getId() { return id; }
    int getStartNodeId() { return startNodeId; }
    int getEndNodeId() { return endNodeId; }
    double getStartAngle() { return startAngle; }
    double getEndAngle() { return endAngle; }
    double getN1() { return N1; }
    double getN2() { return N2; }
    double getMaxSpeedLoaded() { return maxSpeedLoaded; }
    double getMaxSpeedUnloaded() { return maxSpeedUnloaded; }
    double getCostFactor() { return costFactor; }
    std::string getStartNodeDescription() { return startNodeDescription; }
    std::string getEndNodeDescription() { return endNodeDescription; }
    std::vector<Conflict> &getConflicts() { return conflicts; }
    std::vector<Conflict> *getConflicts(int) { return &conflicts; }
    
    void addEdge(int start, int end){
        startNodeId=start;
        endNodeId=end;
    }
    bool loadFromXML(tinyxml2::XMLElement *element)
    {
        if (element == nullptr)
        {
            std::cout << "Edge null pointer" << std::endl;
            return true;
        }

        bool error;
        error = element->QueryIntAttribute("Id", &id);
        if (error)
        {
            std::cout << "Cannot load ID" << std::endl;
            return true;
        }
        error = element->QueryIntAttribute("StartNodeId", &startNodeId);
        if (error)
        {
            std::cout << "Cannot load StartNodeID" << std::endl;
            return true;
        }
        error = element->QueryIntAttribute("EndNodeId", &endNodeId);
        if (error)
        {
            std::cout << "Cannot load EndNodeID" << std::endl;
            return true;
        }
        error = element->QueryDoubleAttribute("StartAngle", &startAngle);
        if (error)
        {
            std::cout << "Cannot load StartAngle" << std::endl;
            return true;
        }
        error = element->QueryDoubleAttribute("EndAngle", &endAngle);
        if (error)
        {
            std::cout << "Cannot load EndAngle" << std::endl;
            return true;
        }
        error = element->QueryDoubleAttribute("N1", &N1);
        if (error)
        {
            std::cout << "Cannot load N1" << std::endl;
            return true;
        }
        error = element->QueryDoubleAttribute("N2", &N2);
        if (error)
        {
            std::cout << "Cannot load N2" << std::endl;
            return true;
        }
        error = element->QueryDoubleAttribute("MaxSpeedLoaded", &maxSpeedLoaded);
        if (error)
        {
            std::cout << "Cannot load MaxSpeedLoaded" << std::endl;
            return true;
        }
        error = element->QueryDoubleAttribute("MaxSpeedUnloaded", &maxSpeedUnloaded);
        if (error)
        {
            std::cout << "Cannot load MaxSpeedUnloaded" << std::endl;
            return true;
        }
        error = element->QueryDoubleAttribute("CostFactor", &costFactor);
        if (error)
        {
            std::cout << "Cannot load CostFactor" << std::endl;
            return true;
        }

        // load the strings
        const char *s;
        error = element->QueryStringAttribute("StartNodeDescription", &s);
        if (error)
        {
            std::cout << "Cannot load StartNodeDescription" << std::endl;
            return true;
        }
        startNodeDescription = std::string(s);
        error = element->QueryStringAttribute("EndNodeDescription", &s);
        if (error)
        {
            std::cout << "Cannot load EndNodeDescription" << std::endl;
            return true;
        }
        endNodeDescription = std::string(s);

        tinyxml2::XMLElement *child = element->FirstChildElement("Conflicts");
        if (child == nullptr)
        {
            std::cout << "Cannot load Conflicts Child" << std::endl;
            return true;
        }

        // load conflicts
        for (child = child->FirstChildElement(); child != nullptr; child = child->NextSiblingElement())
        {
            Conflict conflict;
            error = conflict.loadFromXML(child);
            if (error)
            {
                std::cout << "Cannot load CONFLICT" << std::endl;
                return true;
            }
            conflicts.push_back(conflict);
        }
        return false;
    }
};
