#include <WarehouseGraph.h>
#include <string>
#include <fstream>
#include <iostream>
#include <tinyxml2.h>
#include <LayoutSettings.h>
#include <Node.h>
#include <Edge.h>
#include <Utilities.h>
#include <algorithm>

using namespace std;
using namespace tinyxml2;

using AdjacencyList = std::vector<std::vector<int>>;
using FPoint = Point<float>;

// Empty constructor
WarehouseGraph::WarehouseGraph() {}

// Getter on number of nodes
int WarehouseGraph::getNumNodes()
{
    return nodes.size();
}

// getter on num of edges in graph;
int WarehouseGraph::getNumEdges()
{
    return edges.size();
}

bool WarehouseGraph::loadNodesXML(XMLElement *root)
{
    if (root == nullptr)
        return true;
    XMLElement *iElement;
    for (iElement = root->FirstChildElement(); iElement != nullptr; iElement = iElement->NextSiblingElement())
    {
        // if we find node representing node group
        std::string elementName(iElement->Name());
        if (elementName.find("Nodes") != std::string::npos)
        {
            // iterate over all children nodes
            XMLElement *iNodeChild;
            for (iNodeChild = iElement->FirstChildElement(); iNodeChild != nullptr; iNodeChild = iNodeChild->NextSiblingElement())
            {
                // load nodes
                Node n(iNodeChild, orientedNodes, nodes.size());
                nodes.push_back(n);
            }
        }
    }
}

bool WarehouseGraph::loadEdgesXML(XMLElement *root)
{
    if (root == nullptr)
    {
        std::cout << "Root is wrong" << std::endl;
        return true;
    }
    XMLElement *iElement = root->FirstChildElement("Edges");
    if (iElement != nullptr)
    {
        // iterate over all children nodes
        XMLElement *iNodeChild;
        bool error;
        for (iNodeChild = iElement->FirstChildElement(); iNodeChild != nullptr; iNodeChild = iNodeChild->NextSiblingElement())
        {
            // load nodes
            Edge n;
            error = n.loadFromXML(iNodeChild);
            if(error)
            {
                std::cout << "There was an error while loading edges" << std::endl;
                return true;
            }
            edges.push_back(n);
        }
    }
}

bool WarehouseGraph::loadXML(std::string xmlFile)
{
    XMLDocument doc;
    XMLError result = doc.LoadFile(xmlFile.c_str());
    if (result == XML_SUCCESS)
    {
        printf("XMLERROR is %d\nXML loading successfull.\n", result);
    }
    else
    {
        printf("XMLERROR is %d\nXML loading unsuccessfull.\n", result);
    }

    // extract root element
    XMLElement *root = doc.RootElement();
    if (root == nullptr)
        return false;
    // load nodes and edges from the XML file
    bool error;
    error = settings.loadFromXML(root->FirstChildElement("LayoutSettings"));
    if (error)
    {
        std::cout << "Cannot load settings." << std::endl;
        return true;
    }
    error = loadNodesXML(root);
    if (error)
    {
        std::cout << "Cannot load nodes" << std::endl;
        return true;
    }
    error = loadEdgesXML(root);
    if (error)
    {
        std::cout << "Cannot load edges." << std::endl;
        return true;
    }

    // fill in translation tables from xml ids to the ids in vectors
    for (int i = 0; i < nodes.size(); i++)
    {
        nodeIdTranslationTable.emplace(nodes[i].getId(), i);
    }
    for (int i = 0; i < edges.size(); i++)
    {
        edgeIdTranslationTable.emplace(edges[i].getId(), i);
    }
    for (int i = 0; i < orientedNodes.size(); i++)
    {
        orientedNodeTranslationTable.emplace(orientedNodes[i].getId(), i);
    }
        /*// set length of edge
        Point<float> aPoint = nodes[translateNodeId(e.getStartNodeId())].getCoords();
        Point<float> bPoint = nodes[translateNodeId(e.getEndNodeId())].getCoords();
        //if points are not on a line, then its a spline
        double len;
        if (aPoint.getX() != bPoint.getX() && aPoint.getY() != bPoint.getY())
        {
            // approximate as 1/4 of elipse
            double xDiff = std::abs(aPoint.getX() - bPoint.getX());
            double yDiff = std::abs(aPoint.getY() - bPoint.getY());
            len = 3.1415 * std::sqrt(2 * (xDiff * xDiff + yDiff * yDiff));
            len = len / 4;
        }
        else
        {
            len = Util::euclidDistance(aPoint, bPoint);
        }
        e.setLength(len);*/
    // fill in rotation adjacency or oriented nodes
    /*oriNodeRotateAdjacency.resize(orientedNodes.size());
    for (int i = 0; i < nodes.size(); i++)
    {
        std::vector<int> &orientedNodesIds = nodes[i].getOrientedNodes();
        // for every node find highest lower rotated oriented node
        // and lowest higher oriented node
        std::vector<std::pair<int, double>> sorted;
        for (int j = 0; j < orientedNodesIds.size(); j++)
        {
            sorted.push_back(std::make_pair(j, orientedNodes[orientedNodesIds[j]].getOrientation()));
        }
        // sort them according to orientation
        std::sort(sorted.begin(), sorted.end(), [](const std::pair<int, double> &a, const std::pair<int, double> &b) { return a.second < b.second; });
        // fill in the adjacency
        for (int j = 0; j < sorted.size(); j++)
        {
            if (j == 0)
            {
                // if sorted has only size 1, then no neighbour
                if (sorted.size() == 1)
                {
                    continue;
                }
                // take only right neighbour
                else if (sorted.size() == 2)
                {
                    oriNodeRotateAdjacency[orientedNodesIds[sorted[0].first]].push_back(orientedNodesIds[sorted[1].first]);
                }
                // take right and left
                else
                {
                    oriNodeRotateAdjacency[orientedNodesIds[sorted[0].first]].push_back(orientedNodesIds[sorted[1].first]);
                    oriNodeRotateAdjacency[orientedNodesIds[sorted[0].first]].push_back(orientedNodesIds[sorted[sorted.size() - 1].first]);
                }
            }
            else if (j == sorted.size() - 1)
            {
                if (sorted.size() == 1)
                {
                    continue;
                }
                else if (sorted.size() == 2)
                {
                    oriNodeRotateAdjacency[orientedNodesIds[sorted[1].first]].push_back(orientedNodesIds[sorted[0].first]);
                }
                else
                {
                    oriNodeRotateAdjacency[orientedNodesIds[sorted[j].first]].push_back(orientedNodesIds[sorted[0].first]);
                    oriNodeRotateAdjacency[orientedNodesIds[sorted[j].first]].push_back(orientedNodesIds[sorted[j - 1].first]);
                }
            }
            else
            {
                oriNodeRotateAdjacency[orientedNodesIds[sorted[j].first]].push_back(orientedNodesIds[sorted[j + 1].first]);
                oriNodeRotateAdjacency[orientedNodesIds[sorted[j].first]].push_back(orientedNodesIds[sorted[j - 1].first]);
            }
        }
    }*/

    return false;
}

int WarehouseGraph::translateEdgeId(int edgeId)
{
    if (edgeIdTranslationTable.count(edgeId) > 0)
    {
        return edgeIdTranslationTable[edgeId];
    }
    return -1;
}

int WarehouseGraph::translateNodeId(int nodeId)
{
    if (nodeIdTranslationTable.count(nodeId) > 0)
    {
        return nodeIdTranslationTable[nodeId];
    }
    return -1;
}

int WarehouseGraph::translateOriNodeId(int nodeId)
{
    if (orientedNodeTranslationTable.count(nodeId) > 0)
    {
        return orientedNodeTranslationTable[nodeId];
    }
    return -1;
}