#include <ResourceGraph.h>
#include <WarehouseGraph.h>
#include <Resource.h>
#include <EdgeResource.h>
#include <OriNodeResource.h>
#include <RotationEdgeResource.h>
#include <OrientedNode.h>
#include <Edge.h>
#include <Node.h>
#include <Utilities.h>
#include <Enums.h>
#include <algorithm>
#include <fstream>
#include <Point.h>
#include <unordered_set>

std::vector<std::vector<int>> getOriNodeAdjacency(WarehouseGraph &map, Node &node)
{
    std::vector<std::vector<int>> localAdjacency(node.getOrientedNodes().size());
    std::vector<std::pair<int, double>> sorted;
    for (int i = 0; i < node.getOrientedNodes().size(); i++)
    {
        sorted.push_back(std::make_pair(i, map.getOriNodes()[node.getOrientedNodes()[i]].getOrientation()));
    }
    std::sort(sorted.begin(), sorted.end(), [](const std::pair<int, double> &a, const std::pair<int, double> &b) { return a.second < b.second; });
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
                localAdjacency[sorted[0].first].push_back(sorted[1].first);
            }
            // take right and left
            else
            {
                localAdjacency[sorted[0].first].push_back(sorted[1].first);
                localAdjacency[sorted[0].first].push_back(sorted[sorted.size() - 1].first);
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
                localAdjacency[sorted[1].first].push_back(sorted[0].first);
            }
            else
            {
                localAdjacency[sorted[j].first].push_back(sorted[0].first);
                localAdjacency[sorted[j].first].push_back(sorted[j - 1].first);
            }
        }
        else
        {
            localAdjacency[sorted[j].first].push_back(sorted[j + 1].first);
            localAdjacency[sorted[j].first].push_back(sorted[j - 1].first);
        }
    }
    return localAdjacency;
}

std::vector<std::vector<int>> reverseAdjacency(std::vector<std::vector<int>> &adjacency)
{
    std::vector<std::vector<int>> reversed(adjacency.size());
    for (int i = 0; i < adjacency.size(); i++)
    {
        for (int j = 0; j < adjacency[i].size(); j++)
        {
            reversed[adjacency[i][j]].push_back(i);
        }
    }
    return reversed;
}

int ResourceGraph::getOrientedNodeWithProperOrientation(int nodeId, double orientation)
{
    //Node &node = map.getNodes()[map.translateNodeId(nodeId)];

    for (int &oriNodeIndex : associatedOriNodeResourcesToNodeId[nodeId])
    {
        OriNodeResource *oriNode = dynamic_cast<OriNodeResource *>(resources[oriNodeIndex].get());
        if (oriNode->getOrientation() == orientation)
        {
            return oriNodeIndex;
        }
    }
    //std::cout << "No oriented node with orientation " << orientation << " degrees found on node with id " << nodeId << "." << std::endl;
    return -1;
}

double getEdgeLength(WarehouseGraph &map, Edge &e)
{
    Point<float> startNode = map.getNodes()[map.translateNodeId(e.getStartNodeId())].getCoords();
    Point<float> endNode = map.getNodes()[map.translateNodeId(e.getEndNodeId())].getCoords();
    // if points share at least one coordinate, they are on line
    double edgeLength;
    if (startNode.getX() == endNode.getX() || startNode.getY() == endNode.getY())
    {
        edgeLength = Util::euclidDistance(startNode, endNode);
    }
    // else there is spline - estimate its length as 1/4 of elipse
    else
    {
        double a = std::abs(startNode.getX() - endNode.getX());
        double b = std::abs(startNode.getY() - endNode.getY());
        edgeLength = 3.1415 * (3 * (a + b) - std::sqrt((3 * a + b) * (a + 3 * b)));
        edgeLength = edgeLength / 4;
    }
    return edgeLength;
}

Point<float> getEdgeCoords(WarehouseGraph &map, Edge &e)
{
    Point<float> startNode = map.getNodes()[map.translateNodeId(e.getStartNodeId())].getCoords();
    Point<float> endNode = map.getNodes()[map.translateNodeId(e.getEndNodeId())].getCoords();

    return Point<float>((startNode.getX() + endNode.getX()) / 2, (startNode.getY() + endNode.getY()) / 2);
}

void ResourceGraph::createQueueResourceGraph(WarehouseGraph &map, std::unordered_set<int> &nodeIdsToUse)
{
    // add orientedNodes to the graph
    // except for the queue nodes

    std::unordered_map<int, int> oriNodeIdTranslationTable;
    std::unordered_map<int, std::vector<int>> associatedRotationResources;
    for (Node &node : map.getNodes())
    {
        // skip nodes that are not to be used
        if (nodeIdsToUse.count(node.getId()) == 0)
            continue;

        // push all ori nodes
        if (node.getOrientedNodes().size() > 0)
            associatedOriNodeResourcesToNodeId.emplace(std::make_pair(node.getId(), std::vector<int>()));
        for (int &oriIndex : node.getOrientedNodes())
        {
            OrientedNode &oriNode = map.getOriNodes()[oriIndex];
            oriNodeIdTranslationTable.emplace(std::make_pair(oriNode.getId(), resources.size()));

            Enums::ResourceType oriType = map.getNodes()[oriNode.getNodeIndex()].getNodeType();
            associatedOriNodeResourcesToNodeId[node.getId()].push_back(resources.size());
            nodeIdAssociatedToOriNodeResources[oriNode.getId()] = node.getId();
            resources.push_back(std::make_unique<OriNodeResource>(oriNode.getId(), resources.size(), node.getCoords(), oriType, oriNode.getOrientation()));
            adjacencyList.push_back(std::vector<int>());
        }
        std::vector<std::vector<int>> localAdjacency = getOriNodeAdjacency(map, node);

        // create connections to rotation nodes
        int firstIndex = resources.size() - node.getOrientedNodes().size();
        for (int i = 0; i < localAdjacency.size(); i++)
        {
            for (int j = 0; j < localAdjacency[i].size(); j++)
            {
                int startIndex = firstIndex + i;
                OrientedNode &startOriNode = map.getOriNodes()[node.getOrientedNodes()[i]];
                int endIndex = firstIndex + localAdjacency[i][j];
                OrientedNode &endOriNode = map.getOriNodes()[node.getOrientedNodes()[localAdjacency[i][j]]];
                int rotIndex = resources.size();
                double rotAngle = std::abs(endOriNode.getOrientation() - startOriNode.getOrientation());
                // if robot would rotate more than 180 degrees, make him rotate the other way
                if (rotAngle > 180)
                {
                    rotAngle = 360 - std::max(endOriNode.getOrientation(), startOriNode.getOrientation());
                }
                // create the rotation resource
                resources.push_back(std::make_unique<RotationEdgeResource>(-1, rotIndex, node.getCoords(), Enums::ResourceType::Rotation, rotAngle));
                adjacencyList.push_back(std::vector<int>());
                // create mark to associate rotation resource with its start and end oriented nodes
                if (associatedRotationResources.count(startIndex) == 0)
                {
                    associatedRotationResources.emplace(std::make_pair(startIndex, std::vector<int>()));
                }
                associatedRotationResources[startIndex].push_back(rotIndex);
                if (associatedRotationResources.count(endIndex) == 0)
                {
                    associatedRotationResources.emplace(std::make_pair(endIndex, std::vector<int>()));
                }
                associatedRotationResources[endIndex].push_back(rotIndex);
                // add connections to adjacency list
                adjacencyList[startIndex].push_back(rotIndex);
                adjacencyList[rotIndex].push_back(startIndex);
                adjacencyList[endIndex].push_back(rotIndex);
                adjacencyList[rotIndex].push_back(endIndex);
            }
        }
    }

    // add edges to the graph
    std::unordered_map<int, int> edgeIdTranslationTable;
    for (Edge &e : map.getEdges())
    {
        int startOriNodeResIndex = getOrientedNodeWithProperOrientation(e.getStartNodeId(), e.getStartAngle());
        int endOriNodeResIndex = getOrientedNodeWithProperOrientation(e.getEndNodeId(), e.getEndAngle());

        // skip all edges leading to or from nodes that are not in the graph
        if (startOriNodeResIndex == -1 || endOriNodeResIndex == -1)
        {
            continue;
        }

        double edgeLength = getEdgeLength(map, e);
        int edgeIndex = resources.size();
        // create edge resource
        // get coords in the middle of the edge
        Point<float> edgeCoords = getEdgeCoords(map, e);
        resources.push_back(std::make_unique<EdgeResource>(e.getId(), edgeIndex, edgeCoords, Enums::ResourceType::EdgeResource, edgeLength, e.getMaxSpeedLoaded(), e.getMaxSpeedUnloaded()));
        adjacencyList.push_back(std::vector<int>());
        edgeIdTranslationTable.emplace(std::make_pair(e.getId(), edgeIndex));
        // mark edges into adjacency list
        int startIndex = oriNodeIdTranslationTable[resources[startOriNodeResIndex]->getId()];
        int endIndex = oriNodeIdTranslationTable[resources[endOriNodeResIndex]->getId()];
        adjacencyList[startIndex].push_back(edgeIndex);
        adjacencyList[edgeIndex].push_back(endIndex);
    }

    // fill in conflicts
    // fill in node conflicts
    for (OrientedNode &oriNode : map.getOriNodes())
    {
        if (oriNodeIdTranslationTable.count(oriNode.getId()) == 0)
        {
            continue;
        }
        int oriNodeIndex = oriNodeIdTranslationTable[oriNode.getId()];
        if (associatedRotationResources.count(oriNodeIndex) == 0)
        {
            // std::cout << "Associated resources not found" << std::endl;
        }
        std::vector<int> associatedRotNodes = associatedRotationResources[oriNodeIndex];

        for (Conflict &c : oriNode.getConflicts())
        {
            if (c.getConflictType() == Enums::ConflictType::Edge)
            {
                if (edgeIdTranslationTable.count(c.getResourceId()) == 0)
                    continue;
                int edgeIndex = edgeIdTranslationTable[c.getResourceId()];
                resources[oriNodeIndex]->addOtherConflictResource(edgeIndex);
                for (int &i : associatedRotNodes)
                {
                    resources[i]->addOtherConflictResource(edgeIndex);
                }
            }
            else
            {
                if (oriNodeIdTranslationTable.count(c.getResourceId()) == 0)
                    continue;
                int cOriNodeIndex = oriNodeIdTranslationTable[c.getResourceId()];

                // if original ori node and conflicting one are on the same node, then add it as physical conflict
                int origNodeIndexOne = map.getOriNodes()[map.translateOriNodeId(oriNode.getId())].getNodeIndex();
                int origNodeIndexTwo = map.getOriNodes()[map.translateOriNodeId(c.getResourceId())].getNodeIndex();
                if (origNodeIndexOne == origNodeIndexTwo)
                {
                    resources[oriNodeIndex]->addPhysicalConflictResource(cOriNodeIndex);
                    for (int &i : associatedRotNodes)
                    {
                        resources[i]->addPhysicalConflictResource(cOriNodeIndex);
                    }
                }
                // else as other conflict
                else
                {
                    resources[oriNodeIndex]->addOtherConflictResource(cOriNodeIndex);
                    for (int &i : associatedRotNodes)
                    {
                        resources[i]->addOtherConflictResource(cOriNodeIndex);
                    }
                }
            }
        }
    }
    // fill in edge conflicts
    for (Edge &e : map.getEdges())
    {
        // skip edges not in graph
        if (edgeIdTranslationTable.count(e.getId()) == 0)
            continue;
        int edgeIndex = edgeIdTranslationTable[e.getId()];

        int eStartNodeIndex = e.getStartNodeId();
        int eEndNodeIndex = e.getEndNodeId();

        for (Conflict &c : e.getConflicts())
        {
            if (c.getConflictType() == Enums::ConflictType::Edge)
            {
                // skip conflict edges that are not in graph
                if (edgeIdTranslationTable.count(c.getResourceId()) == 0)
                    continue;
                int cEdgeIndex = edgeIdTranslationTable[c.getResourceId()];
                Edge &cEdge = map.getEdges()[map.translateEdgeId(c.getResourceId())];
                // check if its the same edge just in opposite direction
                int cEStartNodeIndex = cEdge.getStartNodeId();
                int cEEndNodeIndex = cEdge.getEndNodeId();
                if (eStartNodeIndex == cEEndNodeIndex && eEndNodeIndex == cEStartNodeIndex)
                {
                    resources[edgeIndex]->addPhysicalConflictResource(cEdgeIndex);
                }
                else
                {
                    resources[edgeIndex]->addOtherConflictResource(cEdgeIndex);
                }
            }
            else
            {
                // skip nodes that are not in graph
                if (oriNodeIdTranslationTable.count(c.getResourceId()) == 0)
                    continue;
                int cOriNodeIndex = oriNodeIdTranslationTable[c.getResourceId()];
                std::vector<int> &associatedRotNodes = associatedRotationResources[cOriNodeIndex];
                resources[edgeIndex]->addOtherConflictResource(cOriNodeIndex);
                for (int &rotNode : associatedRotNodes)
                {
                    resources[edgeIndex]->addOtherConflictResource(rotNode);
                }
            }
        }
    }

    // add rotation nodes as physical conflicts
    bool found = false;
    for (std::pair<int, std::vector<int>> record : associatedRotationResources)
    {
        int currRes = record.first;
        for (int i : record.second)
        {
            resources[currRes]->addPhysicalConflictResource(i);
            resources[i]->addPhysicalConflictResource(currRes);
        }
    }
}

// build a resource graph from warehouse map
void ResourceGraph::createWarehouseResourceGraph(WarehouseGraph &map)
{
    // find edges that do not start in queue node, but end in queue node
    // these are the first nodes of queue
    std::unordered_set<int> firstQueueNodeIds;
    std::vector<int> pickStationIds;
    for (Edge &e : map.getEdges())
    {
        Node &start = map.getNodes()[map.translateNodeId(e.getStartNodeId())];
        Node &end = map.getNodes()[map.translateNodeId(e.getEndNodeId())];
        if (start.getNodeType() != Enums::ResourceType::Queue &&
            end.getNodeType() == Enums::ResourceType::Queue)
        {
            firstQueueNodeIds.emplace(end.getId());
        }
        if (end.getNodeType() == Enums::ResourceType::PickStation)
        {
            pickStationIds.push_back(end.getId());
        }
    }

    // associate each pick station with its queue start node
    for (int &pickStationId : pickStationIds)
    {
        Node &pickStation = map.getNodes()[map.translateNodeId(pickStationId)];
        double d = INFINITY;
        int bestId = -1;
        for (const int &queueNodeId : firstQueueNodeIds)
        {
            Node &queueNode = map.getNodes()[map.translateNodeId(queueNodeId)];
            double dist = Util::euclidDistance(pickStation.getCoords(), queueNode.getCoords());
            if (dist < d)
            {
                d = dist;
                bestId = queueNodeId;
            }
        }
        pickStationQueueStart.emplace(std::make_pair(pickStationId, bestId));
    }

    // add orientedNodes to the graph
    // except for the queue nodes
    std::unordered_map<int, int> oriNodeIdTranslationTable;
    std::unordered_map<int, std::vector<int>> associatedRotationResources;
    for (Node &node : map.getNodes())
    {
        if (node.getNodeType() == Enums::ResourceType::Queue && firstQueueNodeIds.count(node.getId()) == 0)
            continue;
        // push all ori nodes
        associatedOriNodeResourcesToNodeId.emplace(std::make_pair(node.getId(), std::vector<int>()));
        for (int &oriIndex : node.getOrientedNodes())
        {
            OrientedNode &oriNode = map.getOriNodes()[oriIndex];
            oriNodeIdTranslationTable.emplace(std::make_pair(oriNode.getId(), resources.size()));

            Enums::ResourceType oriType = map.getNodes()[oriNode.getNodeIndex()].getNodeType();
            associatedOriNodeResourcesToNodeId[node.getId()].push_back(resources.size());
            // make note from ori node id to node id
            nodeIdAssociatedToOriNodeResources[oriNode.getId()] = node.getId();
            resources.push_back(std::make_unique<OriNodeResource>(oriNode.getId(), resources.size(), node.getCoords(), oriType, oriNode.getOrientation()));
            adjacencyList.push_back(std::vector<int>());
        }
        std::vector<std::vector<int>> localAdjacency = getOriNodeAdjacency(map, node);

        // create connections to rotation nodes
        int firstIndex = resources.size() - node.getOrientedNodes().size();
        for (int i = 0; i < localAdjacency.size(); i++)
        {
            for (int j = 0; j < localAdjacency[i].size(); j++)
            {
                int startIndex = firstIndex + i;
                OrientedNode &startOriNode = map.getOriNodes()[node.getOrientedNodes()[i]];
                int endIndex = firstIndex + localAdjacency[i][j];
                OrientedNode &endOriNode = map.getOriNodes()[node.getOrientedNodes()[localAdjacency[i][j]]];
                int rotIndex = resources.size();
                double rotAngle = std::abs(endOriNode.getOrientation() - startOriNode.getOrientation());
                // if robot would rotate more than 180 degrees, make him rotate the other way
                if (rotAngle > 180)
                {
                    rotAngle = 360 - std::max(endOriNode.getOrientation(), startOriNode.getOrientation());
                }
                // create the rotation resource
                resources.push_back(std::make_unique<RotationEdgeResource>(-1, rotIndex, node.getCoords(), Enums::ResourceType::Rotation, rotAngle));
                adjacencyList.push_back(std::vector<int>());
                // create mark to associate rotation resource with its start and end oriented nodes
                if (associatedRotationResources.count(startIndex) == 0)
                {
                    associatedRotationResources.emplace(std::make_pair(startIndex, std::vector<int>()));
                }
                associatedRotationResources[startIndex].push_back(rotIndex);
                if (associatedRotationResources.count(endIndex) == 0)
                {
                    associatedRotationResources.emplace(std::make_pair(endIndex, std::vector<int>()));
                }
                associatedRotationResources[endIndex].push_back(rotIndex);
                // add connections to adjacency list
                adjacencyList[startIndex].push_back(rotIndex);
                adjacencyList[rotIndex].push_back(startIndex);
                adjacencyList[endIndex].push_back(rotIndex);
                adjacencyList[rotIndex].push_back(endIndex);
            }
        }
    }

    // add edges to the graph
    std::unordered_map<int, int> edgeIdTranslationTable;
    for (Edge &e : map.getEdges())
    {
        int startOriNodeResIndex = getOrientedNodeWithProperOrientation(e.getStartNodeId(), e.getStartAngle());
        int endOriNodeResIndex = getOrientedNodeWithProperOrientation(e.getEndNodeId(), e.getEndAngle());

        // skip all edges leading to or from nodes that are not in the graph
        if (startOriNodeResIndex == -1 || endOriNodeResIndex == -1)
        {
            continue;
        }

        std::unique_ptr<Resource> &startOriNode = resources[startOriNodeResIndex];
        std::unique_ptr<Resource> &endOriNode = resources[endOriNodeResIndex];

        double edgeLength = getEdgeLength(map, e);
        int edgeIndex = resources.size();
        // create edge resource
        // get coords in the middle of the edge
        Point<float> edgeCoords = getEdgeCoords(map, e);
        resources.push_back(std::make_unique<EdgeResource>(e.getId(), edgeIndex, edgeCoords, Enums::ResourceType::EdgeResource, edgeLength, e.getMaxSpeedLoaded(), e.getMaxSpeedUnloaded()));
        adjacencyList.push_back(std::vector<int>());
        edgeIdTranslationTable.emplace(std::make_pair(e.getId(), edgeIndex));
        // mark edges into adjacency list
        int startIndex = oriNodeIdTranslationTable[startOriNode->getId()];
        int endIndex = oriNodeIdTranslationTable[endOriNode->getId()];
        adjacencyList[startIndex].push_back(edgeIndex);
        adjacencyList[edgeIndex].push_back(endIndex);
    }

    // fill in conflicts
    // fill in node conflicts
    for (OrientedNode &oriNode : map.getOriNodes())
    {
        if (oriNodeIdTranslationTable.count(oriNode.getId()) == 0)
        {
            continue;
        }
        int oriNodeIndex = oriNodeIdTranslationTable[oriNode.getId()];

        if (associatedRotationResources.count(oriNodeIndex) == 0)
        {
            std::cout << "Associated resources not found" << std::endl;
        }
        std::vector<int> associatedRotNodes = associatedRotationResources[oriNodeIndex];

        for (Conflict &c : oriNode.getConflicts())
        {
            if (c.getConflictType() == Enums::ConflictType::Edge)
            {
                if (edgeIdTranslationTable.count(c.getResourceId()) == 0)
                    continue;
                int edgeIndex = edgeIdTranslationTable[c.getResourceId()];
                resources[oriNodeIndex]->addOtherConflictResource(edgeIndex);
                for (int &i : associatedRotNodes)
                {
                    resources[i]->addOtherConflictResource(edgeIndex);
                }
            }
            else
            {
                if (oriNodeIdTranslationTable.count(c.getResourceId()) == 0)
                    continue;
                int cOriNodeIndex = oriNodeIdTranslationTable[c.getResourceId()];

                // if original ori node and conflicting one are on the same node, then add it as physical conflict
                int origNodeIndexOne = map.getOriNodes()[map.translateOriNodeId(oriNode.getId())].getNodeIndex();
                int origNodeIndexTwo = map.getOriNodes()[map.translateOriNodeId(c.getResourceId())].getNodeIndex();
                if (origNodeIndexOne == origNodeIndexTwo)
                {
                    resources[oriNodeIndex]->addPhysicalConflictResource(cOriNodeIndex);
                    for (int &i : associatedRotNodes)
                    {
                        resources[i]->addPhysicalConflictResource(cOriNodeIndex);
                    }
                }
                // else as other conflict
                else
                {
                    resources[oriNodeIndex]->addOtherConflictResource(cOriNodeIndex);
                    for (int &i : associatedRotNodes)
                    {
                        resources[i]->addOtherConflictResource(cOriNodeIndex);
                    }
                }
            }
        }
    }

    // fill in edge conflicts
    for (Edge &e : map.getEdges())
    {
        // skip edges not in graph
        if (edgeIdTranslationTable.count(e.getId()) == 0)
            continue;
        int edgeIndex = edgeIdTranslationTable[e.getId()];

        int eStartNodeIndex = e.getStartNodeId();
        int eEndNodeIndex = e.getEndNodeId();

        std::unordered_set<int> lockedConflictResources;
        for (Conflict &c : e.getConflicts())
        {
            if (c.getConflictType() == Enums::ConflictType::Edge)
            {
                // skip conflict edges that are not in graph
                if (edgeIdTranslationTable.count(c.getResourceId()) == 0)
                    continue;
                int cEdgeIndex = edgeIdTranslationTable[c.getResourceId()];
                Edge &cEdge = map.getEdges()[map.translateEdgeId(c.getResourceId())];
                // check if its the same edge just in opposite direction
                int cEStartNodeIndex = cEdge.getStartNodeId();
                int cEEndNodeIndex = cEdge.getEndNodeId();
                if (eStartNodeIndex == cEEndNodeIndex && eEndNodeIndex == cEStartNodeIndex)
                {
                    resources[edgeIndex]->addPhysicalConflictResource(cEdgeIndex);
                }
                else
                {
                    resources[edgeIndex]->addOtherConflictResource(cEdgeIndex);
                }
            }
            else
            {
                // skip nodes that are not in graph
                if (oriNodeIdTranslationTable.count(c.getResourceId()) == 0)
                    continue;
                int cOriNodeIndex = oriNodeIdTranslationTable[c.getResourceId()];
                std::vector<int> &associatedRotNodes = associatedRotationResources[cOriNodeIndex];
                resources[edgeIndex]->addOtherConflictResource(cOriNodeIndex);
                for (int &rotNode : associatedRotNodes)
                {
                    if (lockedConflictResources.count(rotNode) == 0)
                    {
                        lockedConflictResources.emplace(rotNode);
                        resources[edgeIndex]->addOtherConflictResource(rotNode);
                    }
                }
            }
        }
    }

    // add rotation nodes as physical conflicts
    /*bool found = false;
    for (std::pair<int, std::vector<int>> record : associatedRotationResources)
    {
        int currRes = record.first;
        for (int i : record.second)
        {
            if (i == 3723 || currRes == 3723)
            {
                std::cout << "";
            }
            resources[currRes]->addPhysicalConflictResource(i);
            resources[i]->addPhysicalConflictResource(currRes);
        }
    }*/
}

void ResourceGraph::exportGraph(std::string nodeFile, std::string edgeFile, WarehouseGraph &map)
{
    std::ofstream nodes(nodeFile);
    std::ofstream edges(edgeFile);

    // export nodes
    for (int i = 0; i < resources.size(); i++)
    {
        std::unique_ptr<Resource> &res = resources[i];

        Point<float> coords;

        if (dynamic_cast<EdgeResource *>(res.get()))
        {
            EdgeResource *eRes = dynamic_cast<EdgeResource *>(res.get());
            Edge &edge = map.getEdges()[map.translateEdgeId(eRes->getId())];
            Point<float> oneSide = map.getNodes()[map.translateNodeId(edge.getStartNodeId())].getCoords();
            Point<float> twoSide = map.getNodes()[map.translateNodeId(edge.getEndNodeId())].getCoords();
            coords.setX((oneSide.getX() + twoSide.getX()) / 2);
            coords.setY((oneSide.getY() + twoSide.getY()) / 2);
        }
        else if (dynamic_cast<OriNodeResource *>(res.get()))
        {
            OriNodeResource *oriRes = dynamic_cast<OriNodeResource *>(res.get());
            coords = map.getNodes()[map.getOriNodes()[map.translateOriNodeId(oriRes->getId())].getNodeIndex()].getCoords();
            double shift = 0.2;
            switch ((int)oriRes->getOrientation())
            {
            case 0:
                coords.setX(coords.getX() + shift);
                break;
            case 90:
                coords.setY(coords.getY() + shift);
                break;
            case 180:
                coords.setX(coords.getX() - shift);
                break;
            case 270:
                coords.setY(coords.getY() - shift);
                break;
            }
        }
        else
        {
            RotationEdgeResource *rotRes = dynamic_cast<RotationEdgeResource *>(res.get());
            OriNodeResource *one = dynamic_cast<OriNodeResource *>(resources[adjacencyList[i][0]].get());
            OriNodeResource *two = dynamic_cast<OriNodeResource *>(resources[adjacencyList[i][1]].get());
            Point<float> nodeCoords = map.getNodes()[map.getOriNodes()[map.translateOriNodeId(one->getId())].getNodeIndex()].getCoords();
            Point<float> oneCoords = nodeCoords;
            Point<float> twoCoords = nodeCoords;
            double shift = 0.2;
            switch ((int)one->getOrientation())
            {
            case 0:
                oneCoords.setX(oneCoords.getX() + shift);
                break;
            case 90:
                oneCoords.setY(oneCoords.getY() + shift);
                break;
            case 180:
                oneCoords.setX(oneCoords.getX() - shift);
                break;
            case 270:
                oneCoords.setY(oneCoords.getY() - shift);
                break;
            }
            switch ((int)two->getOrientation())
            {
            case 0:
                twoCoords.setX(twoCoords.getX() + shift);
                break;
            case 90:
                twoCoords.setY(twoCoords.getY() + shift);
                break;
            case 180:
                twoCoords.setX(twoCoords.getX() - shift);
                break;
            case 270:
                twoCoords.setY(twoCoords.getY() - shift);
                break;
            }
            coords.setX((oneCoords.getX() + twoCoords.getX()) / 2);
            coords.setY((oneCoords.getY() + twoCoords.getY()) / 2);
        }
    }

    // export edges
    for (int i = 0; i < adjacencyList.size(); i++)
    {
        for (int j = 0; j < adjacencyList[i].size(); j++)
        {
            edges << i << " " << adjacencyList[i][j] << std::endl;
        }
    }
}

bool ResourceGraph::isConsistent()
{
    for (int i = 0; i < resources.size(); i++)
    {
        Resource *res = resources[i].get();

        if (dynamic_cast<EdgeResource *>(res))
        {
            if (adjacencyList[i].size() > 1)
            {
                std::cout << adjacencyList[i].size() << std::endl;
                return false;
            }
            for (int j = 0; j < adjacencyList[i].size(); j++)
            {
                if (dynamic_cast<OriNodeResource *>(resources[adjacencyList[i][j]].get()) == nullptr)
                {
                    return false;
                }
            }
        }
        else if (dynamic_cast<OriNodeResource *>(res))
        {
            for (int j = 0; j < adjacencyList[i].size(); j++)
            {
                if (dynamic_cast<OriNodeResource *>(resources[adjacencyList[i][j]].get()) != nullptr)
                {
                    return false;
                }
            }
        }
        else
        {
            if (adjacencyList[i].size() != 2)
            {
                std::cout << adjacencyList[i].size() << std::endl;
                return false;
            }
            for (int j = 0; j < adjacencyList[i].size(); j++)
            {
                if (dynamic_cast<OriNodeResource *>(resources[adjacencyList[i][j]].get()) == nullptr)
                {
                    return false;
                }
            }
        }
    }
    return true;
}