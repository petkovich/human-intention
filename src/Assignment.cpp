#include <Assignment.h>
#include <Enums.h>
#include <vector>
#include <unordered_set>
#include <WarehouseGraph.h>
#include <Utilities.h>


// generate one assignment
Assignment Assignments::genRandomAssignment(Enums::AgentType type, int id, WarehouseGraph &g, std::pair<int, int> startRange, std::unordered_set<int> &closedNodes)
{
    int start, goal;
    do
    {
        start = Util::getRandNum(0, g.getNumNodes() - 1);
        goal = Util::getRandNum(0, g.getNumNodes() - 1);
    } while (closedNodes.count(start) > 0 || closedNodes.count(goal) > 0 || start == goal);

    //int startTime = Util::getRandNum(startRange.first, startRange.second);
    int startTime = 0;

    int startOriNode = Util::getRandNum(0,(int)g.getNodes()[start].getOrientedNodes().size()-1);
    double startAngle = g.getOriNodes()[g.getNodes()[start].getOrientedNodes()[startOriNode]].getOrientation();

    Planner::Agent agent(type,id,Util::getRandNum<double>(3,3),30);

    return Assignment(agent, g.getNodes()[start].getId(), g.getNodes()[goal].getId(), startTime,startAngle);
}

// generate fixed number of assignments
std::vector<Assignment> Assignments::generateAssignments(Enums::AgentType type, WarehouseGraph &g, std::pair<int, int> startRange, int numAgents)
{
    std::vector<Assignment> assignments;

    std::unordered_set<int> closed;
    for (int i = 0; i < numAgents; i++)
    {
        Assignment a = genRandomAssignment(type,i, g, startRange, closed);
        closed.emplace(a.assignment.first);
        closed.emplace(a.assignment.second);
        assignments.push_back(a);
    }
    return assignments;
}