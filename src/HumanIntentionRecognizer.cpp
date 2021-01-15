#include "HumanIntentionRecognizer.h"

HumanIntentionRecognizer::HumanIntentionRecognizer() {}

void HumanIntentionRecognizer::SetWarehouseMap(WarehouseGraph aMap) {
  graph_nodes = aMap.getNodes();
  graph_edges = aMap.getEdges();
  hirutils::prune_nodes(graph_nodes, s_nodes, index_map, F);
  hirutils::prune_edges(graph_edges, index_map);
  // hirutils::add_edges_manually(
  //    graph_edges);  // TODO: fix edges on the edge of the map
  original_nodes = graph_nodes;
  original_edges = graph_edges;
  hirutils::import_edges(dijkstra, graph_edges, graph_nodes);
  begin = std::clock();
  hirutils::update_dijkstra_paths(graph_nodes, dijkstra, index_map,
                                  weight_vector);
  hirutils::solve_dijkstra(graph_nodes, dijkstra, index_map, F);

  end = std::clock();
  /*std::cout << "HumanIntentionRecognizer::SetWarehouseMap: SOLVED DIJSKTRA in
     "
                        << double(end - begin) / CLOCKS_PER_SEC << std::endl;*/
  association_vector.resize(graph_nodes.size());

  // R34, R236, R212

  //goal_id.push_back(72);  // WHATCH OUT
  //goal_id.push_back(528);
  //goal_id.push_back(465);
  //goal_id.push_back(90);

  goal_id.push_back(470);  // WHATCH OUT
  goal_id.push_back(512);
  goal_id.push_back(528);
  goal_id.push_back(257);
  // tmp_obs.resize(goal_num);
  // observation_history.resize(1, goal_num);

  std::cout << "HumanIntentionRecognizer::SetWarehouseMap: Initialized Map \n";
  return;
}

void HumanIntentionRecognizer::UpdatePositionsAndTimestamp(
    std::vector<std::pair<int, Point<float>>> aPositions, double aTimestamp) {
  robots.clear();
  TrackedHuman human;
  for (int i = 0; i < aPositions.size(); i++) {
    if (aPositions[i].first >= 1000) {
      std::ofstream positions_file(
          "/home/tomislav/cvut_safelog/safelog_fleet_management_system/utils/"
          "HumanIntentionRecognition/data/results/positions_all.txt",
          std::ios::app);
      positions_file << std::fixed << std::setprecision(8)
                     << aPositions[i].second.getX() << ' '
                     << aPositions[i].second.getY() << std::endl;
      human = humans[aPositions[i].first];
      if (human.plan.empty()) {
        continue;
      }
      human.humanPosition = aPositions[i].second;
      human.f_x = human.humanPosition.getX();
      human.f_y = human.humanPosition.getY();
      // std::cout << "HumanIntentionRecognizer::UpdatePositions:"
      //           << "Position of human #" << human.id << " at " << aTimestamp
      //           << ": " << human.f_x << ' ' << human.f_y << '\n';
      if (!human.initialized) {
        /*std::cout
    << "HumanIntentionRecognizer::UpdatePositions: Initializing human#"
                << human.id << '\n';*/
        human.current_position = human.humanPosition;

        int start = graph_nodes[index_map[human.plan[0]]]
                        .getId();  // CARE: this does not change
        auto minx = graph_nodes[index_map[human.plan[0]]].getCoords().getX();
        auto miny = graph_nodes[index_map[human.plan[0]]].getCoords().getY();
        // find closest node
        for (int j = 0; j < graph_nodes.size(); j++) {
          auto node_x = graph_nodes[j].getCoords().getX();
          auto node_y = graph_nodes[j].getCoords().getY();
          if (sqrt(pow(human.humanPosition.getX() - node_x, 2) +
                   pow(human.humanPosition.getY() - node_y, 2)) <
              sqrt(pow(human.humanPosition.getX() - minx, 2) +
                   pow(human.humanPosition.getY() - miny, 2))) {
            start = graph_nodes[j].getId();
            minx = node_x;
            miny = node_y;
          }
        }
        human.actual_current_node = start;
        human.node_history.push_back(human.actual_current_node);
        // human.current_node=start;
        human.lastX = human.f_x;
        human.lastY = human.f_y;
        human.initialized = true;
      }
      humans[aPositions[i].first] = human;

    } else {
      Point<float> robot;
      robot.setX(aPositions[i].second.getX());
      robot.setY(aPositions[i].second.getY());
      // robots.push_back(robot);
    }
  }
  for (int i = 0; i < idVector.size(); i++) {
    human = humans[idVector[i]];
    if (human.plan.empty()) {
      continue;
    }
    bool enough_moved = false;
    d = sqrt(pow(human.f_x - human.lastX, 2) + pow(human.f_y - human.lastY, 2));
    human.d = d;
    // std::cout << f_x << ' ' << human.lastX << ' ' << f_y << ' ' <<
    // human.lastY
    //          << ' ' << d << '\n';
    if (d > 0.25) {
      human.d_count = 0;
      human.X = human.f_x;
      human.Y = human.f_y;
      enough_moved = true;
      human.counter++;
      // std::cout << "HumanIntentionRecognizer::UpdatePositions: human#"
      //           << human.id << " moved enough, step: " << human.counter << "
      //           "
      //           << human.f_x << ' ' << human.f_y << '\n';
      human.lastX = human.X;
      human.lastY = human.Y;

      // begin = std::clock();
      // hirutils::cut_edges(original_edges, graph_edges, index_map, robots,
      //                     graph_nodes);
      // hirutils::cut_nodes(original_nodes, graph_nodes, graph_edges, robots);
      // end = std::clock();
      // std::cout << "CUT EDGES AND NODES in " << double(end - begin) /
      // CLOCKS_PER_SEC
      //           << '\n';

      // dijkstra.clear();
      // hirutils::import_edges(dijkstra, graph_edges, graph_nodes);
      // std::cout << "IMPORTED EDGES" << std::endl;

      // begin = std::clock();
      // hirutils::solve_dijkstra(graph_nodes, dijkstra, index_map, F);
      // end = std::clock();
      // std::cout << "SOLVED DIJSKTRA in " << double(end - begin) /
      // CLOCKS_PER_SEC
      //           << std::endl;

      for (int j = 0; j < graph_edges.size(); j++) {
        auto startnode = graph_edges[j].getStartNodeId();
        if (startnode == human.actual_current_node) {
          auto endnode = graph_edges[j].getEndNodeId();
          if (sqrt(pow(human.humanPosition.getX() -
                           graph_nodes[index_map[endnode]].getCoords().getX(),
                       2) +
                   pow(human.humanPosition.getY() -
                           graph_nodes[index_map[endnode]].getCoords().getY(),
                       2)) < distance_th) {
            human.actual_current_node = endnode;
            human.node_history.push_back(human.actual_current_node);

            break;
          }
        }
      }

      // std::cout << "HumanIntentionRecognizer::UpdatePositions: Human #"
      //           << human.id << " is at node " << human.actual_current_node
      //           << '\n';
      hirutils::alternative_association(
          human.X, human.Y, human.actual_current_node, graph_edges, graph_nodes,
          association_vector, index_map);

      // hirutils::associate_position(human.X, human.Y, graph_nodes, s_nodes,
      //                              association_vector,
      //                              robots);  // real position

      auto current_observation = association_vector.transpose() * F * human.I;

      // std::cout << human.observation_history << '\n';
      // std::cout << current_observation << '\n';
      if (human.observation_history.rows() > 0) {
        human.observation_history.row(human.observation_history.rows() - 1) =
            current_observation;
      }

      if (human.counter > 10) {
        human.tmp_obs =
            human.observation_history.row(human.observation_history.rows() -
                                          10) -  // discard first 2
            current_observation;

        for (int goals = 0; goals < goal_num + 1; goals++) {
          if (human.tmp_obs(goals) <= 0) {
            human.tmp_obs(goals) = 0.001;
          }
        }
        human.tmp_obs /= human.tmp_obs.sum();
        /* std::cout << "HumanIntentionRecognizer::UpdatePositions:
           Probabilities " "of human #"
                               << human.id << ": " << human.tmp_obs.transpose()
           << '\n';*/
        // std::cout << "HumanIntentionRecognizer::UpdatePositions: Key:  R34, "
        //              "R236, R212 \n";

        std::ofstream positions_file(
            "/home/tomislav/cvut_safelog/safelog_fleet_management_system/utils/"
            "HumanIntentionRecognition/data/results/positions_inside.txt",
            std::ios::app);
        positions_file << std::fixed << std::setprecision(8) << human.X << ' '
                       << human.Y << std::endl;

        std::string obs_file =
            "/home/tomislav/cvut_safelog/safelog_fleet_management_system/utils/"
            "HumanIntentionRecognition/data/results/probabilities.csv";
        std::ifstream t(obs_file.c_str());
        std::string current((std::istreambuf_iterator<char>(t)),
                            std::istreambuf_iterator<char>());
        std::ofstream file(obs_file.c_str());
        file << current << human.tmp_obs.transpose().format(CSVFormat) << '\n';
      }

      human.observation_history.conservativeResize(
          human.observation_history.rows() + 1,
          human.observation_history.cols());

      humans[idVector[i]] = human;
    } else {
      human.d_count++;
      humans[idVector[i]] = human;
    }
  }
}

void HumanIntentionRecognizer::UpdatePlans(
    int id, std::vector<int> aPlan) {  // TODO: new plan handles

  std::cout << "HumanIntentionRecognizer::UpdatePlans: I got: ";
  for (int i = 0; i < aPlan.size(); i++) {
    std::cout << aPlan[i] << ' ';
  }
  std::cout << '\n';
  if (aPlan.empty()) {
    return;
  }
  if (std::find(idVector.begin(), idVector.end(), id) == idVector.end()) {
    /*std::cout << "HumanIntentionRecognizer::UpdatePlans: Start initialization
       " "of human #"
                      << id << '\n';*/
    TrackedHuman human;
    human.id = id;
    human.plan = aPlan;
    human.next_position.setX(
        graph_nodes[index_map[aPlan[0]]].getCoords().getX());
    human.next_position.setY(
        graph_nodes[index_map[aPlan[0]]].getCoords().getY());
    human.current_node = 0;
    human.observation_history.resize(1, goal_num + 1);
    human.tmp_obs.resize(goal_num + 1);
    human.I.resize(graph_nodes.size(), goal_num + 1);
    human.I *= 0;
    human.I(index_map[72], 0) = 1;
    human.I(index_map[528], 1) = 1;
    human.I(index_map[465], 2) = 1;
    human.I(index_map[90], 3) = 1;
    human.I(index_map[aPlan.back()], goal_num) = 1;

    human.tick = ticks;
    humans[id] = human;

    idVector.push_back(id);
    /*std::cout << "HumanIntentionRecognizer::UpdatePlans: Initialized human #"
          << id << '\n';
std::cout << "HumanIntentionRecognizer::UpdatePlans: Human plan: ";
for (int i = 0; i < human.plan.size(); i++) {
  std::cout << human.plan[i] << ' ';
}
    std::cout << '\n';*/
    return;
  }
  // else
  ok = 0;
  if (ok == 1) {
    if (aPlan != humans[id].plan) {
      /*std::cout << "HumanIntentionRecognizer::UpdatePlans:" << id << ' '
                            << humans[id].id << '\n';*/

      humans[id].I(index_map[humans[id].plan.back()], goal_num) = 0;
      humans[id].plan.insert(humans[id].plan.end(), aPlan.begin(), aPlan.end());
      humans[id].observation_history.resize(1, goal_num + 1);
      humans[id].counter = 0;
      // humans[id].next_position.setX(
      //     graph_nodes[index_map[aPlan[0]]].getCoords().getX());
      // humans[id].next_position.setY(
      //     graph_nodes[index_map[aPlan[0]]].getCoords().getY());
      // humans[id].current_node = 0;

      // new, add goal
      humans[id].I(index_map[aPlan.back()], goal_num) = 1;
      /*std::cout
       << "HumanIntentionRecognizer::UpdatePlans: Updated plan for human #"
       << id << '\n';
       std::cout << "HumanIntentionRecognizer::UpdatePlans: Human plan: ";
   for (int i = 0; i < humans[id].plan.size(); i++) {
     std::cout << humans[id].plan[i] << ' ';
   }
       std::cout << '\n';*/
    }
  } else {
    //humans[id].observation_history.conservativeResize(0, goal_num + 1);
    humans[id].counter = 0; //DELETED OBSH
    humans[id].plan = aPlan;
    // humans[id].number = -1;
    humans[id].current_node = 0;
    // humans[id].tmp_obs *= 0;                                // NEW
    humans[id].tmp_obs(humans[id].tmp_obs.size() - 1) = 0;
    humans[id].tmp_obs /= humans[id].tmp_obs.sum();  // NEW
    humans[id].next_position.setX(
        graph_nodes[index_map[aPlan[0]]].getCoords().getX());
    humans[id].next_position.setY(
        graph_nodes[index_map[aPlan[0]]].getCoords().getY());
    ok = 1;
    /*std::cout
    << "HumanIntentionRecognizer::UpdatePlans: Updated plan for human #"
    << id << '\n';
std::cout << "HumanIntentionRecognizer::UpdatePlans: Human plan: ";
for (int i = 0; i < humans[id].plan.size(); i++) {
  std::cout << humans[id].plan[i] << ' ';
}
    std::cout << '\n';*/
  }
}

bool HumanIntentionRecognizer::IsAlarm() {
  trouble_humans.clear();
  bool isflag = false;
  for (int i = 0; i < idVector.size(); i++) {
    TrackedHuman human = humans[idVector[i]];

    if (human.plan.empty() || !human.initialized) {
      continue;
    }
    if (sqrt(pow(human.humanPosition.getX() - human.next_position.getX(), 2) +
             pow(human.humanPosition.getY() - human.next_position.getY(), 2)) <
        distance_th) {
      human.current_position.setX(
          graph_nodes[index_map[human.plan[human.current_node]]]
              .getCoords()
              .getX());
      human.current_position.setY(
          graph_nodes[index_map[human.plan[human.current_node]]]
              .getCoords()
              .getY());
      human.current_node++;
      if (human.current_node >= human.plan.size()) {  // Might be a problem
        continue;
      }
      human.next_position.setX(
          graph_nodes[index_map[human.plan[human.current_node]]]
              .getCoords()
              .getX());
      human.next_position.setY(
          graph_nodes[index_map[human.plan[human.current_node]]]
              .getCoords()
              .getY());
    }
    // std::cout << human.current_node << ' '
    //           << graph_nodes[index_map[human.plan[human.current_node]]].getId()
    //           << '\n';
    // return false;

    //  std::cout << "Human Position: " << human.humanPosition.getX() << ' '
    //            << human.humanPosition.getY() << ' '
    //            << "First Node: " << human.current_position.getX() << ' '
    //            << human.current_position.getY() << ' '
    //            << "Next Node: " << human.next_position.getX() << ' '
    //            << human.next_position.getY() << '\n';
    double distance_from = sqrt(
        pow(human.humanPosition.getX() - human.current_position.getX(), 2) +
        pow(human.humanPosition.getY() - human.current_position.getY(), 2));
    double distance_to =
        sqrt(pow(human.humanPosition.getX() - human.next_position.getX(), 2) +
             pow(human.humanPosition.getY() - human.next_position.getY(), 2));
    double node_distance = sqrt(
        pow(human.next_position.getX() - human.current_position.getX(), 2) +
        pow(human.next_position.getY() - human.current_position.getY(), 2));

    if (distance_from + distance_to > node_distance + distance_th * 3) {
      human.tick--;
      // std::cout << human.tick << '\n';
      humans[idVector[i]] = human;

      if (human.tick <= 0) {
        human.tick = ticks;
        humans[idVector[i]] = human;
        trouble_humans.push_back(human.id);
        // std::cout << "HumanIntentionRecognizer::IsAlarm: plan is: ";
        // for (int i = 0; i < human.plan.size(); i++) {
        //   std::cout << human.plan[i] << ' ';
        // }
        // std::cout << "HumanIntentionRecognizer::IsAlarm:Current node: "
        //           << human.current_node << '\n';
        /*std::cout << "HumanIntentionRecognizer::IsAlarm: ALARM! for human #"
          << human.id << '\n';
std::cout << "HumanIntentionRecognizer::IsAlarm: Probabilities "
             "of human #"
                          << human.id << ": " << human.tmp_obs.transpose() <<
'\n';*/
        // std::cout << "HumanIntentionRecognizer::IsAlarm: Key:  R34, "
        //              "R236, R212 \n";
        // std::cout << "Human Position: " << humanPosition.getX() << ' '
        //           << humanPosition.getY() << ' '
        //           << "First Node: " << current_position.getX() << ' '
        //           << current_position.getY() << ' '
        //           << "Next Node: " << next_position.getX() << ' '
        //           << next_position.getY() << '\n';
        // std::cout << "Probabilities: " << tmp_obs.transpose() << '\n';

        isflag = true;
      }
    } else {
      human.tick = ticks;
      humans[idVector[i]] = human;
    }
  }

  if (isflag) {
    ok = 0;
  }
  return isflag;
}

int HumanIntentionRecognizer::GetTrackedHumanID() { return 1000; }

typedef struct probabilities_paths_ {
  double probability;
  std::vector<int> path;
} probabilities_paths;

bool compare_probpaths(probabilities_paths a, probabilities_paths b) {
  if (a.probability < b.probability) {
    return 0;
  } else {
    return 1;
  }
}

std::vector<std::pair<int, std::vector<std::vector<int>>>>
HumanIntentionRecognizer::GetIntentionPrediction() {
  std::vector<std::pair<int, std::vector<std::vector<int>>>> return_vector;
  std::vector<long int> path;
  std::vector<int> int_path;

  for (int i = 0; i < trouble_humans.size(); i++) {
    // std::cout << "GetIntentionPrediction 2" << i << '\n';

    auto human = humans[trouble_humans[i]];
    if (human.d_count > 10) {
      std::pair<int, std::vector<std::vector<int>>> return_pair;
      std::vector<std::vector<int>> human_position_vector_vector;
      std::vector<int> human_position_vector;
      human_position_vector.push_back(human.actual_current_node);
      human_position_vector_vector.push_back(human_position_vector);
      return_pair.second = human_position_vector_vector;
      return_pair.first = trouble_humans[i];
      return_vector.push_back(return_pair);
    } else {
      if ((human.tmp_obs.maxCoeff() - human.tmp_obs(goal_num) <
           1 / (th_main * goal_num))) {  // was 1. paper
        //&&(human.number == 0))
        std::pair<int, std::vector<std::vector<int>>> return_pair;
        return_pair.first = trouble_humans[i];
        // return_pair.second.push_back(human.plan);
        return_vector.push_back(return_pair);
        /*std::cout << "HumanIntentionRecognizer::GetIntentionPrediction:  "
                 "Original plan OK for human #"
                              << human.id << '\n';*/
      } else {
        int start = graph_nodes[index_map[human.plan[human.current_node]]]
                        .getId();  // CARE: this does not change
        auto minx = graph_nodes[index_map[human.plan[human.current_node]]]
                        .getCoords()
                        .getX();
        auto miny = graph_nodes[index_map[human.plan[human.current_node]]]
                        .getCoords()
                        .getY();
        // find closest node
        for (int j = 0; j < graph_nodes.size(); j++) {
          auto node_x = graph_nodes[j].getCoords().getX();
          auto node_y = graph_nodes[j].getCoords().getY();
          if (sqrt(pow(human.humanPosition.getX() - node_x, 2) +
                   pow(human.humanPosition.getY() - node_y, 2)) <
              sqrt(pow(human.humanPosition.getX() - minx, 2) +
                   pow(human.humanPosition.getY() - miny, 2))) {
            start = graph_nodes[j].getId();
            minx = node_x;
            miny = node_y;
          }
        }
        std::pair<int, std::vector<std::vector<int>>> return_pair;
        return_pair.first = trouble_humans[i];

        probabilities_paths array[goal_num];
        int k = 0;
        for (int j = 0; j < goal_num; j++) {
          if (human.tmp_obs(j) >
              th_other / (goal_num + human.number)) {  // TODO: care bout this
            int end = goal_id[j];                      //
            // std::cout << "HumanIntentionRecognizer::GetIntentionPrediction: "
            //          << start << ' ' << end << '\n';

            // begin = std::clock();
            dijkstra.solve(start, end, weight_vector[index_map[start]], path);
            // end = std::clock();
            // std::cout << "HumanIntentionRecognizer::GetIntentionPrediction:
            // SOLVED
            // "
            //             "DIJSKTRA in "
            //          << double(end - begin) / CLOCKS_PER_SEC << std::endl;
            // std::cout << "HumanIntentionRecognizer::GetIntentionPrediction:
            // ";

            int_path.clear();

            for (int k = 0; k < path.size(); k++) {
              //   std::cout << path[j] << ' ';
              int_path.push_back(path[k]);  // CARE: don't delete
              if (path[k] == end) {         // DON'T GO AROUND
                break;
              }
            }
            array[k].probability = human.tmp_obs(j);
            array[k].path = int_path;
            k++;
            // std::cout << '\n';

            // for (int i = 0; i < goal_num; i++) {
            //  std::pair<int, double> element;
            //  element.first = goal_id[i];
            //  element.second = tmp_obs(i);
            //  return_vector.push_back(element);
            //}

            // return_pair.second.push_back(int_path);
          }
        }

        std::sort(array, array + k, compare_probpaths);
        for (int j = 0; j < k; j++) {
          return_pair.second.push_back(array[j].path);
        }
        return_vector.push_back(return_pair);
      }
    }
  }
  /*for (int i = 0; i < return_vector.size(); i++) {
    auto rtrn_element = return_vector[i];
        std::cout << "HumanIntentionRecognizer::GetIntentionPrediction: Human #"
              << rtrn_element.first << "'s plan: \n";
    for (int j = 0; j < rtrn_element.second.size(); j++) {
      for (int k = 0; k < rtrn_element.second[j].size(); k++) {
        std::cout << rtrn_element.second[j][k] << ' ';
      }
      std::cout << '\n';
        }
  }*/
  // auto dummyboy = DummyPredictor();
  return return_vector;
}

std::vector<std::pair<int, std::vector<std::vector<int>>>>
HumanIntentionRecognizer::DummyPredictor() {
  std::vector<std::pair<int, std::vector<std::vector<int>>>> return_vector;
  for (int i = 0; i < trouble_humans.size(); i++) {
    std::vector<int> path;
    auto human = humans[trouble_humans[i]];
    auto x1 = graph_nodes
                  [index_map[human.node_history[human.node_history.size() - 2]]]
                      .getCoords()
                      .getX();
    auto y1 = graph_nodes
                  [index_map[human.node_history[human.node_history.size() - 2]]]
                      .getCoords()
                      .getY();
    auto x2 = graph_nodes
                  [index_map[human.node_history[human.node_history.size() - 1]]]
                      .getCoords()
                      .getX();
    auto y2 = graph_nodes
                  [index_map[human.node_history[human.node_history.size() - 1]]]
                      .getCoords()
                      .getY();
    auto heading = atan2(y2 - y1, x2 - x1);

    double stop_condition = 0;
    bool added = true;
    path.push_back(human.node_history[human.node_history.size() - 1]);
    auto lastnode = human.node_history[human.node_history.size() - 1];

    while (added) {
      added = false;

      for (int j = 0; j < graph_edges.size(); j++) {
        auto startnode = graph_edges[j].getStartNodeId();
        if (startnode == lastnode) {
          auto endnode = graph_edges[j].getEndNodeId();
          x2 = graph_nodes[index_map[endnode]].getCoords().getX();
          y2 = graph_nodes[index_map[endnode]].getCoords().getY();
          auto second_heading = atan2(y2 - y1, x2 - x1);
          stop_condition = M_PI_2 - abs(abs(heading - second_heading) - M_PI_2);
          if (stop_condition < M_PI / 6) {
            bool do_add = true;
            for (int cntr = 0; cntr < path.size(); cntr++) {
              if (path[cntr] == endnode) {
                do_add = false;
                break;
              }
            }
            if (do_add) {
              path.push_back(endnode);
              // std::cout << endnode << ' ';
              heading = atan2(y2 - y1, x2 - x1);
              x1 = x2;
              y1 = y2;

              added = true;
              lastnode = endnode;
            }
          }
        }
      }
    }
    // std::cout << '\n';
    std::pair<int, std::vector<std::vector<int>>> return_pair;
    return_pair.second.push_back(path);
    return_pair.first = trouble_humans[i];
    return_vector.push_back(return_pair);
  }

  return return_vector;
}
