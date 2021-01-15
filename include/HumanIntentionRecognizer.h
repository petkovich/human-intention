#ifndef HUMANINTENTIONRECOGNIZER_H
#define HUMANINTENTIONRECOGNIZER_H

//#include <HumanIntentionRecognition.h>
#include <HIRutils.h>
#include "TrackedHuman.h"

#include <WarehouseGraph.h>  // libMapUtils

class HumanIntentionRecognizer {
 private:
  int realgoalnum = 4;
  int ticks = 3;
  double th_main = 4;
  double th_other = 0.8;
  double distance_th = 0.3;

  std::map<int, int> index_map;
  std::vector<Point<float>> robots;
  bool initial = true, initialPlan = true;
  // int current_node = 0;
  std::vector<int> idVector;
  // Point<float> humanPosition, current_position, next_position;
  std::vector<Node> graph_nodes, original_nodes;
  std::vector<Edge> graph_edges, original_edges;
  imr::dijkstra::CDijkstraLite<double> dijkstra;
  // std::vector<int> plan;
  std::map<int, TrackedHuman> humans;
  Eigen::MatrixXd F;
  std::vector<Node> s_nodes;
  std::clock_t end, begin;
  Eigen::VectorXd association_vector;
  // Eigen::MatrixXd I;  // I could be part of HIR class
  int goal_num = 4;
  int ok = 1;
  std::vector<std::vector<imr::dijkstra::CDijkstraLite<double>::Weight>>
      weight_vector;
  std::vector<int> trouble_humans;
  std::vector<int> goal_id;
  // Eigen::VectorXd tmp_obs;  //Will go away later
  // Eigen::MatrixXd observation_history;

  double h_x, h_y, l_hx, l_hy, d, alt_x, alt_y;
  int counter = 0;

 public:
  HumanIntentionRecognizer();

  void SetWarehouseMap(WarehouseGraph aMap);

  void UpdatePositionsAndTimestamp(
      std::vector<std::pair<int, Point<float>>> aPositions, double aTimestamp);
  void UpdatePlans(int id, std::vector<int> aPlan);
  std::vector<std::pair<int, std::vector<std::vector<int>>>>
  GetIntentionPrediction();
  bool IsAlarm();
  int GetTrackedHumanID();

  std::vector<std::pair<int, std::vector<std::vector<int>>>> DummyPredictor();
  // Point GetMostProbableTarget();
};

#endif  // HUMANINTENTIONRECOGNIZER_H
