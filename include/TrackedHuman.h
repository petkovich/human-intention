#include <vector>
//#include "Eigen/Dense"
#include "Point.h"

struct TrackedHuman {
  int id;
  double d;
  int d_count;
  int number = 0;
  int counter = 0;
  int actual_current_node;
  int current_node;
  bool initialized = false;
  double X;
  double Y;
  double f_x;
  double f_y;
  double lastX;
  double lastY;
  Point<float> humanPosition, current_position, next_position;
  std::vector<int> plan;
  std::vector<int> node_history;
  Eigen::VectorXd tmp_obs;
  Eigen::MatrixXd observation_history;
  Eigen::MatrixXd I;
  int tick;
};