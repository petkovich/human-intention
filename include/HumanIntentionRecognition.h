#include <HIRutils.h>

class HumanIntentionRecognition {
  int goal_number;
  double alpha = 0.5;
  double beta = 0.1;
  double gamma = 0.05;
  double delta = 0.1;
  double fi = 0;
  double h_x, h_y, l_hx, l_hy, d, f_x, f_y, alt_x, alt_y;
  int possible_position_number = 8;
  std::vector<Point<float>> robots;
  std::vector<Node> nodes;
  std::vector<Edge> edges;
  int buffer_size = 30;
  Eigen::MatrixXd Transition_matrix;
  Eigen::RowVectorXd state_probability;
  Eigen::VectorXd observation;
  Eigen::VectorXd observation_history;
  Eigen::VectorXd emission_vector;

  Eigen::VectorXd association_vector;

  Eigen::MatrixXd I;

 public:
  HumanIntentionRecognition(int goal_number);
  void LoadObservation(Eigen::VectorXd observation,
                       Eigen::MatrixXd dummy_observations);
  void UpdateTransition();
  void UpdateProbabilities();  // viterbi
};
