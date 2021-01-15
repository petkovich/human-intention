#include "HumanIntentionRecognition.h"

int main(int argc, char *argv[]) {
  if (argc < 1 || argc > 2) {
    std::cerr << "Usage: " << argv[0] << " <filename>" << std::endl;
    return EXIT_FAILURE;
  }

  if (argc == 2) {
    std::cout << argv[1] << '\n';
  }
  WarehouseGraph graph;
  graph.loadXML(argv[1]);

  Eigen::MatrixXd F;
  auto nodes = graph.getNodes();
  std::vector<Node> s_nodes;
  std::map<int, int> index_map;
  hirutils::prune_nodes(nodes, s_nodes, index_map, F);
  auto edges = graph.getEdges();
  hirutils::prune_edges(edges, index_map);
  hirutils::add_edges_manually(
      edges);  // TODO: fix edges on the edge of the map
  auto original_nodes = nodes;
  auto original_edges = edges;

  std::clock_t begin = std::clock();

  imr::dijkstra::CDijkstraLite<double> dijkstra;
  hirutils::import_edges(dijkstra, edges, nodes);
  std::clock_t end = std::clock();

  std::cout << "IMPORTED EDGES in " << double(end - begin) / CLOCKS_PER_SEC
            << std::endl;

  begin = std::clock();
  hirutils::solve_dijkstra(nodes, dijkstra, index_map, F);
  end = std::clock();
  std::cout << "SOLVED DIJSKTRA in " << double(end - begin) / CLOCKS_PER_SEC
            << std::endl;

  // goals: 39 R33, 498 R236,11774 R1160

  Eigen::VectorXd association_vector;
  association_vector.resize(nodes.size());

  Eigen::MatrixXd I;  // I could be part of HIR class
  int goal_num = 3;
  I.resize(nodes.size(), goal_num);

  I(index_map[39], 0) = 1;
  I(index_map[498], 1) = 1;
  I(index_map[11774], 2) = 1;

  HumanIntentionRecognition HumanIntentionRecognition(goal_num);
  Eigen::MatrixXd
      dummy_observation_matrix;  // maybe rewrite into function/method
  Eigen::VectorXd dummy_association_vector;
  dummy_association_vector.resize(original_nodes.size());
  Eigen::VectorXd tmp_obs;
  tmp_obs.resize(goal_num);
  Eigen::MatrixXd observation_history;
  observation_history.resize(1, goal_num);
  int possible_position_number = 8;
  dummy_observation_matrix.resize(possible_position_number, goal_num);

  // read human and robot positions
  std::vector<Point<float>> robots;
  Point<float> robot;
  std::clock_t session_start, session_end;

  std::ifstream infile("../data/01-03-2019-155557.txt");
  std::ofstream outfile("../data/results/times.txt");

  std::string line;
  double h_x, h_y, l_hx, l_hy, d, f_x, f_y, alt_x, alt_y;
  bool initial = true;
  int counter = 0;

  // while (true) {
  for (std::string line; getline(infile, line);) {
    session_start = std::clock();
    robots.clear();
    std::string delimiter_begin = "[";
    std::string delimiter_end = "]";

    std::string simulation_time = line.substr(0, line.find(delimiter_begin));
    // std::cout << simulation_time << '\n';
    float f_simulation_time = std::stof(simulation_time);
    bool enoguh_moved = false;
    while (line.length() > 3) {
      delimiter_begin = "ID\":";
      delimiter_end = ",";

      std::string id = line.substr(
          line.find(delimiter_begin) + 5,
          line.find(delimiter_end) - line.find(delimiter_begin) - 5);

      delimiter_begin = "\"x\": ";
      delimiter_end = ", \"y";

      std::string x = line.substr(
          line.find(delimiter_begin) + 5,
          line.find(delimiter_end) - line.find(delimiter_begin) - 5);

      delimiter_begin = "\"y\": ";
      delimiter_end = "}";

      std::string y = line.substr(
          line.find(delimiter_begin) + 5,
          line.find(delimiter_end) - line.find(delimiter_begin) - 5);
      delimiter_begin = "ID\":";
      delimiter_end = "}";
      std::string::size_type sz;
      int int_id;
      std::istringstream iss(id);
      iss >> int_id;
      f_x = std::stod(x);
      f_y = std::stod(y);
      if (int_id == 1000) {
        d = sqrt(pow(f_x - l_hx, 2) + pow(f_y - l_hy, 2));
        if (d > 0.25) {
          h_x = f_x;
          h_y = f_y;
          enoguh_moved = true;
          std::cout << id << ' ' << h_x << ' ' << h_y << '\n';
          counter++;
        }

      } else {
        robot.setX(f_x);
        robot.setY(f_y);
        // robots.push_back(robot);
      }
      // std::cout << id << ' ' << x << ' ' << y << '\n';

      line.erase(0, line.find(delimiter_end) + 4);
    }

    if (initial) {
      initial = false;
      l_hx = f_x;
      l_hy = f_y;
      continue;
    }
    if (!enoguh_moved) {
      continue;
    }
    
    
    //hirutils::cut_edges(original_edges, edges, index_map, robots, nodes);
    //hirutils::cut_nodes(original_nodes, nodes, edges, robots);
    //end = std::clock();
    // std::cout << "CUT EDGES AND NODES in "
    //          << double(end - begin) / CLOCKS_PER_SEC << '\n';

    //dijkstra.clear();
    //hirutils::import_edges(dijkstra, edges, nodes);
    // std::cout << "IMPORTED EDGES" << std::endl;

    //begin = std::clock();
    //hirutils::solve_dijkstra(nodes, dijkstra, index_map, F);
    //end = std::clock();
    //std::cout << "SOLVED DIJSKTRA in " << double(end - begin) / CLOCKS_PER_SEC
    //          << std::endl;

    //for (int i = 0; i < possible_position_number; i++) {
    //  alt_x = l_hx + sin(M_PI * 2.0 * i / possible_position_number) * d;
     // alt_y = l_hy + cos(M_PI * 2.0 * i / possible_position_number) * d;
      // std::cout << alt_x << ' ' << h_x << ' ' << alt_y << ' ' << h_y << '\n';
    //  hirutils::associate_position(alt_x, alt_y, nodes, s_nodes,
    //                               dummy_association_vector, robots);
      // std::cout << dummy_association_vector.transpose() * F * I << '\n';

    //  dummy_observation_matrix.row(i) =
    //      dummy_association_vector.transpose() * F * I;
    //}

    l_hx = h_x;
    l_hy = h_y;

    hirutils::associate_position(h_x, h_y, nodes, s_nodes, association_vector,
                                 robots);  // real position



    auto current_observation = association_vector.transpose() * F * I;
    observation_history.row(observation_history.rows() - 1) =
        current_observation;

    if (counter > 30) {
      tmp_obs = observation_history.row(observation_history.rows() - 30) -
                     current_observation;
      for (int goals = 0; goals < goal_num; goals++) {
        if (tmp_obs(goals) < 0) {
          tmp_obs(goals) = 0;
        }
      }
        tmp_obs /= tmp_obs.sum();
        std::cout<< "Probabilities: " << tmp_obs.transpose() << '\n';
        std::string obs_file = "../data/results/all_observations.csv";
        std::ifstream t(obs_file.c_str());
        std::string current((std::istreambuf_iterator<char>(t)),
                            std::istreambuf_iterator<char>());
        std::ofstream file(obs_file.c_str());
        file << current << tmp_obs.transpose().format(CSVFormat) << '\n';
      
    }

    observation_history.conservativeResize(observation_history.rows() + 1,
                                           observation_history.cols());

   // HumanIntentionRecognition.LoadObservation(current_observation,
   //                                           dummy_observation_matrix);
   // HumanIntentionRecognition.UpdateTransition();
    //HumanIntentionRecognition.UpdateProbabilities();

    session_end = std::clock();
    std::cout << "ITERATION FINISHED in "
              << double(session_end - session_start) / CLOCKS_PER_SEC << '\n'
              << '\n';
    outfile << std::fixed << std::setprecision(8) << f_simulation_time << '\n';
  }
  outfile.close();

  return 0;
}
