#include "HIRutils.h"
namespace hirutils {
void prune_nodes(std::vector<Node> &nodes, std::vector<Node> &s_nodes,
                 std::map<int, int> &index_map, Eigen::MatrixXd &F) {
  int cntr = 0;
  std::vector<int> del_idx;
  for (int i = 0; i < nodes.size(); i++) {
    auto node = nodes[i];
    auto node_type = node.getNodeType();
    if (node_type == Enums::ResourceType::Road ||
        node_type == Enums::ResourceType::PickStation) {
      index_map[node.getId()] = cntr;
      cntr++;
    } else {
      if (node_type == Enums::ResourceType::StorageLocation) {
        s_nodes.push_back(node);
      }
      del_idx.push_back(i);
    }
  }
  std::reverse(del_idx.begin(), del_idx.end());
  for (int i = 0; i < del_idx.size(); i++) {
    nodes.erase(nodes.begin() + del_idx[i]);
  }
  F.resize(cntr, cntr);
}

void prune_edges(std::vector<Edge> &edges, std::map<int, int> index_map) {
  std::vector<int> del_idx;
  for (int i = 0; i < edges.size(); i++) {
    auto startnode = edges[i].getStartNodeId();
    auto endnode = edges[i].getEndNodeId();
    if (index_map.find(startnode) == index_map.end() ||
        index_map.find(endnode) == index_map.end()) {
      del_idx.push_back(i);
    }
  }
  std::sort(del_idx.begin(), del_idx.end(), std::greater<int>());
  for (int i = 0; i < del_idx.size(); i++) {
    edges.erase(edges.begin() + del_idx[i]);
  }
}

void import_edges(imr::dijkstra::CDijkstraLite<double> &dijkstra,
                  std::vector<Edge> &edges, std::vector<Node> &nodes) {
  for (int i = 0; i < edges.size(); i++) {
    auto startnode = edges[i].getStartNodeId();
    auto endnode = edges[i].getEndNodeId();
    for (int j = 0; j < nodes.size(); j++) {
      auto id = nodes[j].getId();
      if (id == startnode) {
        auto start_coords = nodes[j].getCoords();
        for (int k = 0; k < nodes.size(); k++) {
          auto id = nodes[k].getId();
          if (id == endnode) {
            auto end_coords = nodes[k].getCoords();
            auto weight = sqrt(pow(start_coords.getX() - end_coords.getX(), 2) +
                               pow(start_coords.getY() - end_coords.getY(), 2));
            dijkstra.addEdge(startnode, endnode, weight);
            // dijkstra.addEdge(endnode, startnode, weight);
          }
        }
      }
    }
  }
  // add other edges manually
}

void update_dijkstra_paths(
    std::vector<Node> &nodes, imr::dijkstra::CDijkstraLite<double> &dijkstra,
    std::map<int, int> index_map,
    std::vector<std::vector<imr::dijkstra::CDijkstraLite<double>::Weight>>
        &weights) {
  weights.clear();
  std::vector<long int> pred;
  std::vector<imr::dijkstra::CDijkstraLite<double>::Weight> ww;
  std::vector<int> indexes;
  std::vector<int> id_list;
  for (int i = 0; i < nodes.size(); i++) {
    std::vector<long int> pred;
    std::vector<imr::dijkstra::CDijkstraLite<double>::Weight> ww;
    dijkstra.solve(nodes[i].getId(), pred, ww);
    weights.push_back(ww);
  }
}

void solve_dijkstra(std::vector<Node> &nodes,
                    imr::dijkstra::CDijkstraLite<double> &dijkstra,
                    std::map<int, int> index_map, Eigen::MatrixXd &F) {
  std::vector<long int> pred;
  std::vector<imr::dijkstra::CDijkstraLite<double>::Weight> ww;
  std::vector<int> indexes;
  std::vector<int> id_list;
  F *= 0;
  for (int i = 0; i < nodes.size(); i++) {
    std::vector<long int> pred;
    std::vector<imr::dijkstra::CDijkstraLite<double>::Weight> ww;
    dijkstra.solve(nodes[i].getId(), pred, ww);
    for (int j = 0; j < ww.size(); j++) {
      if (ww[j] != -1) {
        F(index_map[nodes[i].getId()], index_map[j]) = ww[j];
      }
    }
  }
}

void alternative_association(double x, double y, int node_id,
                             std::vector<Edge> &edges, std::vector<Node> nodes,
                             Eigen::VectorXd &association_vector,
                             std::map<int, int> &index_map) {
  std::vector<int> associable_nodes;
  double sigma = 0.1;
  association_vector *= 0;
  int curr_node = index_map[node_id];
  auto node_x = nodes[curr_node].getCoords().getX();
  auto node_y = nodes[curr_node].getCoords().getY();
  association_vector[curr_node] = gauss(x, y, node_x, node_y, sigma);

  for (int i = 0; i < edges.size(); i++) {
    auto startnode = edges[i].getStartNodeId();
    if (startnode == node_id) {
      associable_nodes.push_back(edges[i].getEndNodeId());
      // std::cout << edges[i].getEndNodeId() << ' ';
      curr_node = index_map[edges[i].getEndNodeId()];
      node_x = nodes[curr_node].getCoords().getX();
      node_y = nodes[curr_node].getCoords().getY();
      association_vector[curr_node] = gauss(x, y, node_x, node_y, sigma);
    }
  }
  association_vector /= association_vector.sum();

  // std::cout << '\n';
}

void associate_position(double x, double y, std::vector<Node> nodes,
                        std::vector<Node> s_nodes,
                        Eigen::VectorXd &association_vector,
                        std::vector<Point<float>> &robots) {
  float dx = 0.68, dy = 0.52, sigma = 0.005;
  for (int i = 0; i < nodes.size(); i++) {
    Point<float> node_coords = nodes[i].getCoords();
    auto node_x = node_coords.getX();
    auto node_y = node_coords.getY();
    bool intersection_exists = false;
    for (int j = 0; j < s_nodes.size(); j++) {
      Point<float> obstacle_coords = s_nodes[j].getCoords();
      auto obstacle_x = obstacle_coords.getX();
      auto obstacle_y = obstacle_coords.getY();
      bool is_intersection =
          get_line_intersection(x, y, node_x, node_y, obstacle_x + dx,
                                obstacle_y + dy, obstacle_x + dx,
                                obstacle_y - dy) ||
          get_line_intersection(x, y, node_x, node_y, obstacle_x - dx,
                                obstacle_y + dy, obstacle_x - dx,
                                obstacle_y - dy) ||
          get_line_intersection(x, y, node_x, node_y, obstacle_x - dx,
                                obstacle_y + dy, obstacle_x + dx,
                                obstacle_y + dy) ||
          get_line_intersection(x, y, node_x, node_y, obstacle_x - dx,
                                obstacle_y - dy, obstacle_x + dx,
                                obstacle_y - dy);
      if (is_intersection) {
        intersection_exists = true;
        association_vector[i] = 0;
        break;
      }
    }
    if (!intersection_exists) {
      for (int j = 0; j < robots.size(); j++) {
        auto robot = robots[j];
        float rx = robot.getX();
        float ry = robot.getY();
        bool is_intersection =
            get_circle_line_intersection(x, y, node_x, node_y, rx, ry, 0.5);
        if (is_intersection) {
          intersection_exists = true;
          association_vector[i] = 0;
          break;
        }
      }
      if (!intersection_exists) {
        association_vector[i] = gauss(x, y, node_x, node_y, sigma);
        // std::cout << association_vector[i] << ' ';
      }
    }
  }
  association_vector /= association_vector.sum();
}

bool get_line_intersection(float p0_x, float p0_y, float p1_x, float p1_y,
                           float p2_x, float p2_y, float p3_x, float p3_y) {
  float s1_x, s1_y, s2_x, s2_y;
  s1_x = p1_x - p0_x;
  s1_y = p1_y - p0_y;
  s2_x = p3_x - p2_x;
  s2_y = p3_y - p2_y;
  float s, t;
  s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) /
      (-s2_x * s1_y + s1_x * s2_y);
  t = (s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) /
      (-s2_x * s1_y + s1_x * s2_y);
  if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
    return true;
  }
  return false;  // No collision
}

double gauss(float x1, float y1, float x2, float y2, float sigma) {
  double r = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
  return pow(M_E, -r * 2 * sigma) / (sigma * sqrt(2 * M_PI));
}

// void add_edges_manually(std::vector<Edge> &edges) {
//   Edge2 edge;
//   edge.addEdge(635, 2436);
//   edges.push_back(edge);
//   edge.addEdge(2436, 635);
//   edges.push_back(edge);
//   edge.addEdge(637, 11723);
//   edges.push_back(edge);
//   edge.addEdge(11723, 637);
//   edges.push_back(edge);

//   edge.addEdge(11755, 11743);
//   edges.push_back(edge);
//   edge.addEdge(11743, 11755);
//   edges.push_back(edge);
//   edge.addEdge(11663, 11724);
//   edges.push_back(edge);
//   edge.addEdge(11724, 11663);
//   edges.push_back(edge);

//   edge.addEdge(638, 11725);
//   edges.push_back(edge);
//   edge.addEdge(11725, 638);
//   edges.push_back(edge);
// }

void cut_edges(std::vector<Edge> &original_edges, std::vector<Edge> &edges,
               std::map<int, int> &index_map, std::vector<Point<float>> &robots,
               std::vector<Node> &nodes) {
  float r = 0.5;
  Point<float> start_coords, end_coords;
  edges.clear();
  for (int i = 0; i < original_edges.size(); i++) {
    bool intersection_exists = false;
    auto startnode = original_edges[i].getStartNodeId();
    auto endnode = original_edges[i].getEndNodeId();
    for (int k = 0; k < nodes.size(); k++) {
      auto id = nodes[k].getId();
      if (id == startnode) {
        start_coords = nodes[k].getCoords();
        break;
      }
    }
    for (int k = 0; k < nodes.size(); k++) {
      auto id = nodes[k].getId();
      if (id == startnode) {
        end_coords = nodes[k].getCoords();
        break;
      }
    }
    for (int j = 0; j < robots.size(); j++) {
      auto robot = robots[j];
      bool intersection = get_circle_line_intersection(
          start_coords.getX(), start_coords.getY(), end_coords.getX(),
          end_coords.getY(), robot.getX(), robot.getY(), r);
      if (intersection) {
        intersection_exists = true;
        break;
      }
    }
    if (!intersection_exists) {
      edges.push_back(original_edges[i]);
    }
  }
}

bool get_circle_line_intersection(float p0_x, float p0_y, float p1_x,
                                  float p1_y, float s_x, float s_y, float r) {
  float ax = p0_x - s_x;
  float ay = p0_y - s_y;
  float bx = p1_x - s_x;
  float by = p1_x - s_y;
  double a, b, c;
  a = pow(bx - ax, 2) + pow(by - ay, 2);
  b = 2 * (ax * (bx - ax) + ay * (by - ay));
  c = pow(ax, 2) + pow(ay, 2) - pow(r, 2);
  double disc = pow(b, 2) - 4 * a * c;
  if (disc <= 0) {
    return 0;
  }
  disc = sqrt(disc);
  double t1 = (-b + disc) / (2 * a);
  double t2 = (-b - disc) / (2 * a);
  if ((0 < t1 && t1 < 1) || (0 < t2 && t2 < 1)) {
    return true;
  }
  return false;
}
void cut_nodes(std::vector<Node> &original_nodes, std::vector<Node> &nodes,
               std::vector<Edge> &edges, std::vector<Point<float>> &robots) {
  nodes.clear();
  for (int i = 0; i < original_nodes.size(); i++) {
    bool end_exists = false;
    for (int j = 0; j < edges.size(); j++) {
      auto startnode = edges[j].getStartNodeId();
      auto endnode = edges[j].getEndNodeId();
      if (startnode == original_nodes[i].getId() ||
          endnode == original_nodes[i].getId()) {
        end_exists = true;
        break;
      }
    }
    if (end_exists) {
      nodes.push_back(original_nodes[i]);
    }
  }
}


} // namespace hirutils
