#define _USE_MATH_DEFINES

#include <tgmath.h>
#include <ctime>
#include <fstream>
#include <map>
//#include <nlohmann/json.hpp>
#include <iomanip>
#include <ostream>
#include <vector>
#include "Eigen/Dense"
#include "WarehouseGraph.h"
#include "dijkstra_lite.h"

const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision,
                                       Eigen::DontAlignCols, ", ", "\n");
namespace hirutils {

void prune_nodes(std::vector<Node> &nodes, std::vector<Node> &s_nodes,
                 std::map<int, int> &index_map, Eigen::MatrixXd &F);

void prune_edges(std::vector<Edge> &edges, std::map<int, int> index_map);

void import_edges(imr::dijkstra::CDijkstraLite<double> &dijkstra,
                  std::vector<Edge> &edges, std::vector<Node> &nodes);

void update_dijkstra_paths(
    std::vector<Node> &nodes, imr::dijkstra::CDijkstraLite<double> &dijkstra,
    std::map<int, int> index_map,
    std::vector<std::vector<imr::dijkstra::CDijkstraLite<double>::Weight>>
        &weights);

void solve_dijkstra(std::vector<Node> &nodes,
                    imr::dijkstra::CDijkstraLite<double> &dijkstra,
                    std::map<int, int> index_map, Eigen::MatrixXd &F);

void alternative_association(double x, double y, int node_id,
                             std::vector<Edge> &edges, std::vector<Node> nodes,
                             Eigen::VectorXd &association_vector,
                             std::map<int, int> &index_map);

void associate_position(double x, double y, std::vector<Node> nodes,
                        std::vector<Node> s_nodes,
                        Eigen::VectorXd &association_vector,
                        std::vector<Point<float>> &robots);

bool get_line_intersection(float p0_x, float p0_y, float p1_x, float p1_y,
                           float p2_x, float p2_y, float p3_x, float p3_y);

double gauss(float x1, float y1, float x2, float y2, float sigma);

void add_edges_manually(std::vector<Edge> &edges);

void cut_edges(std::vector<Edge> &original_edges, std::vector<Edge> &edges,
               std::map<int, int> &index_map, std::vector<Point<float>> &robots,
               std::vector<Node> &nodes);

bool get_circle_line_intersection(float p0_x, float p0_y, float p1_x,
                                  float p1_y, float s_x, float s_y, float r);

void cut_nodes(std::vector<Node> &original_nodes, std::vector<Node> &nodes,
               std::vector<Edge> &edges, std::vector<Point<float>> &robots);

}  // namespace hirutils
