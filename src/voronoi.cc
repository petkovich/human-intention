
#include <vector>

#include <boost/polygon/voronoi.hpp>
#include <limits>

#include "voronoi.h"

using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;

namespace boost {
namespace polygon {

template <> struct geometry_concept<geom::Point> {
  typedef point_concept type;
};

template <> struct point_traits<geom::Point> {
  typedef int coordinate_type;
  // Beware !!!! Beware this type casting can be problematic 
  static inline coordinate_type get(const geom::Point &point,
                                    orientation_2d orient) {
    // return (orient == HORIZONTAL) ? (int)point.x() : (int)point.y();
    return (orient == HORIZONTAL) ? point.x() : point.y();
  }
};

template <> struct geometry_concept<geom::Segment> {
  typedef segment_concept type;
};

template <> struct segment_traits<geom::Segment> {
  typedef int coordinate_type;
  typedef geom::Point point_type;

  static inline point_type get(const geom::Segment &segment, direction_1d dir) {
    return dir.to_int() ? segment.A() : segment.B();
  }
};

} // namespace polygon
} // namespace boost

void vor::Voro::printInfo() {
  cout << "Number of cells: " << vd.num_cells() << "\n";

  cout << "Number of edges: " << vd.num_edges() << "\n";

  cout << "Number of nodes: " << vd.num_vertices() << "\n";
  for (voronoi_diagram<double>::const_vertex_iterator it =
           vd.vertices().begin();
       it != vd.vertices().end(); ++it) {
    cout << "X:" << it->x() << "Y:" << it->y() << "\n";
  }
}

void vor::Voro::fillPoints() {
  v_points.clear();
  for (voronoi_diagram<double>::const_vertex_iterator it =
           vd.vertices().begin();
       it != vd.vertices().end(); ++it) {
    v_points.push_back(geom::Point(it->x(), it->y()));
  }
}

void vor::Voro::constructDiagram(std::vector<geom::Segment> &segment_data) {
  inputData.resize(segment_data.size());
  std::copy(segment_data.begin(), segment_data.end(), inputData.begin());
  construct_voronoi(segment_data.begin(), segment_data.end(), &vd);
}

std::vector<geom::Segment> vor::Voro::returnDiagramSegments() {

  std::vector<geom::Segment> segmentator;
  return segmentator;
}



vtkSmartPointer<vtkPolyData>
vor::Voro::insertLines(imr::dijkstra::CDijkstraLite<double> &dijkstra) {
  static const int INVALID_NODE = std::numeric_limits<short>::max();
  vtkSmartPointer<vtkPolyData> data = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
  vtkSmartPointer<vtkPoints> vtk_points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> vtk_lines =
      vtkSmartPointer<vtkCellArray>::New();
  std::pair<geom::Point, geom::Point> segment;
  std::vector<geom::Segment> tmp;
  int n = 0;

  // invalidate points outside polygons
  for (voronoi_diagram<double>::const_vertex_iterator it =
           vd.vertices().begin();
       it != vd.vertices().end(); ++it) {
    bool inside = false;
    geom::Point pt = geom::Point(it->x(), it->y());
    for (int k = 0; k < pgn_dta.size(); k++) {
      tmp = pgn_dta[k].edges();
      if (geom::Segment::cellContains(tmp, pt)) {
        inside = true;
      }
    }
    if (inside) {
      it->color(INVALID_NODE);
    } else {
      it->color(n++);
    }
  }

  std::vector<int> rank(n, 0);

  // invalidate points with rank < 2 and edges incedent to these
  // repeat the process until there is no such point
  bool finish;
  do {
    finish = true;
    std::fill(rank.begin(), rank.end(), 0);

    for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin();
         it != vd.edges().end(); ++it) {
      if (it->is_finite() && it->is_primary() &&
          it->vertex0()->color() <= it->vertex1()->color() &&
          it->vertex0()->color() != INVALID_NODE &&
          it->vertex1()->color() != INVALID_NODE) {
        rank[it->vertex0()->color()]++;
        rank[it->vertex1()->color()]++;
      }
    }

    for (voronoi_diagram<double>::const_vertex_iterator it =
             vd.vertices().begin();
         it != vd.vertices().end(); ++it) {
      if (rank[it->color()] < 2 && it->color() != INVALID_NODE) {
        it->color(INVALID_NODE);
        finish = false;
      }
    }
  } while (!finish);

  // insert only nodes not marked to be invalid
  n = 0;
  for (voronoi_diagram<double>::const_vertex_iterator it =
           vd.vertices().begin();
       it != vd.vertices().end(); ++it) {
    if (it->color() != INVALID_NODE) {
      vtk_points->InsertNextPoint(it->x(), it->y(), 0);
      it->color(n++);
    std:
      cout << "node " /*<< it->color() << " "*/ << it->x() << " " << it->y()
           << std::endl;
    }
  }

  // insert edges of VD connecting two valid nodes
  for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin();
       it != vd.edges().end(); ++it) {
    if (it->is_finite() && it->is_primary()) {
      if (it->vertex0()->color() <= it->vertex1()->color() &&
          it->vertex0()->color() != INVALID_NODE &&
          it->vertex1()->color() != INVALID_NODE) {
        std::size_t index = it->cell()->source_index();
        geom::Point p0 = inputData[index].A();
        geom::Point p1 = inputData[index].B();

        if (it->cell()->source_category() ==
            boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT) {
          //           std::cout << "Cell contains segment start point: (" <<
          //           p0.x() <<", " << p0.y() << ")" << std::endl;
        } else if (it->cell()->source_category() ==
                   boost::polygon::SOURCE_CATEGORY_SEGMENT_END_POINT) {
          //           std::cout << "Cell contains segment end point:  (" <<
          //           p0.x() <<", " << p0.y() << ")" << std::endl;
        } else {
          //           std::cout <<"Cell contains a segment: (" << p0.x() <<", "
          //           << p0.y() << "), (" << p1.x() <<", " << p1.y() << ")"  <<
          //           std::endl;
        }

        line->GetPointIds()->SetId(0, it->vertex0()->color());
        line->GetPointIds()->SetId(1, it->vertex1()->color());
        double xx = it->vertex0()->x() - it->vertex1()->x();
        double yy = it->vertex0()->y() - it->vertex1()->y();
        double dd = sqrt(xx * xx + yy * yy);
        dijkstra.addEdge(it->vertex0()->color(), it->vertex1()->color(), dd);
        dijkstra.addEdge(it->vertex1()->color(), it->vertex0()->color(), dd);
        std::cout << "link " << it->vertex0()->color() << " "
                  << it->vertex1()->color() << " " << dd << " " << 2 * dd << " "
                  << 3 * dd << std::endl;
        vtk_lines->InsertNextCell(line);
      }
    }
  }

  data->SetPoints(vtk_points);
  data->SetLines(vtk_lines);
  return data;
}

vtkSmartPointer<vtkPolyData> vor::Voro::insertPoint(int i) {
  vtkSmartPointer<vtkPolyData> data = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkPoints> vtk_points = vtkSmartPointer<vtkPoints>::New();

  vtk_points->InsertNextPoint(vd.vertices()[i].x(), vd.vertices()[i].y(), 0);

  data->SetPoints(vtk_points);

  return data;
}

void vor::Voro::setPolygons(std::vector<geom::Polygon> &pgn_d) {
  pgn_dta = pgn_d;
}

void vor::Voro::filterPoints() {

  std::vector<int> ers;
  std::vector<geom::Segment> tmp;
  //   std::cout << "ERR SIZES " << pgn_dta.size() << " " << v_points.size()  <<
  //   std::endl;
  for (int k = 0; k < pgn_dta.size(); k++) {
    tmp = pgn_dta[k].edges();
    for (int i = 0; i < v_points.size(); i++) {
      //       std::cout << "ERR " << k << " " << i  << std::endl;
      if (geom::Segment::cellContains(tmp, v_points[i])) {
        ers.push_back(i);
      }
    }
  }
  std::sort(ers.begin(), ers.end());
  int ind;
  std::cout << "ERASE SIZE " << ers.size() << std::endl;
  for (int i = ers.size() - 1; i >= 0; i--) {
    v_points.erase(v_points.begin() + ers[i]);
    std::cout << "ERASE " << ers[i] << std::endl;
  }
}
