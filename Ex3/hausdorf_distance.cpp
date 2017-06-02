#include <iostream>
#include <fstream>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
// Simplification function
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
// Stop-condition policy
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_length_cost.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_length_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Midpoint_placement.h>
#include <CGAL/Polygon_mesh_processing/distance.h>

#if defined(CGAL_LINKED_WITH_TBB)
#define TAG CGAL::Parallel_tag
#else
#define TAG CGAL::Sequential_tag
#endif

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Polyhedron_3<Kernel> Surface_mesh;

namespace PMP = CGAL::Polygon_mesh_processing;
namespace SMS = CGAL::Surface_mesh_simplification ;

int main( int argc, char** argv )
{

//get meshes
  if (argc<3)
  {
    std::cerr << "Usage: " << argv[0] << " input.off threshold for edge collapse]";
    return 1;
  }

  //simplification (can vary threshold and cost strategy)
  Surface_mesh surface_mesh;
  Surface_mesh surface_mesh_simp;
  std::ifstream is(argv[1]) ; is >> surface_mesh ;
  surface_mesh_simp = surface_mesh;
  double threshold = atof(argv[2]);
  int r = SMS::edge_collapse
            (surface_mesh
             , CGAL::Surface_mesh_simplification::Edge_length_stop_predicate<double>(threshold)
             , CGAL::parameters::vertex_index_map(get(CGAL::vertex_external_index,surface_mesh))
                               .halfedge_index_map  (get(CGAL::halfedge_external_index  ,surface_mesh))
                               .get_cost (SMS::Edge_length_cost <Surface_mesh>())
                               .get_placement(SMS::Midpoint_placement<Surface_mesh>())
            );
 //calculate hausdorf distance between meshes
  auto hd = CGAL::Polygon_mesh_processing::approximate_Hausdorff_distance<TAG>(surface_mesh, surface_mesh_simp, PMP::parameters::number_of_points_per_area_unit(4000));


  std::cout << "approximate Hausdorf distance is: " << hd;
  std::ofstream os( argc > 3 ? argv[3] : "out.off" ) ; os << surface_mesh ;
  return 0 ;
}
