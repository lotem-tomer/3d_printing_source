// Copyright (c) 2002  Max Planck Institut fuer Informatik (Germany).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
// You can redistribute it and/or modify it under the terms of the GNU
// General Public License as published by the Free Software Foundation,
// either version 3 of the License, or (at your option) any later version.
//
// Licensees holding a valid commercial license may use this file in
// accordance with the commercial license agreement provided with the software.
//
// This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
// WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
// $URL$
// $Id$
//
//
// Author(s)     : Susan Hert
#define _USE_MATH_DEFINES
#include <math.h>
#include <CGAL/config.h>

#include <CGAL/Homogeneous.h>
#include <CGAL/point_generators_3.h>
#include <CGAL/algorithm.h>
#include <CGAL/Convex_hull_traits_3.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/predicates_on_points_3.h>

#include <CGAL/Arr_spherical_gaussian_map_3/Arr_polyhedral_sgm.h>
#include <CGAL/Arr_spherical_gaussian_map_3/Arr_polyhedral_sgm_traits.h>
#include <CGAL/Arr_spherical_gaussian_map_3/Arr_polyhedral_sgm_polyhedron_3.h>

//#ifdef CGAL_USE_LEDA
//#include <CGAL/leda_integer.h>
//typedef leda_integer RT;
//#else
//#ifdef CGAL_USE_GMP
//#include <CGAL/Gmpz.h>
//typedef CGAL::Gmpz RT;
//#else
//// NOTE: the choice of double here for a number type may cause problems
////       for degenerate point sets
//#include <CGAL/double.h>
//typedef double RT;
//#endif
//#endif

typedef double RT;

#include <vector>
#include <cstdlib>

// NOTE: the choice of double here for a number type may cause problems
//       for degenerate point sets
typedef CGAL::Homogeneous<RT>                     K;
typedef CGAL::Convex_hull_traits_3<K>             Traits;
typedef Traits::Polyhedron_3                      Polyhedron_3;

// define point creator
typedef K::Point_3                                Point_3;
typedef CGAL::Creator_uniform_3<double, Point_3>  PointCreator;
typedef CGAL::Random_points_in_sphere_3<Point_3, PointCreator> Generator;

typedef CGAL::Arr_polyhedral_sgm_traits<K>                         Gm_traits;
typedef CGAL::Arr_polyhedral_sgm<Gm_traits>                        Gm;
typedef K                                                          Gm_polyhedron_traits;
typedef CGAL::Arr_polyhedral_sgm_polyhedron_3<Gm, Gm_polyhedron_traits> Gm_polyhedron;
typedef CGAL::Arr_polyhedral_sgm_initializer<Gm, Gm_polyhedron>         Gm_initializer;
typedef Gm_polyhedron::Edge_iterator									Poly_edge_iterator;
//void draw_points_and_hull(const std::vector<Point_3>& points,
//                          const CGAL::Object& object)
//{
//   std::vector<Point_3>::const_iterator p_it;
//
//   CGAL::Geomview_stream geomview;
//   geomview << CGAL::RED;
//   for (p_it = points.begin(); p_it != points.end(); p_it++)
//   {
//      geomview << *p_it;
//   }
//
//   K::Segment_3     segment;
//   K::Triangle_3    triangle;
//   Point_3          point;
//   Polyhedron_3     polyhedron;
//
//   geomview << CGAL::BLUE;
//   if ( CGAL::assign(point, object) )
//      geomview << point;
//   else if ( CGAL::assign(segment, object) )
//      geomview << segment;
//   else if ( CGAL::assign(triangle, object) )
//      geomview << triangle;
//   else if ( CGAL::assign(polyhedron, object) )
//      geomview << polyhedron;
//
//
//   std::cout << "Press any key to end the program: ";
//   char wait;
//   std::cin >> wait;
//}

void read_input(std::string filename, Polyhedron_3& inp_poly) {
	//TODO: replace this function with a real input reader from vrml
	std::vector<Point_3> points;
	Generator gen(100.0);

	// generate num points and copy them to a vector
	CGAL::cpp11::copy_n(gen, 100, std::back_inserter(points));

	// compute convex hull
	CGAL::convex_hull_3(points.begin(), points.end(), inp_poly);
}

void merge_coplanar_faces(Gm_polyhedron& poly) {
	for (Poly_edge_iterator e = poly.edges_begin(); e!=poly.edges_end(); ++e) {
		if (e->facet()->plane() == e->opposite()->facet()->plane()) {
			--e;
			poly.join_facet(e);
		}
	}
}

void create_rotated_gm(const Gm& gm, Gm& rotated_gm) {
	rotated_gm = gm;
	Gm::Point_2 p;
	for (Gm::Vertex_iterator v = rotated_gm.vertices_begin(); v != rotated_gm.vertices_end(); ++v) {
		p = v->point();
		p = Gm::Point_2(p.x + M_PI, -p.y, p.z);
	}
}

void find_width_and_width_direction(Gm_polyhedron input_poly, Point_3& min_direction, RT& squared_width) {
	Gm_polyhedron ch_poly;
	RT best_width1_squared, best_width2_squared;
	Point_3 best_dir1, best_dir2;

	CGAL::convex_hull_3(input_poly.points_begin(), input_poly.points_end(), ch_poly);
	// merge coplanar faces created by the convex hull algorithm
	merge_coplanar_faces(ch_poly);
	// create gaussian map of the polyhedron

	Gm gm, rotateted_gm;
	Gm_initializer gm_initializer(gm);
	gm_initializer(ch_poly);

	// create mirror gaussian map for finding coupled components in the opposite direction
	create_rotated_gm(gm, rotateted_gm);

	// calc width at Vertex-Face Pair
	calc_width_VF(gm, rotated_gm, best_width1_squared, best_dir1);

	//calc width at Edge-Edge Pair
	calc_width_EE(gm, rotated_gm, best_width2_squared, best_dir2);
}

int main(int argc, char* argv[])
{
  // parse input
  if (argc != 2)
  {
      std::cerr << "Usage: " << argv[0] << " filename " << std::endl;
      std::exit(0);
  }

  std::string filename = std::string(argv[1]);
  Polyhedron_3	input_poly;
  read_input(filename, input_poly);

  Point_3 min_direction;
  RT squared_width;

  find_width_and_width_direction(input_poly, min_direction, squared_width);
  


  //draw_points_and_hull(points, ch_object);
  return 0;
}

