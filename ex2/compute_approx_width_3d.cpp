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
#define CGAL_IDENTIFICATION_XY 2

#include <math.h>
#include <CGAL/config.h>
#include <boost/timer.hpp>

#include <CGAL/Homogeneous.h>
#include <CGAL/point_generators_3.h>
#include <CGAL/algorithm.h>
#include <CGAL/Convex_hull_traits_3.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/predicates_on_points_3.h>

#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Aff_transformation_3.h>

#include <CGAL/Arr_spherical_gaussian_map_3/Arr_polyhedral_sgm.h>
#include <CGAL/Arr_spherical_gaussian_map_3/Arr_polyhedral_sgm_traits.h>
#include <CGAL/Arr_spherical_gaussian_map_3/Arr_polyhedral_sgm_polyhedron_3.h>

#include <Polyhedron_viewer.hpp>


#include <cstdlib>

// NOTE: the choice of double here for a number type may cause problems
//       for degenerate point sets
typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef K::RT RT;
typedef CGAL::Convex_hull_traits_3<K>             Traits;
typedef Traits::Polyhedron_3                      Polyhedron_3;
typedef CGAL::Polyhedron_3<K>             Polyhedron;

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

//needed to build polyhedron from VRML file
typedef Polyhedron_3::HalfedgeDS										    HalfedgeDS;
typedef CGAL::Polyhedron_incremental_builder_3<HalfedgeDS>   			PolyBuilder;

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

struct Plane_equation {
	template <class Facet>
	typename Facet::Plane_3 operator()(Facet& f) {
		typename Facet::Halfedge_handle h = f.halfedge();
		typedef typename Facet::Plane_3  Plane;
		return Plane(h->vertex()->point(),
			h->next()->vertex()->point(),
			h->next()->next()->vertex()->point());
	}
};

void read_input(std::string filename, Gm_polyhedron& inp_poly) {
	//use polyhedron_viewer to parse vrml 
	static VRML_READER::Polyhedron_viewer* s_polyhedron_viewer(nullptr);
	s_polyhedron_viewer = new VRML_READER::Polyhedron_viewer;

	s_polyhedron_viewer->parse(filename.c_str());
	auto poly  = s_polyhedron_viewer->get_polyhedron();
	CGAL::convex_hull_3(poly.points_begin(), poly.points_end(), inp_poly);
	std::transform(inp_poly.facets_begin(), inp_poly.facets_end(), inp_poly.planes_begin(),
		Plane_equation());

	//std::vector<Point_3> points;
	//Generator gen(100.0);

	//// generate num points and copy them to a vector
	//CGAL::cpp11::copy_n(gen, 100, std::back_inserter(points));

	//// compute convex hull
	//CGAL::convex_hull_3(points.begin(), points.end(), inp_poly);
}

void merge_coplanar_faces(Gm_polyhedron& poly) {
	Gm_polyhedron::Plane_3 p1, p2;
	K::Vector_3 v1, v2;

	//for (Gm_polyhedron::Point_const_iterator p = poly.points_begin(); p!=poly.points_end(); ++p)
	//	std::cout << *p << std::endl;

	for (Poly_edge_iterator e = poly.edges_begin(); e!=poly.edges_end(); ++e) {
		p1 = e->facet()->plane(); p2 = e->opposite()->facet()->plane();
		//v1 = p1.orthogonal_vector(); v2 = p2.orthogonal_vector();
		//std::cout << v1 << std::endl;
		//std::cout << v2 << std::endl;
		if ( p1 == p2) {
			e = poly.join_facet(e);
		}
	}
}

//void create_rotated_gm(const Gm& gm, Gm& rotated_gm) {
void create_rotated_gm(const Gm_polyhedron& poly, Gm& rotated_gm) {
	Gm_polyhedron rotated_poly;
	std::vector<Gm_polyhedron::Point_3> points;
	points.resize(poly.size_of_border_edges());
	for (auto p = poly.points_begin(); p != poly.points_end(); ++p)
		points.push_back(Gm_polyhedron::Point_3(-p->x(), -p->y(), -p->z()));
	CGAL::convex_hull_3(poly.points_begin(), poly.points_end(), rotated_poly);
	Gm_initializer gm_initializer(rotated_gm);
	gm_initializer(rotated_poly);
	//rotated_gm = gm;
	//Gm::Dcel::Vertex::Point p;
	//for (Gm::Vertex_iterator v = rotated_gm.vertices_begin(); v != rotated_gm.vertices_end(); ++v) {
	//	p = -v->point();
	//	v->point() = p;
	//	v->point().init();
	//}
	
}


K::Plane_3 get_ms_plane_from_gm_vertex(const Gm::Vertex& v) {
	Gm::Halfedge_around_vertex_const_circulator h = v.incident_halfedges();
		Gm::Point_3 p[3];

		for (int i = 0; i<3; ++i)
			p[i] = (++h)->face()->point();

		//std::cout << p[0] << std::endl << p[1] << std::endl << p[2] << std::endl;
		return  K::Plane_3(p[0], p[1], p[2]);

};

void find_width_and_width_direction(Gm_polyhedron input_poly, Gm::Point_2& min_direction, RT& squared_width) {
	// merge coplanar faces created by the convex hull algorithm
	merge_coplanar_faces(input_poly);
	//// create gaussian map of the polyhedron

	Gm gm, rotateted_gm, mink_sum_gm;
	Gm_initializer gm_initializer(gm);
	gm_initializer(input_poly);
	bool is_first = true;
	// create mirror gaussian map for finding coupled components in the opposite direction
	create_rotated_gm(input_poly, rotateted_gm);
	// calculate the minkowski sum (the minkowski facets are dual to the 
	mink_sum_gm.minkowski_sum(gm, rotateted_gm);
	
	for (Gm::Vertex_const_iterator vi = mink_sum_gm.vertices_begin(); vi != mink_sum_gm.vertices_end(); ++vi) {
		if (vi->degree()>2) {
			RT cur_width = CGAL::squared_distance(get_ms_plane_from_gm_vertex(*vi), K::Point_3(CGAL::ORIGIN));
			if (is_first || cur_width < squared_width) {
				is_first = false;
				squared_width = cur_width;
				min_direction = vi->point();
			}
		}
	}
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
  Gm_polyhedron	input_poly;
  read_input(filename, input_poly);

  // set timer
  boost::timer timer;

  // do calculations!
  Gm::Point_2 min_direction;
  RT squared_width;

  find_width_and_width_direction(input_poly, min_direction, squared_width);
  
  //measure algorith time
  double secs = timer.elapsed();

  // write output
  std::cout << "The square of the approximate width of the polyhedron is " << squared_width << std::endl;
  std::cout << "The direction of the approximate width vector is" << min_direction << std::endl;
  std::cout << "The execution time was " << secs << std::endl;
  return 0;
}

