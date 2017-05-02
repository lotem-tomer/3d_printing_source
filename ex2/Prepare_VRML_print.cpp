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
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Origin.h>
#include <CGAL/Filtered_kernel.h>
#include <CGAL/Gmpq.h>
#include <CGAL/algorithm.h>
#include <CGAL/Convex_hull_traits_3.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Aff_transformation_3.h>

#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Polyhedron_3.h>

#include <CGAL/Arr_spherical_gaussian_map_3/Arr_polyhedral_sgm.h>
#include <CGAL/Arr_spherical_gaussian_map_3/Arr_polyhedral_sgm_traits.h>
#include <CGAL/Arr_spherical_gaussian_map_3/Arr_polyhedral_sgm_polyhedron_3.h>

#include <boost/tokenizer.hpp>
#include <boost/type_traits.hpp>

#include <vector>
#include <cstdlib>

// NOTE: the choice of double here for a number type may cause problems
//       for degenerate point sets
typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::RT                                        RT;
typedef Kernel::Point_3                                   Point_3;
typedef Kernel::Plane_3                                   Plane_3;
typedef Kernel::Vector_3                                  Vector_3;

#if POLYHEDRON_TRAITS_WITH_NORMALS
typedef CGAL::Polyhedron_traits_with_normals_3<Kernel>    Polyhedron_traits;
#else
typedef Kernel                                            Polyhedron_traits;
#endif

typedef CGAL::Aff_transformation_3<Kernel>				  Transformation_3;
typedef CGAL::Polyhedron_3<Polyhedron_traits>             Polyhedron;
typedef Polyhedron::Vertex                                Vertex;
typedef Polyhedron::Facet                                 Facet;
typedef Polyhedron::Facet_iterator                        Facet_iterator;
typedef Polyhedron::Vertex_iterator                       Vertex_iterator;
typedef Polyhedron::Point_iterator                        Point_iterator;
typedef Polyhedron::Edge_iterator                         Edge_iterator;
typedef Polyhedron::HalfedgeDS                            HalfedgeDS;

typedef CGAL::Arr_polyhedral_sgm_traits<Kernel>                         Gm_traits;
typedef CGAL::Arr_polyhedral_sgm<Gm_traits>                        Gm;
typedef Kernel                                                          Gm_polyhedron_traits;
typedef CGAL::Arr_polyhedral_sgm_polyhedron_3<Gm, Gm_polyhedron_traits> Gm_polyhedron;
typedef CGAL::Arr_polyhedral_sgm_initializer<Gm, Gm_polyhedron>         Gm_initializer;
typedef Gm_polyhedron::Edge_iterator									Poly_edge_iterator;


void readInput(std::string filename, Polyhedron& poly) {
	//same implementation as in Q1a
}

void find_width_and_width_direction_approx(Polyhedron input_poly, Vector_3 & min_direction, RT& squared_width) {
	//TODO: try and implement approximation
	// if not, use exact algorithm for both
}

void find_width_and_width_direction(Polyhedron input_poly, Vector_3 & min_direction, RT& squared_width) {
	//implemented in Q1A
}

int main(int argc, char* argv[]){

	if (argc != 2)
	{
		std::cerr << "Usage: " << argv[0] << " filename " << std::endl;
		std::cerr << "Usage: " << argv[0] << "0 - approximate width; 1- exact width " << std::endl;
		std::exit(0);
	}

	std::string filename = std::string(argv[1]);

	bool flag = argv[1]; //0 - calculate apprximate width; 1- calculate exact width

	Polyhedron poly;
	Vector_3 min_direction;
	RT squared_width;;

	//get VRML polygon - using readInput function
	readInput(filename, poly);

	switch (flag) {
	case 1: //calculate vector V of minimal width using exact algorithm
		find_width_and_width_direction(poly, min_direction,squared_width);
	case 0: // calculate vector V of minmal width using approximate algorithm
		find_width_and_width_direction_approx(poly, min_direction, squared_width);
	}

	// calculate angle Theta between xy plane normal (0,0,1) and min_direction. rotation axis is in the direction of the cross product of (1,0,0) and min_direction
	// rotation angle is 90-Theta (right hand rule). rotate VRML polygon using std::transform
	Vector_3 XY_Normal = Vector_3(0, 0, 1);
	Vector_3 cross = CGAL::cross_product(XY_Normal, min_direction);
	double cosine = (XY_Normal * min_direction) * XY_Normal.squared_length() * min_direction.squared_length();
	CGAL::Aff_transformation_3<Polyhedron> rotate(); //TODO: complete transformation
	std::transform(poly.points_begin, poly.points_end, poly.planes_begin, rotate);

	//find lowest point (minimal z coordinate) - translation needs to move it to z=0
	//find dx,dy,dz - maximal differences between points (bounding box). calculate how much scaling is needed in order to fit in to 100X100X100 mm box
	for (auto i = poly.points_begin(); i != poly.points_end(); ++i) {

	}

	
	//make necessary changes to VRML file

	using namespace std;
	using namespace boost;
	string s = "12252001";
	int offsets[] = { 2, 2, 4 };
	offset_separator f(offsets, offsets + 3);
	tokenizer<offset_separator> tok(s, f);
	for (tokenizer<offset_separator>::iterator beg = tok.begin(); beg != tok.end(); ++beg){
		cout << *beg << "\n";
	}
}