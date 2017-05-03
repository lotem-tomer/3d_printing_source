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
#include <CGAL/Direction_3.h>

#include <CGAL/Arr_spherical_gaussian_map_3/Arr_polyhedral_sgm.h>
#include <CGAL/Arr_spherical_gaussian_map_3/Arr_polyhedral_sgm_traits.h>
#include <CGAL/Arr_spherical_gaussian_map_3/Arr_polyhedral_sgm_polyhedron_3.h>

#include <boost/tokenizer.hpp>
#include <boost/type_traits.hpp>

#include <viewer/Polyhedron_viewer.hpp>

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
typedef CGAL::Direction_3<Kernel>										Direction_3;


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

void readInput(std::string filename, Polyhedron& inp_poly) {
	//use polyhedron_viewer to parse vrml 
	static Polyhedron_viewer* s_polyhedron_viewer(nullptr);
	s_polyhedron_viewer = new Polyhedron_viewer;

	s_polyhedron_viewer->parse(filename.c_str());
	auto poly = s_polyhedron_viewer->get_polyhedron();
	CGAL::convex_hull_3(poly.points_begin(), poly.points_end(), inp_poly);
	std::transform(inp_poly.facets_begin(), inp_poly.facets_end(), inp_poly.planes_begin(),
		Plane_equation());
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
	// rotation angle is 90-Theta (right hand rule). 
	
	Vector_3 XY_Normal = Vector_3(0, 0, 1);
	Vector_3 cross = CGAL::cross_product(min_direction,XY_Normal);
	
	double sin_theta = std::sqrt(CGAL::to_double((cross.squared_length() / (XY_Normal.squared_length()*min_direction.squared_length()))));
	double cos_theta = std::sqrt(1 - sin_theta*sin_theta);
	
	// normalize vector for rotation
	std::vector<double> u{ std::sqrt(CGAL::to_double((cross.x()*cross.x()) / cross.squared_length())), std::sqrt(CGAL::to_double((cross.y()*cross.y()) / cross.squared_length())), std::sqrt(CGAL::to_double((cross.z()*cross.z()) / cross.squared_length())) }; 

	//matrix for rotation
	Kernel::RT m00 = Kernel::RT(cos_theta + u.at(0)*u.at(0)); Kernel::RT m01 = Kernel::RT(u.at(0)*u.at(1)*(1-cos_theta)) ; Kernel::RT m02 = Kernel::RT(u.at(1)*sin_theta);
	
	Kernel::RT m10 = Kernel::RT(u.at(0)*u.at(1)*(1-cos_theta)); Kernel::RT m11 = Kernel::RT(cos_theta+u.at(1)*u.at(1)*(1-cos_theta)); Kernel::RT m12 = Kernel::RT(-u.at(0)*sin_theta);
	
	Kernel::RT m20 = Kernel::RT(-u.at(1)*sin_theta); Kernel::RT m21 = Kernel::RT(u.at(0)*sin_theta); Kernel::RT m22 = Kernel::RT(cos_theta+u.at(2)*u.at(2)*(1-cos_theta));

	//rotate VRML polygon using std::transform
	CGAL::Aff_transformation_3<Kernel> rotate = CGAL::Aff_transformation_3<Kernel>(m00, m01, m02, m10, m11, m12, m20, m21, m22);
	std::transform(poly.points_begin(), poly.points_end(), poly.points_begin(), rotate);

	//find lowest point (minimal z coordinate) - translation needs to move it to z=0
	//find dx,dy,dz - maximal differences between points (bounding box). calculate how much scaling is needed in order to fit in to 100X100X100 mm box
	Kernel::RT min_x = poly.points_begin()->x(); Kernel::RT max_x = poly.points_begin()->x();
	Kernel::RT min_y = poly.points_begin()->y(); Kernel::RT max_y = poly.points_begin()->y();
	Kernel::RT min_z = poly.points_begin()->z(); Kernel::RT max_z = poly.points_begin()->z();
	
	//get extreme points after rotation
	for (auto i = poly.points_begin(); i != poly.points_end(); ++i) {
		if (i->x() < min_x) { min_x = i->x(); }
		if (i->x() > max_x) { max_x = i->x(); }
		if (i->x() < min_y) { min_y = i->y(); }
		if (i->x() > max_y) { max_y = i->y(); }
		if (i->x() < min_z) { min_z = i->z(); }
		if (i->x() > max_z) { max_z = i->z(); }
	}

	double dx = std::abs(CGAL::to_double(max_x - min_x));
	double dy = std::abs(CGAL::to_double(max_y - min_y));
	double dz = std::abs(CGAL::to_double(max_z - min_z));

	double scale = 100/std::max({ dx, dy, dz }); //assuming model units are in mm...

	//make changes to VRML file - and output new processed file.

		// create transform header
	std::string transformHeader = "translation 0 0 " + std::to_string(-1 * CGAL::to_double(min_z));

		// create translate header
	std::string translateHeader = "rotation " + std::to_string(u.at(0)) + std::to_string(u.at(1)) + std::to_string(u.at(2)) + " 0  # " + std::to_string(asin(sin_theta) * 180 / M_PI) + " deg.";

		// create scale header
	std::string scaleHeader = "scale " + std::to_string(scale) + " " + std::to_string(scale) + " "+ std::to_string(scale);

	//lines from VRML file
	std::string line;
	std::ifstream VRMLfile;
	VRMLfile.open(filename);

	std::ofstream myfile;
	myfile.open("Processed_"+filename);
	std::getline(VRMLfile, line);
	myfile << line+"\n";
	myfile << transformHeader +"\n";
	myfile << translateHeader + "\n";
	myfile << scaleHeader + "\n";
	myfile << "children [\n";
	while (getline(VRMLfile, line)) {
		myfile << line + "\n";
	}
	myfile << "]\n";
	myfile << "}\n";

	myfile.close();
	VRMLfile.close();

	return 0;
}
