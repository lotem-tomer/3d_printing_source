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

#include <boost/tokenizer.hpp>
#include <boost/type_traits.hpp>

#include <CGAL/Arr_spherical_gaussian_map_3/Arr_polyhedral_sgm.h>
#include <CGAL/Arr_spherical_gaussian_map_3/Arr_polyhedral_sgm_traits.h>
#include <CGAL/Arr_spherical_gaussian_map_3/Arr_polyhedral_sgm_polyhedron_3.h>

#include <Polyhedron_viewer.hpp>

#include <vector>
#include <cstdlib>

// NOTE: the choice of double here for a number type may cause problems
//       for degenerate point sets
// NOTE: the choice of double here for a number type may cause problems
//       for degenerate point sets
typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef K::RT RT;
typedef K::Vector_3							  Vector_3;
typedef K::Point_3                                Point_3;
typedef CGAL::Convex_hull_traits_3<K>             Traits;
typedef Traits::Polyhedron_3                      Polyhedron_3;
typedef CGAL::Polyhedron_3<K>             Polyhedron;

typedef CGAL::Arr_polyhedral_sgm_traits<K>                         Gm_traits;
typedef CGAL::Arr_polyhedral_sgm<Gm_traits>                        Gm;
typedef K                                                          Gm_polyhedron_traits;
typedef CGAL::Arr_polyhedral_sgm_polyhedron_3<Gm, Gm_polyhedron_traits> Gm_polyhedron;
typedef CGAL::Arr_polyhedral_sgm_initializer<Gm, Gm_polyhedron>         Gm_initializer;
typedef Gm_polyhedron::Edge_iterator									Poly_edge_iterator;
typedef CGAL::Direction_3<K>										Direction_3;



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


void read_input_hacky(std::string filename, Gm_polyhedron& inp_poly) {	//lines from VRML file
	std::string line;
	std::vector<Point_3> m_coords;
	std::ifstream VRMLfile("C:/3DPrintingAlgorithms/HW2/3d_printing_source/ex2/Release/mushroom_points.txt");
	if (VRMLfile.is_open()) {
	//	std::cout << "file opened" << std::endl;
	}
	std::vector<std::string> results;
	typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
	boost::char_separator<char> sep(", \t\n\r");
	//std::getline(VRMLfile, line);
	int num_of_lines = 0;
	int j = 0;
	while (std::getline(VRMLfile, line)) {
		num_of_lines++;
		tokenizer tokens(line, sep);
		//std::cout << line << std::endl;
		size_t size = 0;
		for (auto it = tokens.begin(); it != tokens.end(); ++it) ++size;
		size = size / 3;
		std::istringstream svalue_coords(line, std::istringstream::in);
		for (size_t i = 0; i < size; i++) {
			K::RT x, y, z;
			svalue_coords >> x >> y >> z;
	//		std::cout << x << " " << y << " " << z << std::endl;
			m_coords.push_back(Point_3(x, y, z));
		}
	}
	VRMLfile.close();
	//std::cout << m_coords.size();
	CGAL::convex_hull_3(m_coords.begin(), m_coords.end(), inp_poly);
	std::transform(inp_poly.facets_begin(), inp_poly.facets_end(), inp_poly.planes_begin(),
		Plane_equation());
}


void read_input(std::string filename, Gm_polyhedron& inp_poly) {
	//use polyhedron_viewer to parse vrml 
	
	static VRML_READER::Polyhedron_viewer* s_polyhedron_viewer(nullptr);
	s_polyhedron_viewer = new VRML_READER::Polyhedron_viewer;

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


void merge_coplanar_faces(Gm_polyhedron& poly) {
	Gm_polyhedron::Plane_3 p1, p2;
	K::Vector_3 v1, v2;

	for (Gm_polyhedron::Point_const_iterator p = poly.points_begin(); p != poly.points_end(); ++p)
	//	std::cout << *p << std::endl;

	for (Poly_edge_iterator e = poly.edges_begin(); e != poly.edges_end(); ++e) {
		p1 = e->facet()->plane(); p2 = e->opposite()->facet()->plane();
		v1 = p1.orthogonal_vector(); v2 = p2.orthogonal_vector();
	//	std::cout << v1 << std::endl;
	//	std::cout << v2 << std::endl;
		if (p1 == p2) {
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

	for (int i = 0; i < 3; ++i)
		p[i] = (++h)->face()->point();

//	std::cout << p[0] << std::endl << p[1] << std::endl << p[2] << std::endl;
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
		if (vi->degree() > 2) {
			RT cur_width = CGAL::squared_distance(get_ms_plane_from_gm_vertex(*vi), K::Point_3(CGAL::ORIGIN));
			if (is_first || cur_width < squared_width) {
				is_first = false;
				squared_width = cur_width;
				min_direction = vi->point();
			}
		}
	}
}


int main(int argc, char* argv[]){
	

	if (argc != 2)
	{
		std::cerr << "Usage: " << argv[0] << " filename " << std::endl;
		std::cerr << "Usage: " << argv[0] << "0 - approximate width; 1- exact width " << std::endl;
		std::exit(0);
	}

	std::string filename = std::string(argv[1]);

	//bool flag = argv[1]; //0 - calculate apprximate width; 1- calculate exact width
	bool flag = 1;
	Gm_polyhedron poly;
	Gm::Point_2 min_direction;
	RT squared_width;

	//get VRML polygon - using read_input function
	read_input_hacky(filename, poly);

	switch (flag) {
	case 1: //calculate vector V of minimal width using exact algorithm
		find_width_and_width_direction(poly, min_direction,squared_width);
	case 0: // calculate vector V of minmal width using approximate algorithm
		find_width_and_width_direction(poly, min_direction, squared_width);
	}

	// calculate angle Theta between xy plane normal (0,0,1) and min_direction. rotation axis is in the direction of the cross product of (1,0,0) and min_direction
	// rotation angle is 90-Theta (right hand rule). 
	
	Vector_3 XY_Normal = Vector_3(0, 0, 1);
	Vector_3 min_dir_vec = min_direction.to_vector();
	Vector_3 cross = CGAL::cross_product(min_dir_vec,XY_Normal);
	double cos_theta = CGAL::to_double(min_dir_vec.z()) / std::sqrt(CGAL::to_double(min_dir_vec.squared_length()));
	double sin_theta = std::sqrt(1 - cos_theta*cos_theta);
	//std::cout << cos_theta << " " << sin_theta << std::endl;
	// normalize vector for rotation
//	std::vector<double> u{ std::sqrt(CGAL::to_double((cross.x()*cross.x()) / cross.squared_length())), std::sqrt(CGAL::to_double((cross.y()*cross.y()) / cross.squared_length())), std::sqrt(CGAL::to_double((cross.z()*cross.z()) / cross.squared_length())) }; 
	double vec_length = std::sqrt(CGAL::to_double(cross.squared_length()));
	std::vector<double> u{ CGAL::to_double(cross.x()) / vec_length, CGAL::to_double(cross.y()) / vec_length, CGAL::to_double(cross.z()) / vec_length };
//	std::cout << u.at(0) << " " << u.at(1) << " " << u.at(2) << std::endl;

	//matrix for rotation
	K::RT m00 = K::RT(cos_theta + u.at(0)*u.at(0)*(1-cos_theta)); K::RT m01 = K::RT(u.at(0)*u.at(1)*(1-cos_theta)) ; K::RT m02 = K::RT(u.at(1)*sin_theta);
	
	K::RT m10 = K::RT(u.at(0)*u.at(1)*(1-cos_theta)); K::RT m11 = K::RT(cos_theta+u.at(1)*u.at(1)*(1-cos_theta)); K::RT m12 = K::RT(-u.at(0)*sin_theta);
	
	K::RT m20 = K::RT(-u.at(1)*sin_theta); K::RT m21 = K::RT(u.at(0)*sin_theta); K::RT m22 = K::RT(cos_theta);

	//rotate VRML polygon using std::transform
	CGAL::Aff_transformation_3<K> rotate = CGAL::Aff_transformation_3<K>(m00, m01, m02, m10, m11, m12, m20, m21, m22);
	std::transform(poly.points_begin(), poly.points_end(), poly.points_begin(), rotate);


	//find lowest point (minimal z coordinate) - translation needs to move it to z=0
	//find dx,dy,dz - maximal differences between points (bounding box). calculate how much scaling is needed in order to fit in to 100X100X100 mm box
	K::RT min_x = poly.points_begin()->x(); K::RT max_x = poly.points_begin()->x();
	K::RT min_y = poly.points_begin()->y(); K::RT max_y = poly.points_begin()->y();
	K::RT min_z = poly.points_begin()->z(); K::RT max_z = poly.points_begin()->z();
	
	//get extreme points after rotation
	for (auto i = poly.points_begin(); i != poly.points_end(); ++i) {
		if (i->x() < min_x) { min_x = i->x(); }
		if (i->x() > max_x) { max_x = i->x(); }
		if (i->y() < min_y) { min_y = i->y(); }
		if (i->y() > max_y) { max_y = i->y(); }
		if (i->z() < min_z) { min_z = i->z(); }
		if (i->z() > max_z) { max_z = i->z(); }
	}

	double dx = std::abs(CGAL::to_double(max_x - min_x));
	double dy = std::abs(CGAL::to_double(max_y - min_y));
	double dz = std::abs(CGAL::to_double(max_z - min_z));


	double scale = 100/std::max({ dx, dy, dz }); //assuming model units are in mm...

	std::cout << max_x << " " << min_x << std::endl;
	std::cout << max_y << " " << min_y << std::endl;
	std::cout << max_z << " " << min_z << std::endl;


	//make changes to VRML file - and output new processed file.

		// create transform header
	std::string transformHeader = "translation 0 0 " + std::to_string(-1 * CGAL::to_double(min_z)*scale);

		// create translate header
	std::string translateHeader = "rotation " + std::to_string(u.at(0)) +" " +std::to_string(u.at(1)) + " 0  # " + std::to_string(acos(cos_theta) * 180 / M_PI) + " deg.";

		// create scale header
	std::string scaleHeader = "scale " + std::to_string(scale) + " " + std::to_string(scale) + " " + std::to_string(scale);

	//lines from VRML file
	std::string line;
	std::ifstream VRMLfile;
	VRMLfile.open(filename);

	std::ofstream myfile;
	myfile.open("Processed_"+filename);
	std::getline(VRMLfile, line);
	myfile << line+"\n";
	myfile << "Transform {\n";
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
