#ifndef POLYHEDRON_VIEWER_H
#define POLYHEDRON_VIEWER_H

#include <string>
#include <sstream>

#include <boost/type_traits.hpp>

#include <CGAL/basic.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Origin.h>
#include <CGAL/Filtered_kernel.h>
#include <CGAL/Gmpq.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_traits_with_normals_3.h>
#include <CGAL/HalfedgeDS_vector.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/enum.h>
namespace VRML_READER {
	class Polyhedron_viewer {
	public:
		typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
		typedef Kernel::Point_3                                   Point_3;
		typedef Kernel::Plane_3                                   Plane_3;
		typedef Kernel::Vector_3                                  Vector_3;

#if POLYHEDRON_TRAITS_WITH_NORMALS
		typedef CGAL::Polyhedron_traits_with_normals_3<Kernel>    Polyhedron_traits;
#else
		typedef Kernel                                            Polyhedron_traits;
#endif
		typedef CGAL::Polyhedron_3<Polyhedron_traits>             Polyhedron;
		typedef Polyhedron::Vertex                                Vertex;
		typedef Polyhedron::Facet                                 Facet;
		typedef Polyhedron::Facet_iterator                        Facet_iterator;
		typedef Polyhedron::Vertex_iterator                       Vertex_iterator;
		typedef Polyhedron::Point_iterator                        Point_iterator;
		typedef Polyhedron::Edge_iterator                         Edge_iterator;
		typedef Polyhedron::HalfedgeDS                            HalfedgeDS;

	public:
		/*! Constructor */
		Polyhedron_viewer();

		/*! Destructor */
		~Polyhedron_viewer();

		/*! Parse the input file */
		bool parse(const char * filename);

		/*! Draw the data structure */
		void draw();

		/*! Clear the internal representation */
		void clear();

		/*! Return true if the internal representation is empty */
		bool is_empty() const { return m_polyhedron.empty(); }

		/*! Print statistics */
		void print_stat();

		/*! Update the representation */
		void update();

		/*! Obtain the polyhedron */
		Polyhedron& get_polyhedron();

		/*! Set the flag that indicates whether to draw facet boundaries as lines */
		void set_draw_lines(bool flag) { m_draw_lines = flag; }

		/*! Set the flag that indicates whether to update the polyhedron
		  data-structure using convex hull
		*/
		void set_update_convex_hull(bool flag) { m_update_convex_hull = flag; }

		/*! Set the flag that indicates whether to simplify the polyhedron
		  data-structure after using convex hull
		*/
		void set_simplify(bool flag) { m_simplify = flag; }

	private:
		Kernel m_kernel;

		/*! Indicates whether to draw facet boundaries as lines */
		bool m_draw_lines;

		/*! Indicates whether to update the polyhedron data-structure using
		  convex hull
		*/
		bool m_update_convex_hull;

		/*! Indicates whether to simplify the polyhedron data-structure after
		 * using convex hull
		*/
		bool m_simplify;

		/*! Indicates whether the polyhedron has been built */
		bool m_dirty;

		/*! The resulting polyhedron */
		Polyhedron m_polyhedron;

		/*! The coordinates */
		std::vector<Point_3> m_coords;

		/*! The number of primitives */
		size_t m_num_primitives;

		/*! The facets */
		std::vector<size_t> m_coord_indices;

		/*! The time is took to compute the minkowski sum in seconds */
		float m_time;

		/*! Draw a facet */
		void draw_facet(Facet_iterator i);

		/*! Draw the boundary of a facet */
		void draw_facet_boundary(Facet_iterator i);

		/*! Obtain the number of tokens in a string */
		unsigned int get_num_tokens(const std::string & str);

		// Distinguish between polyhedra that support planes and polyhedron that
		// support normals

		/*! Obtain the normal of a facet of a polyhedron that supports normals */
		template <typename Facet>
		const Vector_3& get_normal(const Facet& facet, boost::false_type) const
		{
			return facet->plane();
		}

		/*! Obtain the normal of a facet of a polyhedron that supports planes */
		template <typename Facet>
		Vector_3 get_normal(const Facet& facet, boost::true_type) const
		{
			return facet->plane().orthogonal_vector();
		}
	};
}
#endif
