#if defined(_WIN32)
#include <windows.h>
#endif

#include <time.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include <boost/tokenizer.hpp>
#include <boost/type_traits.hpp>

#include <CGAL/basic.h>
#include <CGAL/convex_hull_3.h>

#include "Polyhedron_viewer.hpp"
#include "Wrl_lexer.hpp"
#include "Build_surface.hpp"
#include "compute_planes.hpp"
#include "is_valid.hpp"

namespace VRML_READER{
/*! Construct */
Polyhedron_viewer::Polyhedron_viewer() :
  m_draw_lines(true),
  m_update_convex_hull(true),
  m_simplify(false),
  m_dirty(true),
  m_num_primitives(0),
  m_time(0)
{}

/*! Destruct */
Polyhedron_viewer::~Polyhedron_viewer() { clear(); }

/*! Update the data structure */
void Polyhedron_viewer::update()
{
  auto start_time = clock();
  if (m_update_convex_hull) {
    CGAL::convex_hull_3(m_coords.begin(), m_coords.end(), m_polyhedron);
  }
  else {
    Build_surface<HalfedgeDS, Point_3> surface(m_coords,
                                               m_num_primitives,
                                               m_coord_indices);
    m_polyhedron.delegate(surface);
    m_polyhedron.normalize_border();
  }

  typedef boost::is_same<Polyhedron::Plane_3, Kernel::Plane_3>
    Polyhedron_has_plane;
  compute_planes(m_kernel, m_polyhedron, Polyhedron_has_plane());

  if (!is_valid(m_kernel, m_polyhedron,
                boost::is_same<Polyhedron::Plane_3, Plane_3>()))
    std::cout << "The polyhedron is invalid!" << std::endl;

  auto end_time = clock();
  m_time = (float) (end_time - start_time) / (float) CLOCKS_PER_SEC;
  m_dirty = false;
}

/*! Clear the internal representation */
void Polyhedron_viewer::clear()
{
  m_polyhedron.clear();
  m_dirty = true;
  m_coords.clear();
  m_coord_indices.clear();
}

/*! Obtain the polyhedron */
Polyhedron_viewer::Polyhedron & Polyhedron_viewer::get_polyhedron()
{
  if (m_dirty) update();
  return m_polyhedron;
}

///*! Draw a facet */
//void Polyhedron_viewer::draw_facet(Facet_iterator i)
//{
//  Polyhedron::Halfedge_around_facet_circulator j = i->facet_begin();
//  // Facets in polyhedral surfaces are at least triangles.
//  CGAL_assertion(CGAL::circulator_size(j) >= 3);
//  glBegin(GL_POLYGON);
//  Vector_3 n = get_normal(i, boost::is_same<Polyhedron::Plane_3, Plane_3>());
//  auto x = CGAL::to_double(n.x());
//  auto y = CGAL::to_double(n.y());
//  auto z = CGAL::to_double(n.z());
//  auto length_receip = 1 / sqrtf(x*x + y*y + z*z);
//  x *= length_receip;
//  y *= length_receip;
//  z *= length_receip;
//  double normal[3] = {x, y, z};
//  glNormal3dv(normal);
//  do {
//    auto& point = j->vertex()->point();
//    auto x = CGAL::to_double(point.x());
//    auto y = CGAL::to_double(point.y());
//    auto z = CGAL::to_double(point.z());
//    double vertex[3] = {x, y, z};
//    glVertex3dv(vertex);
//  } while (++j != i->facet_begin());
//  glEnd();
//}

///*! Draw a facet */
//void Polyhedron_viewer::draw_facet_boundary(Facet_iterator i)
//{
//  Polyhedron::Halfedge_around_facet_circulator j = i->facet_begin();
//  // Facets in polyhedral surfaces are at least triangles.
//  CGAL_assertion(CGAL::circulator_size(j) >= 3);
//  glBegin(GL_LINE_LOOP);
//  do {
//    auto& point = j->vertex()->point();
//    auto x = CGAL::to_double(point.x());
//    auto y = CGAL::to_double(point.y());
//    auto z = CGAL::to_double(point.z());
//    double vertex[3] = {x, y, z};
//    glVertex3dv(vertex);
//  } while (++j != i->facet_begin());
//  glEnd();
//}

/*! Obtain the number of tokens in a string
 */
unsigned int Polyhedron_viewer::get_num_tokens(const std::string & str)
{
  typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
  boost::char_separator<char> sep(", \t\n\r");
  tokenizer tokens(str, sep);
  size_t size = 0;
  for (auto it = tokens.begin(); it != tokens.end(); ++it) ++size;
  return size;
}

///*! Draw the internal representation */
//void Polyhedron_viewer::draw()
//{
//  if (m_dirty) update();
//
//  for (auto i = m_polyhedron.facets_begin(); i != m_polyhedron.facets_end(); ++i)
//  {
//    // Pahse 1:
//    glEnable(GL_LIGHTING);
//    if (m_draw_lines) glDepthMask(false);
//    draw_facet(i);
//
//    // Phase 2:
//    glDisable(GL_LIGHTING);
//
//    if (m_draw_lines) {
//      draw_facet_boundary(i);
//
//      // Phase 3:
//      glDepthMask(true);
//      glColorMask(false, false, false, false);
//      draw_facet(i);
//      glColorMask(true, true, true, true);
//    }
//  }
//}

/*! Parse the input file */
bool Polyhedron_viewer::parse(const char* filename)
{
  // Open source file:
  std::ifstream src_stream(filename);

  try {
    Wrl_lexer lexer;
    src_stream >> lexer;
  } catch (std::exception& ex) {
    std::cerr << "Unexpected exception: " << ex.what()
              << "\nBye, bye!" << std::endl;
    return false;
  } catch (...) {
    std::cerr << "Unexpected exception. Bye, bye!" << std::endl;
    return false;
  }

  extern std::string* coord_str_ptr;
  extern std::string* coord_index_str_ptr;

#if 0
  std::cout << "coord: " << coord_str_ptr->c_str()
            << ", coordIndex: " << coord_index_str_ptr->c_str()
            << std::endl;
#endif

  // Coords:
  auto num_values = get_num_tokens(*coord_str_ptr);
  auto size = num_values / 3;
  m_coords.resize(size);
  std::istringstream svalue_coords(*coord_str_ptr, std::istringstream::in);
  for (size_t i = 0; i < size; i++) {
    Kernel::RT x, y, z;
    svalue_coords >> x >> y >> z;
    m_coords[i] = Point_3(x,y,z);
  }

  // Indices:
  num_values = get_num_tokens(*coord_index_str_ptr);
  m_num_primitives = 0;
  m_coord_indices.resize(num_values);
  std::istringstream svalue_index(*coord_index_str_ptr, std::istringstream::in);
  for (size_t i = 0; i < num_values; ++i) {
    int tmp = 0;
    svalue_index >> tmp;
    if (tmp == -1) m_num_primitives++;
    m_coord_indices[i] = tmp;
  }

  return true;
}

/*! Print statistics */
void Polyhedron_viewer::print_stat()
{
  if (m_dirty) update();

  std::cout << "no. vertices: " << m_polyhedron.size_of_vertices()
            << ",  no. edges: " << m_polyhedron.size_of_halfedges()/2
            << ",  no. facets: " << m_polyhedron.size_of_facets()
            << std::endl;

  std::cout << "Update took " << m_time << " seconds to compute"
            << std::endl;
}
}