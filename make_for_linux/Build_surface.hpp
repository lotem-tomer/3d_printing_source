#ifndef BUILD_SURFACE_HPP
#define BUILD_SURFACE_HPP

#include <vector>

#include <CGAL/basic.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>

template <typename HDS, typename Point_3>
class Build_surface : public CGAL::Modifier_base<HDS> {
public:
  /*! Constructor */
  Build_surface(std::vector<Point_3>& coords, size_t num_primitives,
                std::vector<size_t>& coord_indices) :
    m_coords(coords),
    m_num_primitives(num_primitives),
    m_coord_indices(coord_indices)
  {}

  /*! Destructor */
  virtual ~Build_surface() {}

  /*! builds the polyhedron */
  void operator()(HDS & hds) {
    // Postcondition: `hds' is a valid polyhedral surface.
    CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);
    typedef typename CGAL::Polyhedron_incremental_builder_3<HDS>::size_type
      size_type;
    size_type coord_array_size = m_coords.size();
    auto num_facets = m_num_primitives;

    B.begin_surface(coord_array_size, num_facets);
    typedef typename HDS::Vertex Vertex;
    typedef typename Vertex::Point Point;
    // Add the points:
    for (size_t i = 0; i < coord_array_size; i++) B.add_vertex(m_coords[i]);

    // Add the faces:
    size_t j = 0;
    for (size_t i = 0; i < num_facets; i++) {
      B.begin_facet();
      for (; m_coord_indices[j] != (size_t) -1; j++)
        B.add_vertex_to_facet(m_coord_indices[j]);
      j++;
      B.end_facet();
    }
    B.end_surface();
  }

private:
  /*! The coordinates */
  std::vector<Point_3> & m_coords;

  /*! The number of primitives */
  size_t m_num_primitives;

  /*! The facets */
  std::vector<size_t> & m_coord_indices;
};

#endif
