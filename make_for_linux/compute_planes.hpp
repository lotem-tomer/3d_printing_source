#ifndef COMPUTE_PLANES_HPP
#define COMPUTE_PLANES_HPP

/*! \file
 * This file contains a free function that computes the equesions of or the
 * normal to the planes containing the facets of a given polyhedron.
 */

#include <algorithm>
#include <boost/type_traits.hpp>

/*! Compute the equation of the normal to a facet */
template <typename Kernel>
class Normal_equation {
private:
  const Kernel& m_kernel;

public:
  Normal_equation(const Kernel& kernel) : m_kernel(kernel) {}
  template <typename Facet>
  typename Facet::Plane_3 operator()(Facet& f)
  {
    typename Facet::Halfedge_handle h = f.halfedge();
    typename Facet::Vertex::Point_3& p1 = h->vertex()->point();
    typename Facet::Vertex::Point_3& p2 = h->next()->vertex()->point();
    h = h->next()->next();
    typename Kernel::Collinear_3 collinear = m_kernel.collinear_3_object();
    while (collinear(p1, p2, h->vertex()->point())) h = h->next();
    typename Kernel::Construct_cross_product_vector_3 cross_product =
      m_kernel.construct_cross_product_vector_3_object();
    return cross_product(p2 - p1, h->vertex()->point() - p2);
  }
};

template <typename Kernel, typename Polyhedron_T>
void compute_planes(Kernel& kernel, Polyhedron_T& polyhedron, boost::false_type)
{
  std::transform(polyhedron.facets_begin(), polyhedron.facets_end(),
                 polyhedron.planes_begin(), Normal_equation<Kernel>(kernel));
}

/*! Compute the equation of the undelying plane of a facet.
 * This function allows for consecutive vertices to be collinear
 */
template <typename Kernel>
class Plane_equation {
private:
  const Kernel& m_kernel;

public:
  Plane_equation(const Kernel& kernel) : m_kernel(kernel) {}
  template <typename Facet>
  typename Facet::Plane_3 operator()(Facet& f)
  {
    typename Facet::Halfedge_handle h = f.halfedge();
    typename Facet::Vertex::Point_3& p1 = h->vertex()->point();
    typename Facet::Vertex::Point_3& p2 = h->next()->vertex()->point();
    h = h->next()->next();
    typename Kernel::Collinear_3 collinear = m_kernel.collinear_3_object();
    while (collinear(p1, p2, h->vertex()->point())) h = h->next();
    return m_kernel.construct_plane_3_object()(p1, p2, h->vertex()->point());
  }
};

template <typename Kernel, typename Polyhedron_T>
void compute_planes(Kernel& kernel, Polyhedron_T& polyhedron, boost::true_type)
{
  std::transform(polyhedron.facets_begin(), polyhedron.facets_end(),
                 polyhedron.planes_begin(), Plane_equation<Kernel>(kernel));
}

#endif
