#ifndef IS_VALID_HPP
#define IS_VALID_HPP

#include <boost/type_traits.hpp>

/*! Check the validity of a polyhedron
 * If the facets of the input polyhedron are not all triangles, and the
 * coordinates of the ticespoints are provided in floating point, it
 * is most likely that the vertices of each facet are not colinear.
 *
 * If the vertices of a facet are not colinear the input data is
 * erroneous, and the resulting polyhedron is invalid.
 *
 * The Gaussian-map method, for example, is sensitive to this erroneous data.
 */
template <typename Kernel, typename Polyhedron>
bool is_valid(Kernel& kernel, Polyhedron& polyhedron, boost::false_type)
{
  auto has_on = kernel.has_on_3_object();
  for (auto fit = polyhedron.facets_begin(); fit != polyhedron.facets_end();
       ++fit)
  {
    auto he_start = fit->facet_begin();
    auto he = he_start;
    typename Kernel::Plane_3 plane(he->vertex()->point(), fit->plane());
    for (++he; he != he_start; ++he) {
      if (!has_on(plane, he->vertex()->point())) return false;
    }
  }

  return true;
}

template <typename Kernel, typename Polyhedron>
bool is_valid(Kernel& kernel, Polyhedron& polyhedron, boost::true_type)
{
  auto has_on = kernel.has_on_3_object();
  for (auto fit = polyhedron.facets_begin(); fit != polyhedron.facets_end();
       ++fit)
  {
    const auto& plane = fit->plane();
    auto he_start = fit->facet_begin();
    auto he = he_start;
    if (!has_on(plane, he->vertex()->point())) return false;
    for (++he; he != he_start; ++he) {
      if (!has_on(plane, he->vertex()->point())) return false;
    }
  }

  return true;
}

#endif
