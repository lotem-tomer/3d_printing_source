#ifndef SPHERE_BOUND_H
#define SPHERE_BOUND_H

#include <CGAL/Cartesian.h>
#include <CGAL/Min_sphere_of_spheres_d.h>

#include <vector>
#include <list>

class Sphere_bound {
private:
  // Inexact types:
  typedef float                                         Inexact_NT;
  typedef CGAL::Cartesian<Inexact_NT>                   Inexact_kernel;

  typedef CGAL::Min_sphere_of_spheres_d_traits_3<Inexact_kernel, Inexact_NT>
    Min_sphere_traits;
  typedef CGAL::Min_sphere_of_spheres_d<Min_sphere_traits>
    Min_sphere;
  typedef Min_sphere_traits::Sphere                     Inexact_sphere_3;
  typedef Inexact_kernel::Point_3                       Inexact_point_3;
  typedef Inexact_kernel::Vector_3                      Inexact_vector_3;

  /*! Convert the (exact) point of a polyhedron vertex to a sphere */
  template <typename Polyhedron>
  class Vertex_2_sphere {
  private:
    typedef typename Polyhedron::Vertex                 Vertex;

  public:
    Inexact_sphere_3 operator()(const Vertex& vertex) const
    {
      const auto& point = vertex.point();
      auto x = CGAL::to_double(point.x());
      auto y = CGAL::to_double(point.y());
      auto z = CGAL::to_double(point.z());
      Inexact_point_3 inexact_point(x,y,z);
      return Inexact_sphere_3(inexact_point, 0);
    }
  };

public:
  /*! Calculate the sphere bound of a Polyhedron */
  template <typename Polyhedron>
  void operator()(const Polyhedron& polyhedron, float* center, float& radius)
  {
    std::vector<Inexact_sphere_3> spheres;
    if (polyhedron.empty()) return;
    spheres.resize(polyhedron.size_of_vertices());
    std::transform(polyhedron.vertices_begin(), polyhedron.vertices_end(),
                   spheres.begin(), Vertex_2_sphere<Polyhedron>());
    Min_sphere min_sphere(spheres.begin(), spheres.end());
    std::copy(min_sphere.center_cartesian_begin(),
              min_sphere.center_cartesian_end(),
              center);
    radius = min_sphere.radius();
  }
};

#endif
