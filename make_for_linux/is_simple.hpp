#ifndef IS_SIMPLE_HPP
#define IS_SIMPLE_HPP

template <typename Kernel, typename Polyhedron>
bool is_simple(Kernel & kernel, Polyhedron & polyhedron)
{
  auto equal = kernel.equal_3_object();
  for (auto eit = polyhedron.edges_begin(); eit != polyhedron.edges_end();
       ++eit)
  {
    if (equal(eit->face()->plane(), eit->opposite()->face()->plane()))
      return false;
  }

  return true;
}

#endif
