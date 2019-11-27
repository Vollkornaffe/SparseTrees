#pragma once

#include "Tree.h"

#include <Eigen/Dense>
#include <vector>
#include <ostream>

namespace TreePrint {

template<unsigned int LeafSideLength, typename CustomStore>
inline void add_box(
  TreeNode<3, LeafSideLength, CustomStore> const & node,
  std::vector<Eigen::Vector3d> & verts,
  std::vector<Eigen::Vector4i> & faces
) {

  auto const offset = verts.size() + 1;
  auto const min = node.boundingBox.min();
  auto const max = node.boundingBox.max();

  verts.emplace_back(min[0], min[1], min[2]);
  verts.emplace_back(min[0], min[1], max[2]);
  verts.emplace_back(min[0], max[1], min[2]);
  verts.emplace_back(min[0], max[1], max[2]);
  verts.emplace_back(max[0], min[1], min[2]);
  verts.emplace_back(max[0], min[1], max[2]);
  verts.emplace_back(max[0], max[1], min[2]);
  verts.emplace_back(max[0], max[1], max[2]);

  faces.emplace_back(offset + 0, offset + 4, offset + 5, offset + 1);
  faces.emplace_back(offset + 2, offset + 6, offset + 7, offset + 3);
  faces.emplace_back(offset + 0, offset + 2, offset + 3, offset + 1);
  faces.emplace_back(offset + 4, offset + 5, offset + 7, offset + 6);
  faces.emplace_back(offset + 1, offset + 3, offset + 7, offset + 5);
  faces.emplace_back(offset + 0, offset + 4, offset + 6, offset + 2);

}


template <unsigned int Dimension, unsigned int LeafSideLength, typename CustomStore>
inline void add_element(
  TreeNode<Dimension, LeafSideLength, CustomStore> const & node,
  std::vector<Eigen::Vector3d> & verts,
  std::vector<Eigen::Vector4i> & faces
) {
  if constexpr (Dimension == 3) {
    add_box(node, verts, faces);
  } else {
    // TODO
    // NOT IMPLEMENTED YET
  }
}


template <unsigned int Dimension, unsigned int LeafSideLength, typename CustomStore=void*>
void to_obj(std::ostream & out, Tree<Dimension, LeafSideLength, CustomStore> const & tree) {

  std::vector<Eigen::Vector3d> verts;
  std::vector<Eigen::Vector4i> faces;

  for (auto const & node: tree.nodes)
    add_element(node, verts, faces);

  for (auto const & v: verts)
    out << "v " << v[0] << " " << v[1] << " " << v[2] << std::endl;

  out << std::endl;

  for (auto const & f: faces)
    out << "f " << f[0] << " " << f[1] << " " << f[2] << " " << f[3] << std::endl;
}

} // namespace