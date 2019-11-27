#pragma once

template <unsigned int Dimension, unsigned int LeafLength, typename CustomStore=void*>
struct TreeNode {

  using BoundingBox = Eigen::AlignedBox<int, Dimension>;
  using VectorInt = Eigen::Matrix<int, Dimension, 1>;

  constexpr static unsigned int numChildren = 1 << Dimension;

  unsigned int level; ///< level of this node in the tree, 0 -> leaf

  bool used; ///< Whether this node was traversed in the last step
  bool free; ///< Whether this node is waiting to be recycled

  unsigned int parent; ///< containing node, one level higher
  unsigned int child; ///< index of this node in children of parent

  std::array<int, numChildren> children; ///< children contained by this node, -1 for not allocated

  BoundingBox boundingBox; ///< Space contained by this node

  /**
   * @brief Each node is constructed with all children not allocated
   *
   * @param _level
   * @param _parent
   * @param _child
   * @param offset
   */
  TreeNode(
    unsigned int _level,
    unsigned int _parent,
    unsigned int _child,
    VectorInt const & offset
  ) : level(_level), used(true), free(false), parent(_parent), child(_child) {
    children.fill(-1);
    boundingBox = BoundingBox(
      offset,
      offset + LeafLength * (1 << level) * VectorInt::Ones()
    );
  }

  //std::unique_ptr<CustomStore> store; ///< may be used for anything

};