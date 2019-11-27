#pragma once

#include "TreeNode.h"

#include <list>
#include <vector>
#include <Eigen/Dense>

#define CHECK_BIT(var,pos) (((var) & (1<<(pos)))>>(pos))

/**
 * @brief Can be queried for leafs, and allocates paths to leafs for given positions
 *
 * @details Stores the TreeNodes and handles all the connection information
 *          recycles treenodes so that the index connections stay valid when nodes are deleted
 *
 * @tparam Dimension
 * @tparam LeafSideLength
 * @tparam CustomStore Can be used to store additional information
 */
template <unsigned int Dimension, unsigned int LeafSideLength, typename CustomStore=void*>
struct Tree {

  constexpr static unsigned int numChildren = 1 << Dimension;

  using Vector = Eigen::Matrix<double, Dimension, 1>;
  using VectorInt = Eigen::Matrix<int, Dimension, 1>;
  using VectorUnsignedInt = Eigen::Matrix<unsigned int, Dimension, 1>;
  using BoundingBox = Eigen::AlignedBox<int, Dimension>;

  using TreeNodeT = TreeNode<Dimension, LeafSideLength, CustomStore>;

  unsigned int level; ///< Must be larger than 0

  std::vector<TreeNodeT> nodes; ///< all the nodes, including root and leafs
  std::list<unsigned int> free_nodes; ///< these are waiting to be recycled

  /**
   * @brief Returns the lowest position included by the tree
   * @return
   */
  VectorInt get_offset() {
    return - LeafSideLength * (1 << (level - 1)) * VectorInt::Ones();
  }

  /**
   * @brief returns offset of child
   * @param parentBB
   * @param childNum
   * @return
   */
  inline VectorInt get_child_offset(BoundingBox const& parentBB, unsigned int childNum) {
    VectorInt result = parentBB.min();
    VectorInt offset = parentBB.center() - parentBB.min();
    for (unsigned int d = 0; d < Dimension; d++) {
      result[d] += CHECK_BIT(childNum, (Dimension - 1 - d)) * offset[d];
    }
    return result;
  }

  /**
   * @brief very simple constructor, adds root
   */
  Tree(unsigned int _level = 1) : level(_level) {
    nodes.push_back(TreeNodeT(level, 0, 0, get_offset()));
  }

  /**
   * @brief Increases Level of tree, effectively doubling the sidelength of the total bounding box
   *
   * @details There is a lot happening here, because connections have to be broken and rebuild around the root node
   */
  inline void grow_tree() {

    level++;

    // make a copy of the old root
    auto oldRoot = nodes[0];

    // make a new root with higher level now
    nodes[0] = TreeNodeT(level, 0, 0, get_offset());

    // go through the children of the old root
    for (int child = 0; child < numChildren; child++) {

      // look it up in the copy
      int oldRootChildIdx = oldRoot.children[child];

      // check if the child existed
      if (oldRootChildIdx == -1) continue;

      // in which case we need a new child of the new root to contain the old child
      // here the child number of the old child in old root is the same as new child in new root
      unsigned int newRootChildIdx = add_node(0, child);

      // get the references for convenience
      auto & oldRootChild = nodes[oldRootChildIdx];
      auto & newRootChild = nodes[newRootChildIdx];

      // now, the new parent of the old child is the new root child
      oldRootChild.parent = newRootChildIdx;

      // but the old root child is not the same child as before, it is the 'inside' or 'inverse' child now
      oldRootChild.child = numChildren - 1 - child;
      newRootChild.children[numChildren - 1 - child] = oldRootChildIdx;

    }

  }

  /**
   * @brief Creates a new node and returns the index into the nodes vector
   *
   * @details Recycles freed nodes, connects parent and child
   *
   * @param parent is the index of the parent node in the nodes vector
   * @param child  is the index of the new node in the children array of the parent
   * @return
   */
  inline unsigned int add_node(unsigned int parent, unsigned int child) {

    unsigned int result;

    auto & parentNode = nodes[parent];

    TreeNodeT newNode(parentNode.level-1, parent, child, get_child_offset(parentNode.boundingBox, child));

    // check if nodes can be recycled
    if (free_nodes.empty()) {
      result = nodes.size();
      nodes.push_back(newNode);
    } else {
      result = free_nodes.front();
      assert(nodes[result].free);
      free_nodes.pop_front();
      nodes[result] = newNode;
    }

    nodes[parent].children[child] = result;

    return result;

  }

  /**
   * @brief Makes this node recyclable
   * @param idx
   */
  inline void free_node(unsigned int idx) {

    auto & node = nodes[idx];
    assert(!node.free);
    node.free = true;
    free_nodes.push_back(idx);

    nodes[node.parent].children[node.child] = -1;

    node.customStore.~CustomStore();

  }

  /**
   * @brief sets all used flags to false
   */
  inline void reset_used() {
    for (unsigned int i = 1; i < nodes.size(); i++) nodes[i].used = false;
  }

  /**
   * @brief all that are not used are freed
   */
  inline void free_unused() {
    for (unsigned int i = 1; i < nodes.size(); i++) {
      if (nodes[i].free) continue;
      if (nodes[i].used) continue;
      free_node(i);
    }
  }

  /**
   * @brief all that are not used are freed, the relevant customStores are collected
   * @param freedStores
   */
  inline void free_unused(std::vector<CustomStore> & freedStores) {
    for (unsigned int i = 1; i < nodes.size(); i++) {
      if (nodes[i].free) continue;
      if (nodes[i].used) continue;
      freedStores.emplace_back(std::move(nodes[i].customStore));
      free_node(i);
    }
  }

  /**
   * @brief Heart piece of the tree, for a given position returns the path through the levels of the tree
   *
   * @details Assumes that the tree is large enough to contain the given position
   *
   * @param pos
   * @return
   */
  inline std::vector<unsigned int> get_path(Vector const & pos) {

    assert(nodes[0].boundingBox.contains(VectorInt(pos.array().floor().template cast<int>())));
    assert(nodes[0].boundingBox.contains(VectorInt(pos.array().ceil().template cast<int>())));

    // first floor the position to int
    VectorInt posInt = VectorInt(pos.array().floor().template cast<int>());

    // then use the offset of the tree to get a positive position
    VectorUnsignedInt offsetPosUnsignedInt = (posInt - get_offset()).template cast<unsigned int>();

    // and finally divide by leaf side length, this vector is unique for each leaf
    VectorUnsignedInt vectorIndex = offsetPosUnsignedInt / LeafSideLength;

    std::vector<unsigned int> result;
    result.reserve(level);

    // now go through the levels and compute the correct child
    for (unsigned int step = 0; step < level; step++) {
      unsigned int child = 0;

      // check each dimension and add to child
      for (unsigned int d = 0; d < Dimension; d++) {
        child += CHECK_BIT(vectorIndex[d], (level - 1 - step)) * (1 << (Dimension - 1 - d));
      }

      result.push_back(child);
    }

    return result;

  }

  /**
   * @brief Checks whether the leaf containing pos is allocated, and returns its index (-1 if check fails)
   *
   * @details Assumes that the tree is large enough to contain the given position
   *
   * @param pos
   * @return
   */
  inline int access_check(Vector const & pos) {

    // start with root
    int current_idx = 0;
    assert(nodes[current_idx].boundingBox.contains(VectorInt(pos.array().floor().template cast<int>())));
    assert(nodes[current_idx].boundingBox.contains(VectorInt(pos.array().ceil().template cast<int>())));

    // leverage get_path for child choosing
    for(auto child: get_path(pos)) {

      // get the next lower level node
      current_idx = nodes[current_idx].children[child];

      // if the child isn't allocated, check fails
      if (current_idx == -1) break;

      // all the way, the pos should be contained!
      assert(nodes[current_idx].boundingBox.contains(VectorInt(pos.array().floor().template cast<int>())));
      assert(nodes[current_idx].boundingBox.contains(VectorInt(pos.array().ceil().template cast<int>())));

    }

    return current_idx;
  }

  /**
   * @brief Given a pos, traverses the tree down to the corresponding leaf, allocating nodes if need be
   *
   * @details the returned pair is composed of the resulting index of the destination and whether it needed to be allocated
   *
   * @param pos
   * @return
   */
  inline std::pair<unsigned int, bool> access_alloc(Vector const & pos) {

    // start with root
    int current_idx = 0;
    assert(nodes[current_idx].boundingBox.contains(VectorInt(pos.array().floor().template cast<int>())));
    assert(nodes[current_idx].boundingBox.contains(VectorInt(pos.array().ceil().template cast<int>())));

    // if we have to allocate something on the way, we let the calling context know that
    bool is_new = false;

    // leverage get_path for child choosing
    for(auto child: get_path(pos)) {

      // get parent node for convenience
      auto & parent_node = nodes[current_idx];

      // the child may be -1
      int potential_child = parent_node.children[child];
      if (potential_child == -1) {

        // allocate a new node
        current_idx = add_node(current_idx, child);

        // now we had to allocate something
        is_new = true;

      } else {

        // child existed, we can continue
        current_idx = potential_child;

        // just mark this one as used
        nodes[current_idx].used = true;

      }

      // all the way, the pos should be contained!
      assert(nodes[current_idx].boundingBox.contains(VectorInt(pos.array().floor().template cast<int>())));
      assert(nodes[current_idx].boundingBox.contains(VectorInt(pos.array().ceil().template cast<int>())));

    }

    return { current_idx, is_new };

  }

  /**
   * @brief grows tree as often as needed to fit pos, stops growing when maxLevel is exceeded and returns false
   * @param pos
   */
  inline bool grow_to_fit(Vector const & pos, unsigned int maxLevel) {
    if (!nodes[0].boundingBox.contains(VectorInt(pos.array().floor().template cast<int>()))
     || !nodes[0].boundingBox.contains(VectorInt(pos.array().ceil().template cast<int>()))) {
      if (level >= maxLevel) return false;
      grow_tree();
      return grow_to_fit(pos, maxLevel);
    }

    assert(nodes[0].boundingBox.contains(VectorInt(pos.array().floor().template cast<int>())));
    assert(nodes[0].boundingBox.contains(VectorInt(pos.array().ceil().template cast<int>())));

    return true;
  }

  /**
   * @brief finds 'smallest' leaf idx or returns -1 if no leafs exist
   * @param path
   * @return
   */
  inline int begin(std::vector<unsigned int> & path) const {

    bool anyPath = false;
    for (auto c: nodes[0].children) {
      if (c != -1) {
        anyPath = true;
        break;
      }
    }
    if (!anyPath) return -1;

    int currentIdx = 0;
    path.resize(level, 0u);
    for (unsigned int step = 0; step < level; step++) {
      for (unsigned int child = 0; child < numChildren; child++) {
         if (nodes[currentIdx].children[child] == -1) continue;
         currentIdx = nodes[currentIdx].children[child];
         path[step] = child;

         assert(currentIdx != -1);

         break;
      }
    }

    assert(currentIdx != 0);
    assert(nodes[currentIdx].level == 0);
    assert(!nodes[currentIdx].free);
    assert(nodes[currentIdx].used);

    return currentIdx;

  }

  /**
   * @brief finds next 'bigger' leaf idx and returns true or if currentIdx is already the biggest, returns false
   *
   * @details expects currentIdx to index a leaf, also the path must match
   *          only call this with results of begin or subsequent iterate calls
   *
   * @param currentIdx
   * @param path
   * @return
   */
  inline bool iterate(int & currentIdx, std::vector<unsigned int> & path) const {

    assert(currentIdx != 0);
    assert(nodes[currentIdx].level == 0);
    assert(!nodes[currentIdx].free);
    assert(nodes[currentIdx].used);

    // first ascend the tree until a new branch is found
    // or the root is reached and no new branch exists
    bool newBranch = false;
    int step = level-1;
    for (; step >= 0; step--) {

      // no new branch found so far, ascend the tree
      currentIdx = nodes[currentIdx].parent;

      // use path as a reference, then choose the next bigger
      newBranch = false;
      for (unsigned int child = path[step]+1; child < numChildren; child++) {

        newBranch = nodes[currentIdx].children[child] != -1;
        if (newBranch) {

          currentIdx = nodes[currentIdx].children[child];
          path[step] = child;

          // increase step again, because we made one step down the tree
          step++;

          break;
        }
      }

      // found new branch, break!
      if (newBranch) break;
    }

    // the previous loops didn't yield a new branch all leafs were iterated
    if (!newBranch) return false;

    // now descend the tree to leaf level again, always choosing the minimal leaf
    for (; step < level; step++) {
      for (unsigned int child = 0; child < numChildren; child++) {
         if (nodes[currentIdx].children[child] == -1) continue;
         currentIdx = nodes[currentIdx].children[child];
         path[step] = child;

         assert(currentIdx != -1);

         break;
      }
    }

    assert(currentIdx != 0);
    assert(nodes[currentIdx].level == 0);
    assert(!nodes[currentIdx].free);
    assert(nodes[currentIdx].used);

    return true;

  }

};
