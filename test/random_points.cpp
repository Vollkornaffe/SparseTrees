#include "catch.h"
#include "test_util.h"
#include <SparseTrees>
#include <print_tree.h>
#include <fstream>
#include <iostream>
#include "math.h"

TEST_CASE("Sort X random points into octree and print it", "[random,quadtree]") {

  OcTree tree;

  seed_random();
  BIG_TEST
  {
    auto pos = random_vector<3>(-1000,1000);
    tree.grow_to_fit(pos, 100);
    tree.access_alloc(pos);
  }

  std::ofstream out("random_octree.obj");
  TreePrint::to_obj(out, tree);
  out.close();

};

TEST_CASE("Sort points on a halfsphere into octree", "[octree]") {

  OcTree tree;

  double radius = 100.0;

  size_t num_phi   = 500.0;
  size_t num_theta = 500.0;
  for (size_t theta = 0; theta < num_theta; theta++) {
    double dtheta = M_PI * 0.5 * static_cast<double>(theta)/ static_cast<double>(num_theta);
    for (size_t phi = 0; phi < num_phi; phi++) {
      double dphi = M_PI * 2.0 * static_cast<double>(phi)/ static_cast<double>(num_phi);

      Eigen::Vector3d pos(
        sin(dtheta) * cos(dphi),
        sin(dtheta) * sin(dphi),
        cos(dtheta)
      );
      pos *= radius;

      tree.grow_to_fit(pos, 100);
      tree.access_alloc(pos);
    }

  }

  std::ofstream out("halfsphere_octree.obj");
  TreePrint::to_obj(out, tree);
  out.close();

}