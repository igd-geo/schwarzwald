#include <catch2/catch_all.hpp>

#include "datastructures/Octree.h"

#include <string>

SCENARIO("Octree - Constructors", "[Octree]")
{
  WHEN("An octree is default constructed")
  {
    Octree<std::string> octree;

    THEN("Is has no nodes") { REQUIRE(!octree.contains({})); }
  }

  WHEN("An octree is constructed from a set of nodes")
  {
    Octree<std::string> octree{ { {}, "root" }, { { 1 }, "node1" }, { { 2 }, "node2" } };

    Octree<std::string> expected_octree{
      { {}, "root" }, { { 0 }, "" }, { { 1 }, "node1" }, { { 2 }, "node2" }, { { 3 }, "" },
      { { 4 }, "" },  { { 5 }, "" }, { { 6 }, "" },      { { 7 }, "" },
    };

    THEN("All nodes are retained")
    {
      REQUIRE(octree.contains({}));
      REQUIRE(octree.contains({ 1 }));
      REQUIRE(octree.contains({ 2 }));

      REQUIRE(*octree.at({}) == "root");
      REQUIRE(*octree.at({ 1 }) == "node1");
      REQUIRE(*octree.at({ 2 }) == "node2");
    }

    THEN("The necessary nodes that make up the octree are default-constructed")
    {
      // Since we added the node "1", we have to create ALL nodes at level 1, otherwise it wouldn't
      // be an octree! All the nodes are default-constructed

      REQUIRE(octree == expected_octree);
    }
  }
}

SCENARIO("Octree - Inserting nodes", "[Octree]")
{
  GIVEN("An empty octree")
  {
    Octree<std::string> octree;

    WHEN("A new node is added to an octree")
    {
      octree.insert({}, "root_node");

      THEN("The node is in the tree")
      {
        REQUIRE(octree.contains({}));
        REQUIRE(*octree.at({}) == "root_node");
      }
    }

    WHEN("A new node is added to a lower level")
    {
      octree.insert({ 1 }, "node1");

      Octree<std::string> expected_octree{
        { {}, "" },    { { 0 }, "" }, { { 1 }, "node1" }, { { 2 }, "" }, { { 3 }, "" },
        { { 4 }, "" }, { { 5 }, "" }, { { 6 }, "" },      { { 7 }, "" },
      };

      THEN("The node is in the tree")
      {
        REQUIRE(octree.contains({ 1 }));
        REQUIRE(*octree.at({ 1 }) == "node1");
      }

      THEN("All necessary other nodes are default-constructed")
      {
        REQUIRE(octree == expected_octree);
      }
    }
  }

  GIVEN("An octree that contains only the root node")
  {
    Octree<std::string> octree{ { {}, "root" } };

    WHEN("A new node is added as a child of the root node")
    {
      const auto root_node = octree.at({});
      root_node.insert_child_at(3, "node3");

      THEN("The child node has been added")
      {
        REQUIRE(octree.contains({ 3 }));
        REQUIRE(*octree.at({ 3 }) == "node3");
      }

      THEN("The sibling nodes of the child node have been added")
      {
        REQUIRE(octree.contains({ 0 }));
        REQUIRE(octree.contains({ 1 }));
        REQUIRE(octree.contains({ 2 }));
        REQUIRE(octree.contains({ 4 }));
        REQUIRE(octree.contains({ 5 }));
        REQUIRE(octree.contains({ 6 }));
        REQUIRE(octree.contains({ 7 }));

        REQUIRE(*octree.at({ 0 }) == "");
        REQUIRE(*octree.at({ 1 }) == "");
        REQUIRE(*octree.at({ 2 }) == "");
        REQUIRE(*octree.at({ 4 }) == "");
        REQUIRE(*octree.at({ 5 }) == "");
        REQUIRE(*octree.at({ 6 }) == "");
        REQUIRE(*octree.at({ 7 }) == "");
      }
    }
  }
}

SCENARIO("Octree - Removing nodes", "[Octree]")
{
  GIVEN("An octree with three levels")
  {
    Octree<std::string> octree{
      { {}, "root" }, { { 1 }, "node1" }, { { 2 }, "node2" }, { { 1, 3 }, "node13" }
    };

    WHEN("The root node is removed through Octree::erase")
    {
      octree.erase({});

      THEN("The node remains but its contents have been cleared")
      {
        REQUIRE(octree.contains({}));
        REQUIRE(*octree.at({}) == "");
      }

      THEN("All child nodes (direct or indirect) have been completely removed")
      {
        REQUIRE(!octree.contains({ 0 }));
        REQUIRE(!octree.contains({ 1 }));
        REQUIRE(!octree.contains({ 2 }));
        REQUIRE(!octree.contains({ 3 }));
        REQUIRE(!octree.contains({ 4 }));
        REQUIRE(!octree.contains({ 5 }));
        REQUIRE(!octree.contains({ 6 }));
        REQUIRE(!octree.contains({ 7 }));

        REQUIRE(!octree.contains({ 1, 0 }));
        REQUIRE(!octree.contains({ 1, 1 }));
        REQUIRE(!octree.contains({ 1, 2 }));
        REQUIRE(!octree.contains({ 1, 3 }));
        REQUIRE(!octree.contains({ 1, 4 }));
        REQUIRE(!octree.contains({ 1, 5 }));
        REQUIRE(!octree.contains({ 1, 6 }));
        REQUIRE(!octree.contains({ 1, 7 }));
      }
    }

    WHEN("The root node is removed through calling erase() on the node itself")
    {
      const auto root_node = octree.at({});
      root_node.erase();

      THEN("The node remains but its contents have been cleared")
      {
        REQUIRE(octree.contains({}));
        REQUIRE(*octree.at({}) == "");
      }

      THEN("All child nodes (direct or indirect) have been completely removed")
      {
        REQUIRE(!octree.contains({ 0 }));
        REQUIRE(!octree.contains({ 1 }));
        REQUIRE(!octree.contains({ 2 }));
        REQUIRE(!octree.contains({ 3 }));
        REQUIRE(!octree.contains({ 4 }));
        REQUIRE(!octree.contains({ 5 }));
        REQUIRE(!octree.contains({ 6 }));
        REQUIRE(!octree.contains({ 7 }));

        REQUIRE(!octree.contains({ 1, 0 }));
        REQUIRE(!octree.contains({ 1, 1 }));
        REQUIRE(!octree.contains({ 1, 2 }));
        REQUIRE(!octree.contains({ 1, 3 }));
        REQUIRE(!octree.contains({ 1, 4 }));
        REQUIRE(!octree.contains({ 1, 5 }));
        REQUIRE(!octree.contains({ 1, 6 }));
        REQUIRE(!octree.contains({ 1, 7 }));
      }
    }

    WHEN("An internal node is removed through Octree::erase")
    {
      octree.erase({ 1 });

      THEN("The node remains but its contents have been cleared")
      {
        REQUIRE(octree.contains({ 1 }));
        REQUIRE(*octree.at({ 1 }) == "");
      }

      THEN("All child nodes (direct or indirect) have been completely removed")
      {
        REQUIRE(!octree.contains({ 1, 0 }));
        REQUIRE(!octree.contains({ 1, 1 }));
        REQUIRE(!octree.contains({ 1, 2 }));
        REQUIRE(!octree.contains({ 1, 3 }));
        REQUIRE(!octree.contains({ 1, 4 }));
        REQUIRE(!octree.contains({ 1, 5 }));
        REQUIRE(!octree.contains({ 1, 6 }));
        REQUIRE(!octree.contains({ 1, 7 }));
      }
    }

    WHEN("An internal node is removed through calling erase() on the node itself")
    {
      const auto node = octree.at({ 1 });
      node.erase();

      THEN("The node remains but its contents have been cleared")
      {
        REQUIRE(octree.contains({ 1 }));
        REQUIRE(*octree.at({ 1 }) == "");
      }

      THEN("All child nodes (direct or indirect) have been completely removed")
      {
        REQUIRE(!octree.contains({ 1, 0 }));
        REQUIRE(!octree.contains({ 1, 1 }));
        REQUIRE(!octree.contains({ 1, 2 }));
        REQUIRE(!octree.contains({ 1, 3 }));
        REQUIRE(!octree.contains({ 1, 4 }));
        REQUIRE(!octree.contains({ 1, 5 }));
        REQUIRE(!octree.contains({ 1, 6 }));
        REQUIRE(!octree.contains({ 1, 7 }));
      }
    }

    WHEN("A leaf node is removed through Octree::erase")
    {
      octree.erase({ 2 });

      THEN("The node remains but its contents have been cleared")
      {
        REQUIRE(octree.contains({ 2 }));
        REQUIRE(*octree.at({ 2 }) == "");
      }
    }

    WHEN("A leaf node is removed by calling erase() on the node itself")
    {
      const auto node = octree.at({ 2 });
      node.erase();

      THEN("The node remains but its contents have been cleared")
      {
        REQUIRE(octree.contains({ 2 }));
        REQUIRE(*octree.at({ 2 }) == "");
      }
    }
  }
}

SCENARIO("Octree - Accessing nodes", "[Octree]")
{
  GIVEN("A non-empty octree")
  {
    Octree<std::string> octree{
      { {}, "root" }, { { 1 }, "node1" }, { { 2 }, "node2" }, { { 1, 3 }, "node13" }
    };

    WHEN("The root-node is accessed with at()")
    {
      const auto node = octree.at({});

      THEN("This node contains the correct data") { REQUIRE(*node == "root"); }

      THEN("We can access the children of this node")
      {
        const auto children = node.children();

        REQUIRE(children.size() == 8);
        REQUIRE(*children[0] == "");
        REQUIRE(*children[1] == "node1");
        REQUIRE(*children[2] == "node2");
        REQUIRE(*children[3] == "");
        REQUIRE(*children[4] == "");
        REQUIRE(*children[5] == "");
        REQUIRE(*children[6] == "");
        REQUIRE(*children[7] == "");
      }

      THEN("Accessing the parent node throws an exception")
      {
        REQUIRE_THROWS_AS(node.parent(), std::exception);
      }

      THEN("Accessing the siblings of this node returns an empty set")
      {
        const auto siblings = node.siblings();
        REQUIRE(siblings.empty());
      }
    }

    WHEN("An internal node is accessed with at()")
    {
      const auto node = octree.at({ 1 });

      THEN("This node contains the correct data") { REQUIRE(*node == "node1"); }

      THEN("We can access the children of this node")
      {
        const auto children = node.children();

        REQUIRE(children.size() == 8);
        REQUIRE(*children[0] == "");
        REQUIRE(*children[1] == "");
        REQUIRE(*children[2] == "");
        REQUIRE(*children[3] == "node13");
        REQUIRE(*children[4] == "");
        REQUIRE(*children[5] == "");
        REQUIRE(*children[6] == "");
        REQUIRE(*children[7] == "");
      }

      THEN("We can access the parent of this node")
      {
        const auto parent = node.parent();
        REQUIRE(*parent == "root");
      }

      THEN("We can access the siblings of this node")
      {
        const auto siblings = node.siblings();
        REQUIRE(siblings.size() == 7);
        REQUIRE(*siblings[0] == "");
        REQUIRE(*siblings[1] == "node2");
        REQUIRE(*siblings[2] == "");
        REQUIRE(*siblings[3] == "");
        REQUIRE(*siblings[4] == "");
        REQUIRE(*siblings[5] == "");
        REQUIRE(*siblings[6] == "");
      }
    }

    WHEN("A leaf node is accessed with at()")
    {
      const auto node = octree.at({ 1, 3 });

      THEN("This node contains the correct data") { REQUIRE(*node == "node13"); }

      THEN("This node has no children")
      {
        const auto children = node.children();
        REQUIRE(children.empty());
      }

      THEN("We can access the parent of this node")
      {
        const auto parent = node.parent();
        REQUIRE(*parent == "node1");
      }

      THEN("We can access the siblings of this node")
      {
        const auto siblings = node.siblings();
        REQUIRE(siblings.size() == 7);
        REQUIRE(*siblings[0] == "");
        REQUIRE(*siblings[1] == "");
        REQUIRE(*siblings[2] == "");
        REQUIRE(*siblings[3] == "");
        REQUIRE(*siblings[4] == "");
        REQUIRE(*siblings[5] == "");
        REQUIRE(*siblings[6] == "");
      }
    }
  }
}

SCENARIO("Traversal", "[Octree]")
{
  GIVEN("An octree with some nodes")
  {
    Octree<std::string> octree{
      { {}, "root" }, { { 1 }, "node1" }, { { 2 }, "node2" }, { { 1, 6 }, "node16" }
    };

    WHEN("The octree is traversed using range-based for with level order")
    {

      std::vector<OctreeNodeIndex64> visited_nodes;
      for (auto node : octree.traverse_level_order()) {
        visited_nodes.push_back(node.index());
      }

      THEN("All nodes are visited in level-order")
      {
        std::vector<OctreeNodeIndex64> expected_nodes{
          {},       { 0 },    { 1 },    { 2 },    { 3 },    { 4 },    { 5 },    { 6 },   { 7 },
          { 1, 0 }, { 1, 1 }, { 1, 2 }, { 1, 3 }, { 1, 4 }, { 1, 5 }, { 1, 6 }, { 1, 7 }
        };

        REQUIRE(visited_nodes == expected_nodes);
      }
    }

    WHEN("std::find_if is used to locate a node in the iterator")
    {
      const auto iter =
        std::find_if(std::begin(octree.traverse_level_order()),
                     std::end(octree.traverse_level_order()),
                     [](const auto& node) { return node.index() == OctreeNodeIndex64{ 2 }; });

      THEN("The correct node is found")
      {
        REQUIRE(iter != std::end(octree.traverse_level_order()));
        REQUIRE(**iter == "node2");
      }
    }

    // WHEN("The octree is traversed using range-based for with Traversal::PreOrder")
    // {
    //   std::vector<OctreeNodeIndex64> visited_nodes;
    //   for (auto node : octree.traverse(Traversal::PreOrder)) {
    //     visited_nodes.push_back(node.index());
    //   }

    //   THEN("All nodes are visited in preorder")
    //   {
    //     std::vector<OctreeNodeIndex64> expected_nodes{
    //       {},       { 0 },    { 1 }, { 1, 0 }, { 1, 1 }, { 1, 2 }, { 1, 3 }, { 1, 4 }, { 1, 5 },
    //       { 1, 6 }, { 1, 7 }, { 2 }, { 3 },    { 4 },    { 5 },    { 6 },    { 7 },
    //     };

    //     REQUIRE(visited_nodes == expected_nodes);
    //   }
    // }

    // WHEN("The octree is traversed using range-based for with Traversal::PostOrder")
    // {
    //   std::vector<OctreeNodeIndex64> visited_nodes;
    //   for (auto node : octree.traverse(Traversal::PostOrder)) {
    //     visited_nodes.push_back(node.index());
    //   }

    //   THEN("All nodes are visited in postorder")
    //   {
    //     std::vector<OctreeNodeIndex64> expected_nodes{
    //       { 0 }, { 1, 0 }, { 1, 1 }, { 1, 2 }, { 1, 3 }, { 1, 4 }, { 1, 5 }, { 1, 6 }, { 1, 7 },
    //       { 1 }, { 2 },    { 3 },    { 4 },    { 5 },    { 6 },    { 7 },    {}
    //     };

    //     REQUIRE(visited_nodes == expected_nodes);
    //   }
    // }
  }
}

SCENARIO("Octree - Merge trees", "[Octree]")
{
  WHEN("Two trees are merged")
  {
    Octree<std::string> tree1{ { {}, "RA" }, { { 1 }, "1A" }, { { 2 }, "2A" } };
    Octree<std::string> tree2{ { {}, "RB" }, { { 1 }, "1B" }, { { 6 }, "6B" } };

    const auto merged = Octree<std::string>::merge(tree1, tree2);

    Octree<std::string> expected_tree{
      { {}, "RARB" }, { { 1 }, "1A1B" }, { { 2 }, "2A" }, { { 6 }, "6B" }
    };

    THEN("The merged tree has all the nodes of both trees, and duplicated nodes are merged with "
         "std::plus")
    {
      REQUIRE(merged == expected_tree);
    }
  }
}