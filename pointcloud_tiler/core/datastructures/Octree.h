#pragma once

#include "datastructures/OctreeNodeIndex.h"

#include <functional>
#include <iterator>
#include <optional>
#include <queue>
#include <unordered_map>

#include <types/type_util.h>

template<typename Tree>
struct LevelOrderIterator;
struct PreOrderIterator;
struct PostOrderIterator;

/**
 * Very general Octree datastructure
 */
template<typename T>
struct Octree
{

  /**
   * Proxy object that represents a node in the octree. Provides mutable access to
   * the nodes contents
   */
  template<typename Tree>
  struct Node
  {
    /**
     * If the tree is const, we can only access the tree data as const T, otherwise accessing it as
     * T is fine. Since we can't overload on return type alone, we need this typedef
     */
    using TreeValueType = std::conditional_t<std::is_const_v<Tree>, const T, T>;

    Node()
      : _octree{ nullptr }
    {}
    Node(Tree* octree, const OctreeNodeIndex64& index)
      : _octree(octree)
      , _index(index)
    {}

    TreeValueType& operator*() const { return _octree->_nodes.at(_index); }
    TreeValueType* operator->() const { return &_octree->_nodes.at(_index); }

    const OctreeNodeIndex64& index() const { return _index; }

    std::vector<Node> children() const
    {
      if (_index.levels() == OctreeNodeIndex64::MAX_LEVELS) {
        throw std::runtime_error{ "Can't get children of node that is at MAX_LEVELS!" };
      }

      // If this node has no children, we don't return any, but we have to check first
      if (_octree->_nodes.find(_index.child(0)) == std::end(_octree->_nodes)) {
        return {};
      }

      std::vector<Node> children;
      children.reserve(8);
      for (uint8_t octant = 0; octant < 8; ++octant) {
        children.emplace_back(_octree, _index.child(octant));
      }
      return children;
    }

    std::vector<Node> siblings() const
    {
      if (_index.levels() == 0)
        return {};

      std::vector<Node> siblings;
      siblings.reserve(7);
      for (uint8_t octant = 0; octant < 8; ++octant) {
        if (octant == _index.octant_at_level(_index.levels()))
          continue;
        siblings.emplace_back(_octree, _index.sibling(octant));
      }
      return siblings;
    }

    Node parent() const
    {
      if (_index.levels() == 0) {
        throw std::runtime_error{ "root node has no parent node!" };
      }
      return { _octree, _index.parent() };
    }

    Node insert_child_at(uint8_t octant, const T& data) const
    {
      static_assert(!std::is_const_v<Tree>, "Can't call insert_child_at() on const Node!");
      const auto child_index = _index.child(octant);
      _octree->insert(child_index, data);
      return { _octree, child_index };
    }

    Node insert_child_at(uint8_t octant, T&& data) const
    {
      static_assert(!std::is_const_v<Tree>, "Can't call insert_child_at() on const Node!");
      const auto child_index = _index.child(octant);
      _octree->insert(child_index, std::move(data));
      return { _octree, child_index };
    }

    void erase() const
    {
      static_assert(!std::is_const_v<Tree>, "Can't call erase() on const Node!");
      _octree->erase(_index);
    }

    bool is_leaf() const
    {
      if (_index.levels() == OctreeNodeIndex64::MAX_LEVELS) {
        return true;
      }

      return _octree->_nodes.find(_index.child(0)) == std::end(_octree->_nodes);
    }

  private:
    Tree* _octree;
    OctreeNodeIndex64 _index;
  };

  using ConstNode = Node<const Octree>;
  using MutableNode = Node<Octree>;

  /**
   * Proxy object which can be used in a range-based for-loop to traverse the Octree. It
   * accepts different iterators for different ways of traversing the tree
   */
  template<typename Iterator, typename Tree>
  struct TraversalProxy
  {
    TraversalProxy(Tree* octree)
      : _octree(octree)
    {}

    Iterator begin() const { return Iterator::begin(_octree); }

    Iterator end() const { return Iterator::end(_octree); }

  private:
    Tree* _octree;
  };

  template<typename Tree>
  friend struct LevelOrderIterator;
  friend struct PreOrderIterator;
  friend struct PostOrderIterator;

  template<typename U>
  friend struct Octree;

  Octree() {}
  Octree(std::initializer_list<std::pair<const OctreeNodeIndex64, T>> nodes)
  {
    for (auto& node : nodes) {
      insert_node(std::move(node));
    }
  }

  Octree(const Octree&) = default;
  Octree(Octree&&) = default;

  Octree& operator=(const Octree&) = default;
  Octree& operator=(Octree&&) = default;

  Node<Octree> at(const OctreeNodeIndex64& index)
  {
    auto iter = _nodes.find(index);
    if (iter == std::end(_nodes)) {
      throw std::out_of_range{ std::string("No node found with index ") +
                               OctreeNodeIndex64::to_string(index) };
    }
    return { this, index };
  }

  Node<const Octree> at(const OctreeNodeIndex64& index) const
  {
    auto iter = _nodes.find(index);
    if (iter == std::end(_nodes)) {
      throw std::out_of_range{ std::string("No node found with index ") +
                               OctreeNodeIndex64::to_string(index) };
    }
    return { this, index };
  }

  /**
   * Inserts the node with the given index into this tree and stores the given value in it. If the
   * node already exists, its value is overwritten with the new value
   */
  void insert(const OctreeNodeIndex64& where, const T& val) { insert_node({ where, val }); }
  /**
   * Inserts the node with the given index into this tree and stores the given value in it. If the
   * node already exists, its value is overwritten with the new value
   */
  void insert(const OctreeNodeIndex64& where, T&& val) { insert_node({ where, std::move(val) }); }

  /**
   * Erase the node with the given index from this octree. Since the Octree class has specific
   * guarantees for its structure, 'erase' does never actually remove the given node. Instead,
   * it clears its contents by default-constructing a new object of type T and placing it in
   * the node. If the node has children, all children (direct or indirect) are permanently
   * removed from the tree.
   *
   * 'erase' is implemented in this way to guarantee that each node always has either zero children
   * or exactly eight children!
   */
  void erase(const OctreeNodeIndex64& where) { clear_node(where); }

  /**
   * Returns true if the node with the given index is in this octree
   */
  bool contains(const OctreeNodeIndex64& which) const
  {
    return _nodes.find(which) != std::end(_nodes);
  }

  /**
   * Returns the size of this Octree (i.e. the number of nodes)
   */
  size_t size() const { return _nodes.size(); }

  /**
   * Returns an object that has begin() and end() member-functions to traverse this Octree in level
   * order
   */
  TraversalProxy<LevelOrderIterator<Octree<T>>, Octree<T>> traverse_level_order()
  {
    return { this };
  }

  /**
   * Returns an object that has begin() and end() member-functions to traverse this Octree in level
   * order
   */
  TraversalProxy<LevelOrderIterator<const Octree<T>>, const Octree<T>> traverse_level_order() const
  {
    return { this };
  }

  /**
   * Returns true if the given Octrees are equal. Two Octrees are equal iff they have the exact same
   * nodes. The contents of the nodes are irrelevant, only the structure is relevant.
   */
  friend bool operator==(const Octree& l, const Octree& r) { return l._nodes == r._nodes; }
  /**
   * Returns true if the given Octrees are not equal
   */
  friend bool operator!=(const Octree& l, const Octree& r) { return !(l == r); }

  /**
   * Merges two Octrees 'l' and 'r' using the given binary function. The merged tree has all nodes
   * that are in either 'l' or 'r'. For nodes that are in both 'l' and 'r', their contents are
   * merged using 'merge_func'.
   */
  template<typename BinaryFunc = std::plus<T>>
  static Octree merge(const Octree& l, const Octree& r, BinaryFunc merge_func = {})
  {
    Octree merged;
    for (auto& kv : l._nodes) {
      auto iter_in_r = r._nodes.find(kv.first);
      if (iter_in_r == std::end(r._nodes)) {
        merged.insert(kv.first, kv.second);
      } else {
        merged.insert(kv.first, merge_func(kv.second, iter_in_r->second));
      }
    }

    for (auto& kv : r._nodes) {
      auto iter_in_merged = merged._nodes.find(kv.first);
      if (iter_in_merged != std::end(merged._nodes))
        continue; // Already added to 'merged'

      merged.insert(kv.first, kv.second);
    }

    return merged;
  }

  /**
   * Same as merge(), but allows to transform the type of the data that the octree stored
   */
  template<typename U, typename UnaryFunc, typename BinaryFunc = std::plus<T>>
  static Octree<T> transform_merge(const Octree<T>& l,
                                   const Octree<U>& r,
                                   UnaryFunc transform_func,
                                   BinaryFunc merge_func = {})
  {
    Octree<T> merged;
    for (auto& kv : l._nodes) {
      auto iter_in_r = r._nodes.find(kv.first);
      if (iter_in_r == std::end(r._nodes)) {
        merged.insert(kv.first, kv.second);
      } else {
        merged.insert(kv.first, merge_func(kv.second, transform_func(iter_in_r->second)));
      }
    }

    for (auto& kv : r._nodes) {
      auto iter_in_merged = merged._nodes.find(kv.first);
      if (iter_in_merged != std::end(merged._nodes))
        continue; // Already added to 'merged'

      merged.insert(kv.first, transform_func(kv.second));
    }

    return merged;
  }

  template<typename LabelFunc, typename IncludeFunc = AlwaysTruePredicate>
  std::string to_graphviz(LabelFunc label_func, IncludeFunc include_func = {}) const
  {
    std::stringstream ss;

    ss << "digraph octree {\n";

    for (auto node : traverse_level_order()) {
      if (!include_func(node))
        continue;

      if (node.index() == OctreeNodeIndex64{}) {
        ss << "\troot [label=\"" << label_func(node) << "\"];\n";
      } else {
        ss << "\t" << OctreeNodeIndex64::to_string(node.index()) << " [label=\"" << label_func(node)
           << "\"];\n";
      }
    }

    for (auto node : traverse_level_order()) {
      if (!include_func(node))
        continue;

      if (node.index() == OctreeNodeIndex64{}) {
        ss << "\troot;\n";
      } else {
        const std::string parent_name = (node.index().parent().levels() == 0)
                                          ? ("root")
                                          : (OctreeNodeIndex64::to_string(node.index().parent()));
        ss << "\t" << parent_name << " -> " << OctreeNodeIndex64::to_string(node.index()) << ";\n";
      }
    }

    ss << "}";
    return ss.str();
  }

private:
  std::unordered_map<OctreeNodeIndex64, T> _nodes;

  void insert_node(std::pair<const OctreeNodeIndex64, T> node)
  {
    _nodes.insert_or_assign(node.first, std::move(node.second));

    OctreeNodeIndex64 cur_node = node.first;
    while (cur_node.levels() > 0) {
      ensure_node_is_valid(cur_node);
      cur_node = cur_node.parent();
    }

    // cur_node is now the root node
    if (_nodes.find(cur_node) != std::end(_nodes))
      return;

    default_construct_node(cur_node);
  }

  void ensure_node_is_valid(const OctreeNodeIndex64& key)
  {
    // 1) Node has to exist
    // 2) Node has to have its relevant sibling nodes

    if (_nodes.find(key) == std::end(_nodes)) {
      default_construct_node(key);
    }

    for (uint8_t octant = 0; octant < 8; ++octant) {
      const auto sibling = key.sibling(octant);
      if (sibling == key)
        continue;
      if (_nodes.find(sibling) != std::end(_nodes))
        continue;

      default_construct_node(sibling);
    }
  }

  void default_construct_node(const OctreeNodeIndex64& key) { _nodes[key] = {}; }

  void clear_node(const OctreeNodeIndex64& key)
  {
    _nodes[key] = {};

    std::queue<OctreeNodeIndex64> to_delete;

    for (uint8_t octant = 0; octant < 8; ++octant) {
      to_delete.push(key.child(octant));
    }

    while (!to_delete.empty()) {
      const auto current_node = to_delete.front();
      to_delete.pop();

      if (!_nodes.erase(current_node))
        continue;
      if (current_node.levels() == OctreeNodeIndex64::MAX_LEVELS)
        continue;

      for (uint8_t octant = 0; octant < 8; ++octant) {
        to_delete.push(current_node.child(octant));
      }
    }
  }
};

template<typename Tree>
struct LevelOrderIterator
{

  using difference_type = std::ptrdiff_t;
  using value_type = typename Tree::template Node<Tree>;
  using pointer = value_type*;
  using reference = value_type&;
  using iterator_category = std::forward_iterator_tag;

  static LevelOrderIterator begin(Tree* octree)
  {
    LevelOrderIterator iter{ octree };
    if (!octree->contains({}))
      return iter;

    auto root_node = octree->at({});
    iter._next.push(root_node);
    return iter;
  }

  static LevelOrderIterator end(Tree* octree) { return { octree }; }

  friend bool operator==(const LevelOrderIterator& l, const LevelOrderIterator& r)
  {
    if (l._octree != r._octree)
      return false;
    if (l._next.size() != r._next.size())
      return false;
    if (l._next.empty())
      return true;

    return l._next.front().index() == r._next.front().index();
  }

  friend bool operator!=(const LevelOrderIterator& l, const LevelOrderIterator& r)
  {
    return !(l == r);
  }

  const value_type& operator*() const { return _next.front(); }

  const value_type& operator->() const { return _next.front(); }

  LevelOrderIterator& operator++()
  {
    auto children = _next.front().children();
    for (auto child : children)
      _next.push(child);
    _next.pop();
    return *this;
  }

  LevelOrderIterator operator++(int) const
  {
    auto it = *this;
    ++*this;
    return it;
  }

private:
  LevelOrderIterator(Tree* octree)
    : _octree(octree)
  {}

  Tree* _octree;
  std::queue<value_type> _next;
};