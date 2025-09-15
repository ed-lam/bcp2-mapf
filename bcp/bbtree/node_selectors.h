/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "bbtree/bbnode.h"
#include "types/basic_types.h"
#include "types/pointers.h"
#include "types/priority_queue.h"

class Problem;
struct Instance;

class BestFirstNodeSelection
{
    struct Comparison
    {
        Bool operator()(const SharedPtr<BBNode>& lhs, const SharedPtr<BBNode>& rhs) const
        {
            return std::tie(lhs->lb, rhs->depth) > std::tie(rhs->lb, lhs->depth);
        }
    };
    using PriorityQueue = ImmutablePriorityQueue<SharedPtr<BBNode>, Comparison>;

    // Tree state
    PriorityQueue open_;

  public:
    // Constructors and destructor
    BestFirstNodeSelection() noexcept = delete;
    ~BestFirstNodeSelection() noexcept = default;
    BestFirstNodeSelection(const BestFirstNodeSelection&) noexcept = default;
    BestFirstNodeSelection(BestFirstNodeSelection&&) noexcept = default;
    BestFirstNodeSelection& operator=(const BestFirstNodeSelection&) noexcept = default;
    BestFirstNodeSelection& operator=(BestFirstNodeSelection&&) noexcept = default;
    BestFirstNodeSelection(const Instance&, Problem&) noexcept
    {
    }

    // Tree functions
    constexpr static Bool is_best_first_search()
    {
        return true;
    }
    Bool empty() const;
    Size64 size() const;
    void clear();
    const SharedPtr<BBNode>& top() const;
    void pop();
    void push(SharedPtr<BBNode>&& node);
    void push_root(SharedPtr<BBNode>&& node)
    {
        push(std::move(node));
    }
    Cost lb() const;
};

class DepthFirstNodeSelection
{
    // Tree state
    Vector<SharedPtr<BBNode>> open_;

  public:
    // Constructors and destructor
    DepthFirstNodeSelection() noexcept = default;
    ~DepthFirstNodeSelection() noexcept = default;
    DepthFirstNodeSelection(const DepthFirstNodeSelection&) noexcept = default;
    DepthFirstNodeSelection(DepthFirstNodeSelection&&) noexcept = default;
    DepthFirstNodeSelection& operator=(const DepthFirstNodeSelection&) noexcept = default;
    DepthFirstNodeSelection& operator=(DepthFirstNodeSelection&&) noexcept = default;
    DepthFirstNodeSelection(const Instance&, Problem&) noexcept
    {
    }

    // Tree functions
    constexpr static Bool is_best_first_search()
    {
        return false;
    }
    Bool empty() const;
    Size64 size() const;
    void clear();
    const SharedPtr<BBNode>& top() const;
    void pop();
    void push(SharedPtr<BBNode>&& node);
    void push_root(SharedPtr<BBNode>&& node)
    {
        push(std::move(node));
    }
    Cost lb() const;
};

class BestFirstDivingNodeSelection
{
    struct Comparison
    {
        Bool operator()(const SharedPtr<BBNode>& lhs, const SharedPtr<BBNode>& rhs) const
        {
            return std::tie(lhs->lb, rhs->depth) > std::tie(rhs->lb, lhs->depth);
        }
    };
    using PriorityQueue = ImmutablePriorityQueue<SharedPtr<BBNode>, Comparison>;

    // Tree state
    mutable PriorityQueue bfs_;
    mutable Vector<SharedPtr<BBNode>> dfs_;
    mutable SharedPtr<BBNode> subtree_root_;

  public:
    // Constructors and destructor
    BestFirstDivingNodeSelection() noexcept = default;
    ~BestFirstDivingNodeSelection() noexcept = default;
    BestFirstDivingNodeSelection(const BestFirstDivingNodeSelection&) noexcept = default;
    BestFirstDivingNodeSelection(BestFirstDivingNodeSelection&&) noexcept = default;
    BestFirstDivingNodeSelection& operator=(const BestFirstDivingNodeSelection&) noexcept = default;
    BestFirstDivingNodeSelection& operator=(BestFirstDivingNodeSelection&&) noexcept = default;
    BestFirstDivingNodeSelection(const Instance&, Problem&) noexcept
    {
    }

    // Tree functions
    constexpr static Bool is_best_first_search()
    {
        return false;
    }
    Bool empty() const;
    Size64 size() const;
    void clear();
    const SharedPtr<BBNode>& top() const;
    void pop();
    void push(SharedPtr<BBNode>&& node);
    void push_root(SharedPtr<BBNode>&& node);
    Cost lb() const;
};

class CappedBestFirstDivingNodeSelection
{
    struct Comparison
    {
        Bool operator()(const SharedPtr<BBNode>& lhs, const SharedPtr<BBNode>& rhs) const
        {
            return std::tie(lhs->lb, rhs->depth) > std::tie(rhs->lb, lhs->depth);
        }
    };
    using PriorityQueue = ImmutablePriorityQueue<SharedPtr<BBNode>, Comparison>;

    mutable PriorityQueue bfs_;
    mutable Vector<SharedPtr<BBNode>> dfs_;
    mutable SharedPtr<BBNode> subtree_root_;
    mutable Size64 subtree_size_;

  public:
    // Constructors and destructor
    CappedBestFirstDivingNodeSelection() noexcept = delete;
    ~CappedBestFirstDivingNodeSelection() noexcept = default;
    CappedBestFirstDivingNodeSelection(const CappedBestFirstDivingNodeSelection&) noexcept =
        default;
    CappedBestFirstDivingNodeSelection(CappedBestFirstDivingNodeSelection&&) noexcept = default;
    CappedBestFirstDivingNodeSelection& operator=(
        const CappedBestFirstDivingNodeSelection&) noexcept = default;
    CappedBestFirstDivingNodeSelection& operator=(CappedBestFirstDivingNodeSelection&&) noexcept =
        default;
    CappedBestFirstDivingNodeSelection(const Instance&, Problem&) noexcept :
        subtree_size_(0)
    {
    }

    // Tree functions
    constexpr static Bool is_best_first_search()
    {
        return false;
    }
    Bool empty() const;
    Size64 size() const;
    void clear();
    const SharedPtr<BBNode>& top() const;
    void pop();
    void push(SharedPtr<BBNode>&& node);
    void push_root(SharedPtr<BBNode>&& node);
    Cost lb() const;
};

class BoundTargetedNodeSelection
{
    struct Comparison
    {
        Bool operator()(const SharedPtr<BBNode>& lhs, const SharedPtr<BBNode>& rhs) const
        {
            return std::tie(lhs->lb, rhs->depth) > std::tie(rhs->lb, lhs->depth);
        }
    };
    using PriorityQueue = ImmutablePriorityQueue<SharedPtr<BBNode>, Comparison>;

    // Problem
    Problem& problem_;

    // Tree state
    mutable Bool dual_phase_;
    mutable PriorityQueue bfs_;
    mutable Vector<SharedPtr<BBNode>> dfs_;
    mutable SharedPtr<BBNode> subtree_root_;
    mutable Size64 subtree_size_;

  public:
    // Constructors and destructor
    BoundTargetedNodeSelection() noexcept = delete;
    ~BoundTargetedNodeSelection() noexcept = default;
    BoundTargetedNodeSelection(const BoundTargetedNodeSelection&) noexcept = default;
    BoundTargetedNodeSelection(BoundTargetedNodeSelection&&) noexcept = default;
    BoundTargetedNodeSelection& operator=(const BoundTargetedNodeSelection&) noexcept = delete;
    BoundTargetedNodeSelection& operator=(BoundTargetedNodeSelection&&) noexcept = delete;
    BoundTargetedNodeSelection(const Instance& instance, Problem& problem) noexcept;

    // Tree functions
    constexpr static Bool is_best_first_search()
    {
        return false;
    }
    Bool empty() const;
    Size64 size() const;
    void clear();
    const SharedPtr<BBNode>& top() const;
    void pop();
    void push(SharedPtr<BBNode>&& node);
    void push_root(SharedPtr<BBNode>&& node)
    {
        push(std::move(node));
    }
    Cost lb() const;
};
