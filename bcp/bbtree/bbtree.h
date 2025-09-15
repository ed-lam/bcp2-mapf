/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "bbtree/brancher.h"
#include "bbtree/node_selectors.h"
#include "types/vector.h"

// using NodeSelection = BestFirstNodeSelection;
// using NodeSelection = DepthFirstNodeSelection;
// using NodeSelection = BestFirstDivingNodeSelection;
using NodeSelection = CappedBestFirstDivingNodeSelection;
// using NodeSelection = BoundTargetedNodeSelection;

class Problem;
struct BBNode;
struct Instance;

class BBTree
{
    // Node selector
    NodeSelection node_sel_;

    // Branch-and-bound state
    SharedPtr<BBNode> current_;
    Size64 num_closed_;
    Size64 next_node_id_;

  public:
    // Constructors and destructor
    BBTree() noexcept = delete;
    ~BBTree() noexcept = default;
    BBTree(const BBTree&) noexcept = default;
    BBTree(BBTree&&) noexcept = default;
    BBTree& operator=(const BBTree&) noexcept = delete;
    BBTree& operator=(BBTree&&) noexcept = delete;
    BBTree(const Instance& instance, Problem& problem) noexcept;

    // Tree functions
    Bool empty() const
    {
        return size() == 0;
    }
    Size64 size() const
    {
        return node_sel_.size() + (current_.get() != nullptr);
    }
    Size64 num_closed() const
    {
        return num_closed_;
    }
    Bool next();
    void branch(Brancher* brancher, Decisions&& decisions);
    void clear()
    {
        current_ = nullptr;
        node_sel_.clear();
    }
    Cost lb() const;
    constexpr static Bool is_best_first_search()
    {
        return NodeSelection::is_best_first_search();
    }

    // Current node functions
    auto& current()
    {
        return *current_;
    }
    auto current_id() const
    {
        return current_->id;
    }
    auto current_depth() const
    {
        return current_->depth;
    }
    auto current_lb() const
    {
        return current_->lb;
    }
    // auto current_decision() const
    // {
    //     return current_->decision.get();
    // }
    auto current_parent_id() const
    {
        return current_->parent ? current_->parent->id : -1;
    }
    Vector<Pair<Brancher*, BrancherData*>> all_decisions() const;

    // Update solve progress
    void update_node_lb(const Cost lb);
};
