/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#include "bbtree/node_selectors.h"
#include "problem/problem.h"
#include "types/float_compare.h"
#include "types/tracy.h"

#define BEST_FIRST_DIVING_LB_DIFFERENCE (2.0)
#define BEST_FIRST_DIVING_CAP (100)
#define BFS_SWITCH_GAP (0.1) // in percent (0.1%)

#define TRACY_COLOUR tracy::Color::ColorType::ForestGreen

Bool BestFirstNodeSelection::empty() const
{
    ZoneScopedC(TRACY_COLOUR);

    return open_.empty();
}

Size BestFirstNodeSelection::size() const
{
    ZoneScopedC(TRACY_COLOUR);

    return open_.size();
}

void BestFirstNodeSelection::clear()
{
    ZoneScopedC(TRACY_COLOUR);

    open_.clear();
}

const SharedPtr<BBNode>& BestFirstNodeSelection::top() const
{
    ZoneScopedC(TRACY_COLOUR);

    debug_assert(!empty());
    return open_.top();
}

void BestFirstNodeSelection::pop()
{
    ZoneScopedC(TRACY_COLOUR);

    debug_assert(!empty());
    return open_.pop();
}

void BestFirstNodeSelection::push(SharedPtr<BBNode>&& node)
{
    ZoneScopedC(TRACY_COLOUR);

    open_.push(std::move(node));
}

Cost BestFirstNodeSelection::lb() const
{
    ZoneScopedC(TRACY_COLOUR);

    return !empty() ? top()->lb : INF;
}

// ---------------------------------------------------------------------------------------------------------------------

Bool DepthFirstNodeSelection::empty() const
{
    ZoneScopedC(TRACY_COLOUR);

    return open_.empty();
}

Size DepthFirstNodeSelection::size() const
{
    ZoneScopedC(TRACY_COLOUR);

    return open_.size();
}

void DepthFirstNodeSelection::clear()
{
    ZoneScopedC(TRACY_COLOUR);

    open_.clear();
}

const SharedPtr<BBNode>& DepthFirstNodeSelection::top() const
{
    ZoneScopedC(TRACY_COLOUR);

    debug_assert(!empty());
    return open_.back();
}

void DepthFirstNodeSelection::pop()
{
    ZoneScopedC(TRACY_COLOUR);

    debug_assert(!empty());
    return open_.pop_back();
}

void DepthFirstNodeSelection::push(SharedPtr<BBNode>&& node)
{
    ZoneScopedC(TRACY_COLOUR);

    open_.push_back(std::move(node));
}

Cost DepthFirstNodeSelection::lb() const
{
    ZoneScopedC(TRACY_COLOUR);

    return !empty() ? open_.front()->lb : INF;
}

// ---------------------------------------------------------------------------------------------------------------------

Bool BestFirstDivingNodeSelection::empty() const
{
    ZoneScopedC(TRACY_COLOUR);

    return dfs_.empty() && bfs_.empty();
}

Size BestFirstDivingNodeSelection::size() const
{
    ZoneScopedC(TRACY_COLOUR);

    return dfs_.size() + bfs_.size();
}

void BestFirstDivingNodeSelection::clear()
{
    ZoneScopedC(TRACY_COLOUR);

    dfs_.clear();
    bfs_.clear();
}

const SharedPtr<BBNode>& BestFirstDivingNodeSelection::top() const
{
    ZoneScopedC(TRACY_COLOUR);

    if (dfs_.empty())
    {
        debug_assert(!bfs_.empty());
        subtree_root_ = std::move(bfs_.top());
        bfs_.pop();
        dfs_.push_back(subtree_root_);
        // println("New subtree root node {}, lb {} from the priority queue", subtree_root_->id, subtree_root_->lb);
    }
    else
    {
        // println("Top stack node {}, lb {}", dfs_.back()->id, dfs_.back()->lb);
    }
    return dfs_.back();
}

void BestFirstDivingNodeSelection::pop()
{
    ZoneScopedC(TRACY_COLOUR);

    debug_assert(!dfs_.empty());
    dfs_.pop_back();
}

void BestFirstDivingNodeSelection::push(SharedPtr<BBNode>&& node)
{
    ZoneScopedC(TRACY_COLOUR);

    debug_assert(subtree_root_);
    if (is_gt(node->lb, eps_ceil(subtree_root_->lb) + BEST_FIRST_DIVING_LB_DIFFERENCE))
    {
        // if (subtree_root_)
        // {
        //     println("Pushing node {}, lb {} on to the priority queue with root {}, lb {}",
        //             node->id, node->lb,
        //             subtree_root_ ? subtree_root_->id : INF, subtree_root_ ? subtree_root_->lb : INF);
        // }
        // else
        // {
        //     println("Pushing node {} with lb {} on to the priority queue without root",
        //             node->id, node->lb,
        //             subtree_root_ ? subtree_root_->id : INF, subtree_root_ ? subtree_root_->lb : INF);
        // }
        bfs_.push(std::move(node));
    }
    else
    {
        // if (subtree_root_)
        // {
        //     println("Pushing node {}, lb {} on to the stack with root {}, lb {}",
        //             node->id, node->lb,
        //             subtree_root_ ? subtree_root_->id : INF, subtree_root_ ? subtree_root_->lb : INF);
        // }
        // else
        // {
        //     println("Pushing node {}, lb {} on to the stack without root",
        //             node->id, node->lb,
        //             subtree_root_ ? subtree_root_->id : INF, subtree_root_ ? subtree_root_->lb : INF);
        // }
        dfs_.push_back(std::move(node));
    }
}

void BestFirstDivingNodeSelection::push_root(SharedPtr<BBNode>&& node)
{
    ZoneScopedC(TRACY_COLOUR);

    bfs_.push(std::move(node));
}

Cost BestFirstDivingNodeSelection::lb() const
{
    ZoneScopedC(TRACY_COLOUR);

    return std::min(!dfs_.empty() ? dfs_.front()->lb : INF, !bfs_.empty() ? bfs_.top()->lb : INF);
}

// ---------------------------------------------------------------------------------------------------------------------

Bool CappedBestFirstDivingNodeSelection::empty() const
{
    ZoneScopedC(TRACY_COLOUR);

    return dfs_.empty() && bfs_.empty();
}

Size CappedBestFirstDivingNodeSelection::size() const
{
    ZoneScopedC(TRACY_COLOUR);

    return dfs_.size() + bfs_.size();
}

void CappedBestFirstDivingNodeSelection::clear()
{
    ZoneScopedC(TRACY_COLOUR);

    dfs_.clear();
    bfs_.clear();
}

const SharedPtr<BBNode>& CappedBestFirstDivingNodeSelection::top() const
{
    ZoneScopedC(TRACY_COLOUR);

    if (dfs_.empty())
    {
        debug_assert(!bfs_.empty());
        subtree_root_ = std::move(bfs_.top());
        bfs_.pop();
        dfs_.push_back(subtree_root_);
        subtree_size_ = 0;
        // println("New subtree root node {}, lb {} from the priority queue", subtree_root_->id, subtree_root_->lb);
    }
    else
    {
        // println("Top stack node {}, lb {}", dfs_.back()->id, dfs_.back()->lb);
    }
    return dfs_.back();
}

void CappedBestFirstDivingNodeSelection::pop()
{
    ZoneScopedC(TRACY_COLOUR);

    debug_assert(!dfs_.empty());
    dfs_.pop_back();
}

void CappedBestFirstDivingNodeSelection::push(SharedPtr<BBNode>&& node)
{
    ZoneScopedC(TRACY_COLOUR);

    debug_assert(subtree_root_);
    if (is_gt(node->lb, eps_ceil(subtree_root_->lb) + BEST_FIRST_DIVING_LB_DIFFERENCE) ||
        subtree_size_ >= BEST_FIRST_DIVING_CAP)
    {
        // if (subtree_root_)
        // {
        //     println("Pushing node {}, lb {} on to the priority queue with root {}, lb {}",
        //             node->id, node->lb,
        //             subtree_root_ ? subtree_root_->id : INF, subtree_root_ ? subtree_root_->lb : INF);
        // }
        // else
        // {
        //     println("Pushing node {} with lb {} on to the priority queue without root",
        //             node->id, node->lb,
        //             subtree_root_ ? subtree_root_->id : INF, subtree_root_ ? subtree_root_->lb : INF);
        // }
        bfs_.push(std::move(node));
    }
    else
    {
        // if (subtree_root_)
        // {
        //     println("Pushing node {}, lb {} on to the stack with root {}, lb {}",
        //             node->id, node->lb,
        //             subtree_root_ ? subtree_root_->id : INF, subtree_root_ ? subtree_root_->lb : INF);
        // }
        // else
        // {
        //     println("Pushing node {}, lb {} on to the stack without root",
        //             node->id, node->lb,
        //             subtree_root_ ? subtree_root_->id : INF, subtree_root_ ? subtree_root_->lb : INF);
        // }
        dfs_.push_back(std::move(node));
        ++subtree_size_;
    }
}

void CappedBestFirstDivingNodeSelection::push_root(SharedPtr<BBNode>&& node)
{
    ZoneScopedC(TRACY_COLOUR);

    bfs_.push(std::move(node));
}

Cost CappedBestFirstDivingNodeSelection::lb() const
{
    ZoneScopedC(TRACY_COLOUR);

    return std::min(!dfs_.empty() ? dfs_.front()->lb : INF, !bfs_.empty() ? bfs_.top()->lb : INF);
}

// ---------------------------------------------------------------------------------------------------------------------

BoundTargetedNodeSelection::BoundTargetedNodeSelection(const Instance&, Problem& problem) noexcept :
    problem_(problem),

    dual_phase_(false),
    bfs_(),
    dfs_(),
    subtree_root_(),
    subtree_size_(0)
{
}

Bool BoundTargetedNodeSelection::empty() const
{
    ZoneScopedC(TRACY_COLOUR);

    return dfs_.empty() && bfs_.empty();
}

Size BoundTargetedNodeSelection::size() const
{
    ZoneScopedC(TRACY_COLOUR);

    return dfs_.size() + bfs_.size();
}

void BoundTargetedNodeSelection::clear()
{
    ZoneScopedC(TRACY_COLOUR);

    dfs_.clear();
    bfs_.clear();
}

void BoundTargetedNodeSelection::push(SharedPtr<BBNode>&& node)
{
    ZoneScopedC(TRACY_COLOUR);

    if (dual_phase_ ||
        !subtree_root_ ||
        is_gt(node->lb, eps_ceil(subtree_root_->lb) + BEST_FIRST_DIVING_LB_DIFFERENCE) ||
        subtree_size_ >= BEST_FIRST_DIVING_CAP)
    {
        // if (subtree_root_)
        // {
        //     println("Pushing node {}, lb {} on to the priority queue with root {}, lb {}",
        //             node->id, node->lb,
        //             subtree_root_ ? subtree_root_->id : INF, subtree_root_ ? subtree_root_->lb : INF);
        // }
        // else
        // {
        //     println("Pushing node {} with lb {} on to the priority queue without root",
        //             node->id, node->lb,
        //             subtree_root_ ? subtree_root_->id : INF, subtree_root_ ? subtree_root_->lb : INF);
        // }
        bfs_.push(std::move(node));
    }
    else
    {
        // if (subtree_root_)
        // {
        //     println("Pushing node {}, lb {} on to the stack with root {}, lb {}",
        //             node->id, node->lb,
        //             subtree_root_ ? subtree_root_->id : INF, subtree_root_ ? subtree_root_->lb : INF);
        // }
        // else
        // {
        //     println("Pushing node {}, lb {} on to the stack without root",
        //             node->id, node->lb,
        //             subtree_root_ ? subtree_root_->id : INF, subtree_root_ ? subtree_root_->lb : INF);
        // }
        dfs_.push_back(std::move(node));
        ++subtree_size_;
    }
}

const SharedPtr<BBNode>& BoundTargetedNodeSelection::top() const
{
    ZoneScopedC(TRACY_COLOUR);

    if (!dual_phase_ && problem_.gap() < BFS_SWITCH_GAP / 100.0)
    {
        for (auto& node : dfs_)
        {
            bfs_.push(std::move(node));
        }
        dfs_.clear();
        dual_phase_ = true;
        // println("Switching to dual phase");
    }

    if (dual_phase_)
    {
        return bfs_.top();
    }
    else
    {
        if (dfs_.empty())
        {
            debug_assert(!bfs_.empty());
            subtree_root_ = std::move(bfs_.top());
            bfs_.pop();
            dfs_.push_back(subtree_root_);
            subtree_size_ = 0;
            // println("New subtree root node {}, lb {} from the priority queue", subtree_root_->id, subtree_root_->lb);
        }
        else
        {
            // println("Top stack node {}, lb {}", dfs_.back()->id, dfs_.back()->lb);
        }
        return dfs_.back();
    }
}

void BoundTargetedNodeSelection::pop()
{
    ZoneScopedC(TRACY_COLOUR);

    if (dual_phase_)
    {
        debug_assert(!bfs_.empty());
        bfs_.pop();
    }
    else
    {
        debug_assert(!dfs_.empty());
        dfs_.pop_back();
    }
}

Cost BoundTargetedNodeSelection::lb() const
{
    ZoneScopedC(TRACY_COLOUR);

    return std::min(!dfs_.empty() ? dfs_.front()->lb : INF, !bfs_.empty() ? bfs_.top()->lb : INF);
}
