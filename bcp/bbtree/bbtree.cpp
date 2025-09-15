/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "bbtree/bbtree.h"
#include "bbtree/brancher.h"
#include "types/debug.h"
#include "types/tracy.h"

#define TRACY_COLOUR tracy::Color::ColorType::SpringGreen

BBTree::BBTree(const Instance& instance, Problem& problem) noexcept :
    node_sel_(instance, problem),

    current_(),
    num_closed_(-1),
    next_node_id_(0)
{
    ZoneScopedC(TRACY_COLOUR);

    // Create the root node.
    auto root = std::make_shared<BBNode>();
    root->id = next_node_id_++;
    root->depth = 0;
    root->lb = -COST_INF;
    root->brancher = nullptr;

    // Insert the root into the tree.
    node_sel_.push_root(std::move(root));
}

Bool BBTree::next()
{
    ZoneScopedC(TRACY_COLOUR);

    ++num_closed_;
    if (!node_sel_.empty())
    {
        current_ = node_sel_.top();
        node_sel_.pop();
        return true;
    }
    else
    {
        current_ = nullptr;
        return false;
    }
}

void BBTree::update_node_lb(const Cost lb)
{
    ZoneScopedC(TRACY_COLOUR);

    DEBUG_ASSERT(current_);
    current_->lb = std::max(current_->lb, lb);
}

void BBTree::branch(Brancher* brancher, Decisions&& decisions)
{
    ZoneScopedC(TRACY_COLOUR);

    DEBUG_ASSERT(current_);

    // Create the first child.
    {
        auto child = std::make_shared<BBNode>();
        child->id = next_node_id_++;
        child->depth = current_->depth + 1;
        child->lb = current_->lb;
        child->parent = current_;
        child->brancher = brancher;
        child->decision = std::move(decisions.first);
        child->variables = current_->variables;
        child->constraints = current_->constraints;
        DEBUGLN("Created B&B node {} from first decision", child->id);
        node_sel_.push(std::move(child));
    }

    // Create the second child.
    {
        auto child = std::make_shared<BBNode>();
        child->id = next_node_id_++;
        child->depth = current_->depth + 1;
        child->lb = current_->lb;
        child->parent = current_;
        child->brancher = brancher;
        child->decision = std::move(decisions.second);
        child->variables = current_->variables;
        child->constraints = current_->constraints;
        DEBUGLN("Created B&B node {} from second decision", child->id);
        node_sel_.push(std::move(child));
    }
}

Cost BBTree::lb() const
{
    ZoneScopedC(TRACY_COLOUR);

    return std::min(current_ ? current_->lb : COST_INF, node_sel_.lb());
}

Vector<Pair<Brancher*, BrancherData*>> BBTree::all_decisions() const
{
    ZoneScopedC(TRACY_COLOUR);

    DEBUG_ASSERT(current_);

    Vector<Pair<Brancher*, BrancherData*>> decisions;
    decisions.reserve(current_->depth);
    for (auto node = current_.get(); node->parent; node = node->parent.get())
    {
        decisions.push_back({node->brancher, node->decision.get()});
    }
    DEBUG_ASSERT(decisions.size() == current_->depth);
    return decisions;
}
