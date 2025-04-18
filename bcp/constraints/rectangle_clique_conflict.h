/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "constraints/separator.h"
#include "master/constraint.h"
#include "types/array.h"
#include "types/hash_map.h"
#include "types/matrix.h"
#include "types/memory_pool.h"
#include "types/tuple.h"
#include "types/vector.h"

class Map;
class Projection;
struct MDDNode;

class RectangleCliqueConflictSeparator : public Separator
{
  public:
    struct RectangleCliqueConstraint : public Constraint
    {
        Agent a1;
        Agent a2;
        EdgeTime a1_first_entry;
        EdgeTime a1_first_exit;
        Time a1_length;
        Node a1_n_increment;
        EdgeTime a2_first_entry;
        EdgeTime a2_first_exit;
        Time a2_length;
        Node a2_n_increment;
    };

  private:
    struct RectangleCliqueConstraintCandidate
    {
        Float lhs;
        Time area;
        Agent a1;
        Agent a2;
        EdgeTime a1_first_entry;
        EdgeTime a1_first_exit;
        Time a1_length;
        Node a1_n_increment;
        EdgeTime a2_first_entry;
        EdgeTime a2_first_exit;
        Time a2_length;
        Node a2_n_increment;
    };

    // Helper data
    MemoryPool memory_pool_;
    Vector<HashMap<NodeTime, MDDNode*>> agent_mdd_;
    HashMap<NodeTime, MDDNode*> mdd_next_agent_;
    Vector<RectangleCliqueConstraintCandidate> candidates_;
    Matrix<UInt8> num_separated_;

  public:
    // Constructors and destructor
    using Separator::Separator;
    RectangleCliqueConflictSeparator(const Instance& instance, Problem& problem);

    // Separator type
    constexpr static auto name() { return "Rectangle clique"; }

    // Separate
    void separate();

    // Add dual solution to pricing costs
    void add_pricing_costs(const Constraint& constraint, const Float dual);

    // Add coefficient to a column
    Float get_coeff(const Constraint& constraint, const Agent a, const Path& path);

  private:
    static Array<Pair<Direction, Direction>, 4> rectangle_directions()
    {
        return {Pair<Direction, Direction>{Direction::WEST, Direction::NORTH},
                Pair<Direction, Direction>{Direction::EAST, Direction::NORTH},
                Pair<Direction, Direction>{Direction::WEST, Direction::SOUTH},
                Pair<Direction, Direction>{Direction::EAST, Direction::SOUTH}};
    }
    Bool calculate_coeff(const EdgeTime first_et, const Time length, const Node n_increment, const Path& path);
    void create_row(const Agent a1,
                    const Agent a2,
                    EdgeTime a1_first_entry,
                    EdgeTime a1_first_exit,
                    Time a1_length,
                    const Node a1_n_increment,
                    EdgeTime a2_first_entry,
                    EdgeTime a2_first_exit,
                    Time a2_length,
                    const Node a2_n_increment);
};
