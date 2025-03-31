/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "pricing/independent_intervals.h"
#include "pricing/once_off_penalties.h"
#include "pricing/rectangle_penalties.h"
#include "types/astar_priority_queue.h"
#include "types/hash_map.h"
#include "types/map_types.h"
#include "types/memory_pool.h"
#ifdef CHECK_USING_ASTAR
#include "pricing/independent_time_expanded_astar.h"
#endif

class Problem;
class DistanceHeuristic;
struct Instance;

template<Bool feasible>
class IndependentTimeIntervalAStar
{
    static constexpr Cost default_edge_cost = feasible ? 1.0 : 0.0;

    // Label associated with a state
    using PriorityQueueSizeType = Size32;
    struct FeasibleLabel
    {
        NodeTime nt;
        FeasibleLabel* parent;
        FeasibleLabel* next;
        Cost f;
        Cost g;
        UInt16 num_bitset_ones; // TODO: unused
        UInt16 reservations;
        PriorityQueueSizeType pq_index;
        Byte bitset[];

        static const Size data_size = 8*5 + 2*2 + 4*1;
        static const Size padding_size = 0;
    };
    static_assert(sizeof(FeasibleLabel) == FeasibleLabel::data_size + FeasibleLabel::padding_size);
    struct InfeasibleLabel
    {
        NodeTime nt;
        InfeasibleLabel* parent;
        InfeasibleLabel* next;
        Cost g;
        PriorityQueueSizeType pq_index;
        UInt16 time_f;
        UInt16 num_bitset_ones; // TODO: unused
        UInt16 reservations;
        Byte bitset[];

        static const Size data_size = 8*4 + 4*1 + 2*3;
        static const Size padding_size = 6;
    };
    static_assert(sizeof(InfeasibleLabel) == InfeasibleLabel::data_size + InfeasibleLabel::padding_size);

    // Priority queue data structures
    using Label = typename std::conditional<feasible, FeasibleLabel, InfeasibleLabel>::type;
    struct LabelComparison
    {
        static inline Bool lt(const FeasibleLabel* const lhs, const FeasibleLabel* const rhs)
        {
            return std::tie(lhs->f, lhs->reservations, rhs->g) <
                   std::tie(rhs->f, rhs->reservations, lhs->g);
        }
        static inline Bool le(const FeasibleLabel* const lhs, const FeasibleLabel* const rhs)
        {
            return std::tie(lhs->f, lhs->reservations, rhs->g) <=
                   std::tie(rhs->f, rhs->reservations, lhs->g);
        }
        static inline Bool eq(const FeasibleLabel* const lhs, const FeasibleLabel* const rhs)
        {
            return std::tie(lhs->f, lhs->reservations, rhs->g) ==
                   std::tie(rhs->f, rhs->reservations, lhs->g);
        }

        static inline Bool lt(const InfeasibleLabel* const lhs, const InfeasibleLabel* const rhs)
        {
            return std::tie(lhs->g, lhs->time_f, lhs->reservations, rhs->nt.t) <
                   std::tie(rhs->g, rhs->time_f, rhs->reservations, lhs->nt.t);
        }
        static inline Bool le(const InfeasibleLabel* const lhs, const InfeasibleLabel* const rhs)
        {
            return std::tie(lhs->g, lhs->time_f, lhs->reservations, rhs->nt.t) <=
                   std::tie(rhs->g, rhs->time_f, rhs->reservations, lhs->nt.t);
        }
        static inline Bool eq(const InfeasibleLabel* const lhs, const InfeasibleLabel* const rhs)
        {
            return std::tie(lhs->g, lhs->time_f, lhs->reservations, rhs->nt.t) ==
                   std::tie(rhs->g, rhs->time_f, rhs->reservations, lhs->nt.t);
        }
    };
    class PriorityQueue : public AStarPriorityQueue<Label*, LabelComparison, PriorityQueueSizeType>
    {
      public:
        // Modify the handle in the label pointing to its position in the priority queue
        void update_index(Label* label, const PriorityQueueSizeType index)
        {
            label->pq_index = index;
        }

        // Check the validity of an index
        Bool check_index(Label* const& label, const PriorityQueueSizeType index) const
        {
            return (label->pq_index == index);
        }
    };

    // Problem
    const Instance& instance_;
    const Map& map_;
    Problem& problem_;
    DistanceHeuristic& distance_heuristic_;

    // Data for the pricing problem
    const Agent a_;
    NodeTime start_;
    Node target_;

    // Branching decisions
    Vector<NodeTime> waypoints_;
    Vector<Time> h_waypoint_to_target_;
    Size next_waypoint_index_;
    Time waypoint_time_;
    const Time* h_to_waypoint_;

    // Costs
    Cost constant_;
    IndependentIntervals intervals_;
    OnceOffPenalties once_off_penalties_;
    RectanglePenalties rectangle_penalties_;
    Size once_off_bitset_size_;
    Size rectangle_bitset_size_;
    Time earliest_target_time_;
    Time latest_target_time_;

    // Solver state
    MemoryPool label_storage_;
    Label** closed_;
    PriorityQueue open_;
    Cost obj_;
#ifdef USE_INDEPENDENT_TIME_INTERVAL_ASTAR_PRICER
    Size& num_added_;
#endif

    // Debug
#ifdef CHECK_USING_ASTAR
    IndependentTimeExpandedAStar<feasible> astar_;
#endif

  public:
    // Constructors and destructor
    IndependentTimeIntervalAStar() = delete;
    IndependentTimeIntervalAStar(const Instance& instance,
                                 Problem& problem,
                                 DistanceHeuristic& distance_heuristic,
                                 const Agent a,
                                 Size& num_added);
    ~IndependentTimeIntervalAStar();
    IndependentTimeIntervalAStar(const IndependentTimeIntervalAStar&) noexcept = delete;
    IndependentTimeIntervalAStar(IndependentTimeIntervalAStar&&) noexcept = default;
    IndependentTimeIntervalAStar& operator=(const IndependentTimeIntervalAStar&) noexcept = delete;
    IndependentTimeIntervalAStar& operator=(IndependentTimeIntervalAStar&&) noexcept = delete;

    // Set up costs
    void add_waypoint(const NodeTime nt);
    void set_constant(const Cost constant);
    void add_nodetime_penalty(const NodeTime nt, const Cost cost);
    void add_edgetime_penalty(const EdgeTime et, const Cost cost);
    template<OnceOffDirection d = OnceOffDirection::GEq>
    void add_once_off_penalty(const NodeTime nt, const Cost cost);
    void add_rectangle_penalty(const EdgeTime first_entry,
                               const EdgeTime first_exit,
                               const Time length,
                               const Node n_increment,
                               const Cost cost);
    void add_end_penalty(const Time earliest, const Time latest, const Cost cost);

    // Solve
    Cost solve();

    // Debug
#ifdef DEBUG
    void set_verbose(const Bool verbose = true);
#endif

  protected:
    // Set up
    void reset();

    // Constants
    constexpr static inline Node end_n() { return -1; }
    constexpr static inline Bool is_end(const NodeTime nt) { return nt.n == end_n(); }

    // State generation
    Byte* get_once_off_bitset(Label* const label);
    Byte* get_rectangle_bitset(Label* const label);
#ifdef USE_RESERVATION_LOOKUP
    Bool calculate_reservation(const NodeTime nt);
#endif
    template<Bool towards_end>
    void generate_start();
    template<Bool towards_end>
    void generate_nodetimes(const Node next_n, const Direction d, Label* current);
    template<Bool towards_end>
    void generate_waypoint(Label* current);
    void generate_end(Label* current);
    template<Bool towards_end>
    void expand(Label* current);
    void add_path(Label* end);

    // Queue functions
    template<Bool is_wait_action>
    Label* push(Label* next);

    // Debug
    // String format_label(const Label* const label) const;
};
