/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#ifdef USE_SHARED_TIME_EXPANDED_ASTAR_PRICER

#pragma once

#include "problem/map.h"
#include "types/arena.h"
#include "types/dary_heap.h"
#include "types/hash_map.h"
#include "types/map_types.h"
#include "types/pointers.h"

class DistanceHeuristic;
class EdgeTimePenalties;
class FinishTimePenalties;
class OnceOffPenalties;
class Problem;
class RectanglePenalties;
struct Instance;

class SharedTimeExpandedAStar
{
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

        static const Size64 data_size = 8 * 5 + 2 * 2 + 4 * 1;
        static const Size64 padding_size = 0;
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

        static const Size64 data_size = 8 * 4 + 4 * 1 + 2 * 3;
        static const Size64 padding_size = 6;
    };
    static_assert(sizeof(InfeasibleLabel) ==
                  InfeasibleLabel::data_size + InfeasibleLabel::padding_size);

    // Priority queue data structures
    template <Bool feasible>
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
    template <class Label>
    class PriorityQueue : public DAryHeap<Label*, LabelComparison, PriorityQueueSizeType>
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
    Agent a_;
    NodeTime start_;
    Node target_;

    // Branching decisions
    Vector<NodeTime>* waypoints_;
    Vector<Time> h_waypoint_to_target_;
    Size64 next_waypoint_index_;
    Time waypoint_time_;
    const Time* h_to_waypoint_;

    // Costs
    Cost constant_;
    const EdgeTimePenalties* edgetime_penalties_;
    const FinishTimePenalties* finish_time_penalties_;
    const OnceOffPenalties* once_off_penalties_;
    const RectanglePenalties* rectangle_penalties_;
    Size64 once_off_bitset_size_;
    Size64 rectangle_bitset_size_;
    const Time* latest_visit_time_;
    Time earliest_target_time_;
    Time latest_target_time_;

    // Solver state
    Arena label_storage_;
    UniquePtr<HashMap<NodeTime, void*>[]> closed_;
    PriorityQueue<InfeasibleLabel> open_infeasible_;
    PriorityQueue<FeasibleLabel> open_feasible_;
    Cost obj_;
#ifdef USE_SHARED_TIME_EXPANDED_ASTAR_PRICER
    Size64& num_added_;
#endif

  public:
    // Constructors and destructor
    SharedTimeExpandedAStar() = delete;
    SharedTimeExpandedAStar(const Instance& instance, Problem& problem,
                            DistanceHeuristic& distance_heuristic, Size64& num_added);
    ~SharedTimeExpandedAStar() = default;
    SharedTimeExpandedAStar(const SharedTimeExpandedAStar&) noexcept = delete;
    SharedTimeExpandedAStar(SharedTimeExpandedAStar&&) noexcept = default;
    SharedTimeExpandedAStar& operator=(const SharedTimeExpandedAStar&) noexcept = delete;
    SharedTimeExpandedAStar& operator=(SharedTimeExpandedAStar&&) noexcept = delete;

    // Solve
    template <Bool feasible>
    Cost solve(const Agent a, Vector<NodeTime>& waypoints, const Real64 constant,
               const EdgeTimePenalties& edgetime_penalties,
               FinishTimePenalties& finish_time_penalties,
               const OnceOffPenalties& once_off_penalties,
               const RectanglePenalties& rectangle_penalties, const Time* const latest_visit_time,
               const Time earliest_target_time, const Time latest_target_time);

    // Debug
#ifdef DEBUG
    void set_verbose(const Bool verbose = true);
#endif

  protected:
    // Set up
    void reset();

    // Constants
    constexpr static inline Node end_n()
    {
        return -1;
    }
    constexpr static inline Bool is_end(const NodeTime nt)
    {
        return nt.n == end_n();
    }
    template <Bool feasible>
    constexpr static inline Time default_edge_cost()
    {
        return static_cast<Time>(feasible);
    }

    // State generation
    template <Bool feasible>
    Byte* get_once_off_bitset(Label<feasible>* const label);
    template <Bool feasible>
    Byte* get_rectangle_bitset(Label<feasible>* const label);
#ifdef USE_RESERVATION_LOOKUP
    Bool calculate_reservation(const NodeTime nt);
#endif
    template <Bool feasible, Bool towards_end>
    void generate_start();
    template <Bool feasible, Bool towards_end>
    void generate_nodetime(const NodeTime next_nt, Label<feasible>* current, const Direction d,
                           const Cost edgetime_penalty);
    template <Bool feasible>
    void generate_end(Label<feasible>* current);
    template <Bool feasible, Bool towards_end>
    void expand(Label<feasible>* current);
    template <Bool feasible>
    void add_path(Label<feasible>* end);

    // Queue functions
    template <Bool feasible>
    constexpr inline auto& open()
    {
        if constexpr (feasible)
        {
            return open_feasible_;
        }
        else
        {
            return open_infeasible_;
        }
    }
    template <Bool feasible>
    Label<feasible>* push(Label<feasible>* next);
};

#endif
