#pragma once

#include "sait/hypergraph.hpp"
#include "sait/cutwidth_analysis.hpp"
#include <vector>
#include <chrono>
#include <random>
#include <algorithm>
#include <cmath>
#include <deque>

/**
 * @brief Parameters for peak cutwidth optimization
 */
struct PeakCutwidthParams {
    int max_iterations = 20;             // Maximum refinement passes (recommended: 20)
    bool verbose = false;                // Print progress information
    bool use_random_start = false;      // Start from random instead of Fiedler
    double time_limit_seconds = 300.0;  // Time limit for refinement
    
    // Enhanced refinement parameters (optimized based on comprehensive analysis)
    int max_swap_distance = 1;      // Maximum distance for vertex swaps (recommended: 1 = adjacent only)
    int max_block_size = 1;         // Maximum size of vertex blocks to move (recommended: 1 = no block moves)
    bool use_hyperedge_aware = true; // Use hyperedge-aware move selection (recommended: true - PRIMARY ENHANCEMENT)
    double hyperedge_weight = 2.0;  // Weight for hyperedges contributing to peak
};

/**
 * @brief Result of peak cutwidth optimization
 */
struct PeakCutwidthResult {
    std::vector<int> ordering;           // Final vertex ordering
    std::vector<int> cutwidth_curve;     // Cut size at each position
    int peak_cutwidth;                   // Maximum cut value
    int peak_position;                   // Position of peak cut
    int initial_peak;                    // Peak before refinement
    int final_peak;                      // Peak after refinement
    int iterations_performed;           // Number of refinement passes
    int swaps_performed;                 // Total number of beneficial swaps
    double computation_time_ms;          // Total time
    double refinement_time_ms;           // Time spent in refinement
    bool converged;                      // Whether algorithm converged
    
    PeakCutwidthResult() : peak_cutwidth(0), peak_position(0), initial_peak(0), 
                          final_peak(0), iterations_performed(0), swaps_performed(0),
                          computation_time_ms(0.0), refinement_time_ms(0.0), converged(false) {}
};

/**
 * @brief Compute peak cutwidth minimization ordering
 * @param hg Input hypergraph
 * @param params Optimization parameters
 * @return Complete optimization result
 */
PeakCutwidthResult computePeakCutwidthOrdering(
    const Hypergraph& hg, 
    const PeakCutwidthParams& params = PeakCutwidthParams()
);

/**
 * @brief Efficient tracker for hyperedge cuts during vertex swaps
 */
class HypergraphCutwidthTracker {
private:
    // For each hyperedge, track its contribution to cuts
    struct HyperedgeInfo {
        int min_pos;                     // Minimum position of vertices in hyperedge
        int max_pos;                     // Maximum position of vertices in hyperedge
        
        // A hyperedge cuts at positions [min_pos+1, max_pos]
        int first_cut_pos() const { return min_pos + 1; }
        int last_cut_pos() const { return max_pos; }
        bool cuts_at_position(int k) const { 
            return k >= first_cut_pos() && k <= last_cut_pos(); 
        }
        bool is_cutting() const { return max_pos > min_pos; }
    };

protected:
    const Hypergraph& hg;
    std::vector<int> ordering;        // Current vertex ordering
    std::vector<int> position;        // position[v] = position of vertex v in ordering
    std::vector<int> cut_sizes;       // cut_sizes[k-1] = cutwidth at position k
    std::vector<HyperedgeInfo> hyperedge_info;  // Info for each hyperedge
    int peak_cut;                     // Current peak cutwidth
    int peak_position;                // Position of peak cutwidth
    
    // Helper methods
    void updateHyperedgeInfo(int hyperedge_id);  // Update info for one hyperedge
    void recomputeAllCutSizes();         // Full recomputation
    void updatePeakInfo();               // Update peak cutwidth and position
    
public:
    explicit HypergraphCutwidthTracker(const Hypergraph& hypergraph) : hg(hypergraph) {}
    
    // Main interface methods
    void initialize(const std::vector<int>& ordering);
    int evaluateSwap(int pos_k);
    void performSwap(int pos_k);
    
    // Enhanced move evaluation methods
    int evaluateDistantSwap(int pos1, int pos2);
    int evaluateBlockMove(int start_pos, int block_size, int target_pos);
    std::vector<int> getHyperedgesAtPeak();
    std::vector<std::pair<int, int>> getPrioritySwapCandidates();
    
    // Block move helper methods
    void performBlockMove(int start_pos, int block_size, int target_pos);
    bool isValidBlockMove(int start_pos, int block_size, int target_pos);
    
    // Distant swap helper method
    void performDistantSwap(int pos1, int pos2);
    
    // Getters
    int getPeakCutwidth() const { return peak_cut; }
    int getPeakPosition() const { return peak_position; }
    const std::vector<int>& getOrdering() const { return ordering; }
    const std::vector<int>& getCutwidthCurve() const { return cut_sizes; }
};

// Enhanced refinement function that takes an initial ordering
PeakCutwidthResult computeEnhancedRefinement(const Hypergraph& hg, const std::vector<int>& initial_ordering, const PeakCutwidthParams& params = PeakCutwidthParams());

/**
 * @brief Parameters for simulated annealing optimization
 */
struct AnnealingParams {
    // Annealing-specific parameters
    double initial_temperature = 50.0;     // T_init
    double cooling_rate = 0.95;            // alpha (0.95-0.99)
    int max_annealing_iterations = 30;     // Outer annealing loops
    int candidates_per_iteration = 1000;   // Limit candidates per iteration
    
    // Acceptance criteria
    double min_acceptance_rate = 0.05;     // Stop if acceptance drops below 5%
    bool adaptive_temperature = true;      // Adjust T based on acceptance rate
    
    // LSMC parameters
    bool use_large_steps = true;           // Enable Large Step Markov Chains
    double large_step_probability = 0.3;   // Probability of large step moves
    int max_block_size = 15;               // Maximum block size for large steps
    int max_segment_length = 30;           // Maximum segment length for reversals
    
    // Integration with existing system
    bool use_hyperedge_aware = true;       // Use hyperedge-aware selection
    int max_swap_distance = 1;             // Distance for small steps
    bool verbose = false;                  // Print progress information
};

/**
 * @brief Result of simulated annealing optimization
 */
struct AnnealingResult {
    std::vector<int> ordering;             // Final vertex ordering
    std::vector<int> cutwidth_curve;       // Cut size at each position
    int initial_peak;                      // Peak before annealing
    int final_peak;                        // Peak after annealing
    int best_peak_seen;                    // Global minimum found
    
    // Annealing-specific metrics
    int total_swaps_evaluated;             // Total moves evaluated
    int beneficial_swaps;                  // Improving moves
    int neutral_swaps;                     // Neutral moves accepted
    int harmful_swaps_accepted;            // Worsening moves accepted
    int large_steps_attempted;             // LSMC moves attempted
    int large_steps_accepted;              // LSMC moves accepted
    
    double final_temperature;             // Final temperature
    double average_acceptance_rate;        // Overall acceptance rate
    
    // Performance tracking
    std::vector<int> peak_history;         // Peak at each iteration
    std::vector<double> temperature_history; // Temperature evolution
    double computation_time_ms;            // Total time
    bool converged;                        // Whether algorithm converged
    
    AnnealingResult() : initial_peak(0), final_peak(0), best_peak_seen(0),
                       total_swaps_evaluated(0), beneficial_swaps(0), neutral_swaps(0),
                       harmful_swaps_accepted(0), large_steps_attempted(0), large_steps_accepted(0),
                       final_temperature(0.0), average_acceptance_rate(0.0),
                       computation_time_ms(0.0), converged(false) {}
};

/**
 * @brief Move types for Large Step Markov Chains
 */
enum class MoveType {
    SMALL_STEP,         // Traditional adjacent swaps
    BLOCK_MOVE,         // Move contiguous blocks
    HYPEREDGE_CLUSTER,  // Cluster hyperedge vertices
    SEGMENT_REVERSAL,   // Reverse segments
    RANDOM_SHUFFLE      // Shuffle random subset
};

/**
 * @brief Enhanced tracker with incremental updates for annealing
 */
class IncrementalHyperedgeTracker : public HypergraphCutwidthTracker {
private:
    // Cache for peak-contributing hyperedges
    std::vector<int> cached_peak_hyperedges;
    int cached_peak_position;
    bool cache_valid;
    
    // LSMC move generation
    std::vector<double> move_probabilities;
    
public:
    explicit IncrementalHyperedgeTracker(const Hypergraph& hypergraph);
    
    // Efficient updates after moves
    void updateAfterSwap(int pos1, int pos2);
    void updateAfterBlockMove(int start_pos, int block_size, int target_pos);
    void updateAfterSegmentReversal(int start_pos, int end_pos);
    
    // LSMC move generation
    MoveType selectMoveType(double temperature);
    std::pair<int, int> generateBlockMove(double temperature);
    std::pair<int, int> generateSegmentReversal(double temperature);
    std::vector<int> generateHyperedgeClusterMove();
    
    // Cached candidate generation
    std::vector<std::pair<int, int>> getCachedPrioritySwapCandidates();
    
    // Cache management
    void invalidateCache();
    void refreshCache();
};

/**
 * @brief Compute simulated annealing optimization
 * @param hg Input hypergraph
 * @param initial_ordering Starting vertex ordering
 * @param params Annealing parameters
 * @return Complete annealing result
 */
AnnealingResult computeSimulatedAnnealing(
    const Hypergraph& hg,
    const std::vector<int>& initial_ordering,
    const AnnealingParams& params = AnnealingParams()
); 