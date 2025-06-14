#include "sait/peak_cutwidth_ordering.hpp"
#include "sait/fiedler_ordering.hpp"
#include "sait/random_ordering.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <unordered_set>
#include <limits>
#include <set>
#include <tuple>
#include <random>
#include <unordered_set>
#include <climits>
#include <iomanip>

// HypergraphCutwidthTracker implementation

void HypergraphCutwidthTracker::initialize(const std::vector<int>& initial_ordering) {
    if (static_cast<int>(initial_ordering.size()) != hg.num_vertices) {
        throw std::runtime_error("Ordering size doesn't match number of vertices");
    }
    
    // Initialize data structures
    ordering = initial_ordering;
    position.resize(hg.num_vertices);
    cut_sizes.resize(hg.num_vertices - 1, 0);
    hyperedge_info.resize(hg.num_hyperedges);
    
    // Build position mapping
    for (int k = 0; k < hg.num_vertices; ++k) {
        position[ordering[k]] = k;
    }
    
    // Initialize hyperedge information
    for (int he = 0; he < hg.num_hyperedges; ++he) {
        updateHyperedgeInfo(he);
    }
    
    // Compute initial cut sizes
    recomputeAllCutSizes();
    updatePeakInfo();
}

void HypergraphCutwidthTracker::updateHyperedgeInfo(int hyperedge_id) {
    const auto& vertices = hg.hyperedges[hyperedge_id];
    
    if (vertices.empty()) {
        hyperedge_info[hyperedge_id].min_pos = 0;
        hyperedge_info[hyperedge_id].max_pos = 0;
        return;
    }
    
    int min_pos = std::numeric_limits<int>::max();
    int max_pos = std::numeric_limits<int>::min();
    
    for (int vertex : vertices) {
        if (vertex >= 0 && vertex < hg.num_vertices) {
            int pos = position[vertex];
            min_pos = std::min(min_pos, pos);
            max_pos = std::max(max_pos, pos);
        }
    }
    
    hyperedge_info[hyperedge_id].min_pos = min_pos;
    hyperedge_info[hyperedge_id].max_pos = max_pos;
}

void HypergraphCutwidthTracker::recomputeAllCutSizes() {
    // Reset all cut sizes
    std::fill(cut_sizes.begin(), cut_sizes.end(), 0);
    
    // For each hyperedge, add its contribution to cuts
    for (int he = 0; he < hg.num_hyperedges; ++he) {
        const auto& info = hyperedge_info[he];
        
        if (info.is_cutting()) {
            // This hyperedge contributes to cuts at positions [first_cut_pos, last_cut_pos]
            int start_pos = info.first_cut_pos();
            int end_pos = info.last_cut_pos();
            
            for (int k = start_pos; k <= end_pos && k < static_cast<int>(cut_sizes.size()); ++k) {
                cut_sizes[k - 1]++;  // Convert to 0-indexed for cut_sizes array
            }
        }
    }
}

void HypergraphCutwidthTracker::updatePeakInfo() {
    if (cut_sizes.empty()) {
        peak_cut = 0;
        peak_position = 0;
        return;
    }
    
    auto max_it = std::max_element(cut_sizes.begin(), cut_sizes.end());
    peak_cut = *max_it;
    peak_position = std::distance(cut_sizes.begin(), max_it) + 1; // Convert to 1-indexed
}

int HypergraphCutwidthTracker::evaluateSwap(int pos_k) {
    if (pos_k < 0 || pos_k >= hg.num_vertices - 1) {
        return 0; // Invalid swap
    }
    
    int v1 = ordering[pos_k];      // Vertex currently at position k
    int v2 = ordering[pos_k + 1];  // Vertex currently at position k+1
    
    // Collect all hyperedges that might be affected
    std::unordered_set<int> affected_hyperedges;
    
    // Add hyperedges incident to v1
    for (int he : hg.vertex_to_hyperedges[v1]) {
        affected_hyperedges.insert(he);
    }
    
    // Add hyperedges incident to v2
    for (int he : hg.vertex_to_hyperedges[v2]) {
        affected_hyperedges.insert(he);
    }
    
    if (affected_hyperedges.empty()) {
        return 0; // No hyperedges affected
    }
    
    // Store original state
    int old_peak = peak_cut;
    std::vector<int> old_cut_sizes = cut_sizes;
    std::vector<HyperedgeInfo> old_hyperedge_info;
    
    // Save info for affected hyperedges
    for (int he : affected_hyperedges) {
        old_hyperedge_info.push_back(hyperedge_info[he]);
    }
    
    // Temporarily perform swap
    std::swap(position[v1], position[v2]);
    std::swap(ordering[pos_k], ordering[pos_k + 1]);
    
    // Update affected hyperedges and recompute their contributions
    for (int he : affected_hyperedges) {
        const auto& old_info = hyperedge_info[he];
        
        // Remove old contribution
        if (old_info.is_cutting()) {
            int start_pos = old_info.first_cut_pos();
            int end_pos = old_info.last_cut_pos();
            
            for (int k = start_pos; k <= end_pos && k < static_cast<int>(cut_sizes.size()); ++k) {
                cut_sizes[k - 1]--;
            }
        }
        
        // Update hyperedge info
        updateHyperedgeInfo(he);
        
        // Add new contribution
        const auto& new_info = hyperedge_info[he];
        if (new_info.is_cutting()) {
            int start_pos = new_info.first_cut_pos();
            int end_pos = new_info.last_cut_pos();
            
            for (int k = start_pos; k <= end_pos && k < static_cast<int>(cut_sizes.size()); ++k) {
                cut_sizes[k - 1]++;
            }
        }
    }
    
    // Find new peak
    auto max_it = std::max_element(cut_sizes.begin(), cut_sizes.end());
    int new_peak = (max_it != cut_sizes.end()) ? *max_it : 0;
    
    // Restore original state
    std::swap(position[v1], position[v2]);
    std::swap(ordering[pos_k], ordering[pos_k + 1]);
    cut_sizes = old_cut_sizes;
    
    // Restore hyperedge info
    auto info_it = old_hyperedge_info.begin();
    for (int he : affected_hyperedges) {
        hyperedge_info[he] = *info_it;
        ++info_it;
    }
    
    return new_peak - old_peak;  // Change in peak cutwidth
}

void HypergraphCutwidthTracker::performSwap(int pos_k) {
    if (pos_k < 0 || pos_k >= hg.num_vertices - 1) {
        return; // Invalid swap
    }
    
    int v1 = ordering[pos_k];      // Vertex currently at position k
    int v2 = ordering[pos_k + 1];  // Vertex currently at position k+1
    
    // Collect all hyperedges that might be affected
    std::unordered_set<int> affected_hyperedges;
    
    // Add hyperedges incident to v1
    for (int he : hg.vertex_to_hyperedges[v1]) {
        affected_hyperedges.insert(he);
    }
    
    // Add hyperedges incident to v2
    for (int he : hg.vertex_to_hyperedges[v2]) {
        affected_hyperedges.insert(he);
    }
    
    // Remove old contributions
    for (int he : affected_hyperedges) {
        const auto& old_info = hyperedge_info[he];
        
        if (old_info.is_cutting()) {
            int start_pos = old_info.first_cut_pos();
            int end_pos = old_info.last_cut_pos();
            
            for (int k = start_pos; k <= end_pos && k < static_cast<int>(cut_sizes.size()); ++k) {
                cut_sizes[k - 1]--;
            }
        }
    }
    
    // Perform the swap
    std::swap(position[v1], position[v2]);
    std::swap(ordering[pos_k], ordering[pos_k + 1]);
    
    // Update affected hyperedges and add new contributions
    for (int he : affected_hyperedges) {
        updateHyperedgeInfo(he);
        
        const auto& new_info = hyperedge_info[he];
        if (new_info.is_cutting()) {
            int start_pos = new_info.first_cut_pos();
            int end_pos = new_info.last_cut_pos();
            
            for (int k = start_pos; k <= end_pos && k < static_cast<int>(cut_sizes.size()); ++k) {
                cut_sizes[k - 1]++;
            }
        }
    }
    
    // Update peak information
    updatePeakInfo();
}

// Enhanced move evaluation methods

int HypergraphCutwidthTracker::evaluateDistantSwap(int pos1, int pos2) {
    if (pos1 < 0 || pos1 >= hg.num_vertices || pos2 < 0 || pos2 >= hg.num_vertices || pos1 == pos2) {
        return 0; // Invalid swap
    }
    
    // Ensure pos1 < pos2 for consistency
    if (pos1 > pos2) {
        std::swap(pos1, pos2);
    }
    
    int v1 = ordering[pos1];
    int v2 = ordering[pos2];
    
    // Collect all hyperedges that might be affected
    std::unordered_set<int> affected_hyperedges;
    
    for (int he : hg.vertex_to_hyperedges[v1]) {
        affected_hyperedges.insert(he);
    }
    for (int he : hg.vertex_to_hyperedges[v2]) {
        affected_hyperedges.insert(he);
    }
    
    if (affected_hyperedges.empty()) {
        return 0;
    }
    
    // Store original state
    int old_peak = peak_cut;
    std::vector<int> old_cut_sizes = cut_sizes;
    std::vector<HyperedgeInfo> old_hyperedge_info;
    
    for (int he : affected_hyperedges) {
        old_hyperedge_info.push_back(hyperedge_info[he]);
    }
    
    // Temporarily perform swap
    std::swap(position[v1], position[v2]);
    std::swap(ordering[pos1], ordering[pos2]);
    
    // Update affected hyperedges
    for (int he : affected_hyperedges) {
        const auto& old_info = hyperedge_info[he];
        
        // Remove old contribution
        if (old_info.is_cutting()) {
            int start_pos = old_info.first_cut_pos();
            int end_pos = old_info.last_cut_pos();
            
            for (int k = start_pos; k <= end_pos && k < static_cast<int>(cut_sizes.size()); ++k) {
                cut_sizes[k - 1]--;
            }
        }
        
        // Update hyperedge info
        updateHyperedgeInfo(he);
        
        // Add new contribution
        const auto& new_info = hyperedge_info[he];
        if (new_info.is_cutting()) {
            int start_pos = new_info.first_cut_pos();
            int end_pos = new_info.last_cut_pos();
            
            for (int k = start_pos; k <= end_pos && k < static_cast<int>(cut_sizes.size()); ++k) {
                cut_sizes[k - 1]++;
            }
        }
    }
    
    // Find new peak
    auto max_it = std::max_element(cut_sizes.begin(), cut_sizes.end());
    int new_peak = (max_it != cut_sizes.end()) ? *max_it : 0;
    
    // Restore original state
    std::swap(position[v1], position[v2]);
    std::swap(ordering[pos1], ordering[pos2]);
    cut_sizes = old_cut_sizes;
    
    auto info_it = old_hyperedge_info.begin();
    for (int he : affected_hyperedges) {
        hyperedge_info[he] = *info_it;
        ++info_it;
    }
    
    return new_peak - old_peak;
}

std::vector<int> HypergraphCutwidthTracker::getHyperedgesAtPeak() {
    std::vector<int> peak_hyperedges;
    
    // Find the peak position(s)
    int peak_pos = getPeakPosition();
    
    // Find all hyperedges that contribute to the peak cutwidth
    for (int he = 0; he < hg.num_hyperedges; ++he) {
        const auto& info = hyperedge_info[he];
        if (info.is_cutting()) {
            int start_pos = info.first_cut_pos();
            int end_pos = info.last_cut_pos();
            
            // Check if this hyperedge contributes to the peak position
            if (start_pos <= peak_pos && peak_pos <= end_pos) {
                peak_hyperedges.push_back(he);
            }
        }
    }
    
    return peak_hyperedges;
}

std::vector<std::pair<int, int>> HypergraphCutwidthTracker::getPrioritySwapCandidates() {
    std::vector<std::pair<int, int>> candidates;
    
    // Get hyperedges contributing to the peak
    std::vector<int> peak_hyperedges = getHyperedgesAtPeak();
    
    // Collect vertices involved in peak hyperedges
    std::unordered_set<int> priority_vertices;
    for (int he : peak_hyperedges) {
        for (int v : hg.hyperedges[he]) {
            priority_vertices.insert(v);
        }
    }
    
    // Generate swap candidates involving priority vertices
    for (int v : priority_vertices) {
        int pos = position[v];
        
        // Add adjacent swaps
        if (pos > 0) {
            candidates.emplace_back(pos - 1, pos);
        }
        if (pos < hg.num_vertices - 1) {
            candidates.emplace_back(pos, pos + 1);
        }
        
        // Add distant swaps with other priority vertices
        for (int v2 : priority_vertices) {
            if (v != v2) {
                int pos2 = position[v2];
                if (abs(pos - pos2) > 1) {  // Only distant swaps
                    candidates.emplace_back(std::min(pos, pos2), std::max(pos, pos2));
                }
            }
        }
    }
    
    // Remove duplicates
    std::sort(candidates.begin(), candidates.end());
    candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
    
    return candidates;
}

int HypergraphCutwidthTracker::evaluateBlockMove(int start_pos, int block_size, int target_pos) {
    if (!isValidBlockMove(start_pos, block_size, target_pos)) {
        return 0;
    }
    
    // Store original state
    int old_peak = peak_cut;
    std::vector<int> old_ordering = ordering;
    std::vector<int> old_position = position;
    std::vector<int> old_cut_sizes = cut_sizes;
    std::vector<HyperedgeInfo> old_hyperedge_info = hyperedge_info;
    
    // Perform the block move temporarily
    performBlockMove(start_pos, block_size, target_pos);
    
    // Recompute all cutwidth information after the move
    recomputeAllCutSizes();
    updatePeakInfo();
    
    int new_peak = peak_cut;
    
    // Restore original state
    ordering = old_ordering;
    position = old_position;
    cut_sizes = old_cut_sizes;
    hyperedge_info = old_hyperedge_info;
    peak_cut = old_peak;
    updatePeakInfo();
    
    return new_peak - old_peak;
}

bool HypergraphCutwidthTracker::isValidBlockMove(int start_pos, int block_size, int target_pos) {
    if (start_pos < 0 || block_size <= 0 || target_pos < 0) {
        return false;
    }
    
    if (start_pos + block_size > hg.num_vertices) {
        return false;
    }
    
    if (target_pos + block_size > hg.num_vertices) {
        return false;
    }
    
    // Don't move to the same position
    if (target_pos == start_pos) {
        return false;
    }
    
    // Don't allow overlapping moves
    if (target_pos > start_pos && target_pos < start_pos + block_size) {
        return false;
    }
    
    if (target_pos < start_pos && target_pos + block_size > start_pos) {
        return false;
    }
    
    return true;
}

void HypergraphCutwidthTracker::performBlockMove(int start_pos, int block_size, int target_pos) {
    if (!isValidBlockMove(start_pos, block_size, target_pos)) {
        return;
    }
    
    // Extract the block to be moved
    std::vector<int> block;
    for (int i = 0; i < block_size; ++i) {
        block.push_back(ordering[start_pos + i]);
    }
    
    // Create new ordering by reconstructing it properly
    std::vector<int> new_ordering;
    new_ordering.reserve(hg.num_vertices);
    
    if (target_pos < start_pos) {
        // Moving left: [before_target][block][between_target_and_start][after_start]
        
        // Add elements before target position
        for (int i = 0; i < target_pos; ++i) {
            new_ordering.push_back(ordering[i]);
        }
        
        // Add the moved block
        for (int v : block) {
            new_ordering.push_back(v);
        }
        
        // Add elements between target and original start (these shift right)
        for (int i = target_pos; i < start_pos; ++i) {
            new_ordering.push_back(ordering[i]);
        }
        
        // Add elements after original block
        for (int i = start_pos + block_size; i < hg.num_vertices; ++i) {
            new_ordering.push_back(ordering[i]);
        }
        
    } else {
        // Moving right: [before_start][between_start_and_target][block][after_target]
        
        // Add elements before original block
        for (int i = 0; i < start_pos; ++i) {
            new_ordering.push_back(ordering[i]);
        }
        
        // Add elements between original block and target (these shift left)
        for (int i = start_pos + block_size; i < target_pos; ++i) {
            new_ordering.push_back(ordering[i]);
        }
        
        // Add the moved block
        for (int v : block) {
            new_ordering.push_back(v);
        }
        
        // Add remaining elements after target
        for (int i = target_pos; i < hg.num_vertices; ++i) {
            new_ordering.push_back(ordering[i]);
        }
    }
    
    // Verify the new ordering has the correct size
    if (static_cast<int>(new_ordering.size()) != hg.num_vertices) {
        std::cerr << "Error: Block move created ordering of wrong size: " 
                  << new_ordering.size() << " vs " << hg.num_vertices << std::endl;
        return;
    }
    
    // Update ordering and position arrays
    ordering = new_ordering;
    for (int i = 0; i < hg.num_vertices; ++i) {
        position[ordering[i]] = i;
    }
}

void HypergraphCutwidthTracker::performDistantSwap(int pos1, int pos2) {
    if (pos1 < 0 || pos1 >= hg.num_vertices || pos2 < 0 || pos2 >= hg.num_vertices || pos1 == pos2) {
        return; // Invalid swap
    }
    
    // Swap the vertices in the ordering
    std::swap(ordering[pos1], ordering[pos2]);
    
    // Update position mapping
    position[ordering[pos1]] = pos1;
    position[ordering[pos2]] = pos2;
    
    // Recompute cutwidth information for affected hyperedges
    std::unordered_set<int> affected_hyperedges;
    
    for (int he : hg.vertex_to_hyperedges[ordering[pos1]]) {
        affected_hyperedges.insert(he);
    }
    for (int he : hg.vertex_to_hyperedges[ordering[pos2]]) {
        affected_hyperedges.insert(he);
    }
    
    // Reset cut sizes and recompute
    std::fill(cut_sizes.begin(), cut_sizes.end(), 0);
    
    // Recompute all hyperedge info and contributions
    for (int he = 0; he < hg.num_hyperedges; ++he) {
        updateHyperedgeInfo(he);
        
        const auto& info = hyperedge_info[he];
        if (info.is_cutting()) {
            int start_pos = info.first_cut_pos();
            int end_pos = info.last_cut_pos();
            
            for (int k = start_pos; k <= end_pos && k < static_cast<int>(cut_sizes.size()); ++k) {
                cut_sizes[k - 1]++;
            }
        }
    }
    
    updatePeakInfo();
}

// Main algorithm implementation

PeakCutwidthResult computePeakCutwidthOrdering(const Hypergraph& hg, 
                                              const PeakCutwidthParams& params) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    PeakCutwidthResult result;
    
    if (params.verbose) {
        std::cout << "Computing peak cutwidth minimization ordering..." << std::endl;
    }
    
    // Step 1: Get initial ordering
    std::vector<int> ordering;
    if (params.use_random_start) {
        ordering = computeRandomOrdering(hg);
        if (params.verbose) std::cout << "Starting from random ordering" << std::endl;
    } else {
        ordering = computeFiedlerOrdering(hg);
        if (params.verbose) std::cout << "Starting from Fiedler ordering" << std::endl;
    }
    
    // Initialize cutwidth tracker
    HypergraphCutwidthTracker tracker(hg);
    tracker.initialize(ordering);
    
    int initial_peak = tracker.getPeakCutwidth();
    result.initial_peak = initial_peak;
    
    if (params.verbose) {
        std::cout << "Initial peak cutwidth: " << initial_peak 
                  << " at position " << tracker.getPeakPosition() << std::endl;
    }
    
    auto refinement_start = std::chrono::high_resolution_clock::now();
    
    // Apply greedy refinement
    bool improved = true;
    int iteration = 0;
    int total_swaps = 0;
    
    while (improved && iteration < params.max_iterations) {
        improved = false;
        int iteration_swaps = 0;
        
        if (params.verbose) {
            std::cout << "\n--- Iteration " << (iteration + 1) << " ---" << std::endl;
        }
        
        // Phase 1: Hyperedge-aware priority moves
        if (params.use_hyperedge_aware) {
            auto priority_candidates = tracker.getPrioritySwapCandidates();
            
            if (params.verbose && !priority_candidates.empty()) {
                std::cout << "Evaluating " << priority_candidates.size() << " priority swap candidates..." << std::endl;
            }
            
            for (const auto& [pos1, pos2] : priority_candidates) {
                int delta;
                if (abs(pos1 - pos2) == 1) {
                    // Adjacent swap
                    delta = tracker.evaluateSwap(std::min(pos1, pos2));
                } else {
                    // Distant swap
                    delta = tracker.evaluateDistantSwap(pos1, pos2);
                }
                
                if (delta < 0) {
                    if (abs(pos1 - pos2) == 1) {
                        tracker.performSwap(std::min(pos1, pos2));
                    } else {
                        tracker.performDistantSwap(pos1, pos2);
                    }
                    
                    improved = true;
                    iteration_swaps++;
                    total_swaps++;
                    
                    if (params.verbose && iteration_swaps <= 5) {
                        std::cout << "  Priority swap (" << pos1 << "," << pos2 
                                  << ") improved peak by " << -delta 
                                  << " (new peak: " << tracker.getPeakCutwidth() << ")" << std::endl;
                    }
                }
            }
        }
        
        // Phase 2: Block moves (if enabled)
        if (params.max_block_size > 1) {
            if (params.verbose) {
                std::cout << "Evaluating block moves (max size: " << params.max_block_size << ")..." << std::endl;
            }
            
            // Track moves to prevent immediate reversals
            std::set<std::tuple<int, int, int>> recent_moves; // (start_pos, block_size, target_pos)
            
            for (int block_size = 2; block_size <= params.max_block_size; ++block_size) {
                for (int start_pos = 0; start_pos <= static_cast<int>(tracker.getOrdering().size()) - block_size; start_pos += block_size) {
                    // Try moving the block to different positions
                    for (int target_pos = 0; target_pos <= static_cast<int>(tracker.getOrdering().size()) - block_size; target_pos += block_size) {
                        if (abs(target_pos - start_pos) < block_size) continue; // Skip overlapping moves
                        
                        // Check if this move was recently performed (prevent immediate reversal)
                        auto move_tuple = std::make_tuple(start_pos, block_size, target_pos);
                        auto reverse_tuple = std::make_tuple(target_pos, block_size, start_pos);
                        if (recent_moves.count(reverse_tuple) > 0) continue;
                        
                        int delta = tracker.evaluateBlockMove(start_pos, block_size, target_pos);
                        if (delta < 0) {
                            tracker.performBlockMove(start_pos, block_size, target_pos);
                            recent_moves.insert(move_tuple);
                            improved = true;
                            iteration_swaps++;
                            total_swaps++;
                            
                            if (params.verbose && iteration_swaps <= 3) {
                                std::cout << "  Block move (pos " << start_pos 
                                          << ", size " << block_size 
                                          << " -> pos " << target_pos 
                                          << ") improved peak by " << -delta 
                                          << " (new peak: " << tracker.getPeakCutwidth() << ")" << std::endl;
                            }
                            break; // Only one block move per iteration
                        }
                    }
                    if (improved) break;
                }
                if (improved) break;
            }
        }
        
        // Phase 3: Extended distance swaps
        if (params.max_swap_distance > 1 && !improved) {
            if (params.verbose) {
                std::cout << "Evaluating distant swaps (max distance: " << params.max_swap_distance << ")..." << std::endl;
            }
            
            for (int distance = 2; distance <= params.max_swap_distance && !improved; ++distance) {
                for (int pos1 = 0; pos1 < static_cast<int>(tracker.getOrdering().size()) - distance && !improved; ++pos1) {
                    int pos2 = pos1 + distance;
                    
                    int delta = tracker.evaluateDistantSwap(pos1, pos2);
                    if (delta < 0) {
                        tracker.performDistantSwap(pos1, pos2);
                        
                        improved = true;
                        iteration_swaps++;
                        total_swaps++;
                        
                        if (params.verbose) {
                            std::cout << "  Distant swap (" << pos1 << "," << pos2 
                                      << ", distance " << distance 
                                      << ") improved peak by " << -delta 
                                      << " (new peak: " << tracker.getPeakCutwidth() << ")" << std::endl;
                        }
                    }
                }
            }
        }
        
        // Phase 4: Traditional adjacent swaps (fallback)
        if (!improved) {
            for (int k = 0; k < static_cast<int>(tracker.getOrdering().size()) - 1; ++k) {
                int delta = tracker.evaluateSwap(k);
                
                if (delta < 0) {
                    tracker.performSwap(k);
                    improved = true;
                    iteration_swaps++;
                    total_swaps++;
                    
                    if (params.verbose && iteration_swaps <= 3) {
                        std::cout << "  Adjacent swap at position " << k 
                                  << " improved peak by " << -delta 
                                  << " (new peak: " << tracker.getPeakCutwidth() << ")" << std::endl;
                    }
                }
            }
        }
        
        iteration++;
        if (params.verbose && iteration_swaps > 0) {
            std::cout << "Iteration " << iteration 
                      << ": " << iteration_swaps << " moves, peak = " 
                      << tracker.getPeakCutwidth() << std::endl;
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    
    // Prepare result
    result.ordering = tracker.getOrdering();
    result.cutwidth_curve = tracker.getCutwidthCurve();
    result.peak_cutwidth = tracker.getPeakCutwidth();
    result.peak_position = tracker.getPeakPosition();
    result.final_peak = tracker.getPeakCutwidth();
    result.iterations_performed = iteration;
    result.swaps_performed = total_swaps;
    result.converged = !improved;
    
    result.computation_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    result.refinement_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - refinement_start).count();
    
    if (params.verbose) {
        std::cout << "\nPeak cutwidth optimization complete:" << std::endl;
        std::cout << "  Initial peak: " << result.initial_peak << std::endl;
        std::cout << "  Final peak: " << result.final_peak << std::endl;
        std::cout << "  Improvement: " << (result.initial_peak - result.final_peak) << std::endl;
        std::cout << "  Iterations: " << result.iterations_performed << std::endl;
        std::cout << "  Total swaps: " << result.swaps_performed << std::endl;
        std::cout << "  Total time: " << result.computation_time_ms << " ms" << std::endl;
        std::cout << "  Refinement time: " << result.refinement_time_ms << " ms" << std::endl;
        std::cout << "  Converged: " << (result.converged ? "Yes" : "No") << std::endl;
    }
    
    return result;
}

// Enhanced refinement function that takes an initial ordering
PeakCutwidthResult computeEnhancedRefinement(const Hypergraph& hg, const std::vector<int>& initial_ordering, const PeakCutwidthParams& params) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    if (params.verbose) {
        std::cout << "=== Enhanced Peak Cutwidth Refinement ===" << std::endl;
        std::cout << "Max swap distance: " << params.max_swap_distance << std::endl;
        std::cout << "Max block size: " << params.max_block_size << std::endl;
        std::cout << "Hyperedge-aware: " << (params.use_hyperedge_aware ? "Yes" : "No") << std::endl;
        std::cout << "Max iterations: " << params.max_iterations << std::endl;
    }
    
    // Initialize tracker with the provided initial ordering
    HypergraphCutwidthTracker tracker(hg);
    tracker.initialize(initial_ordering);
    
    int initial_peak = tracker.getPeakCutwidth();
    int initial_peak_pos = tracker.getPeakPosition();
    
    if (params.verbose) {
        std::cout << "Initial peak cutwidth: " << initial_peak << " at position " << initial_peak_pos << std::endl;
    }
    
    auto refinement_start = std::chrono::high_resolution_clock::now();
    
    // Apply enhanced greedy refinement
    bool improved = true;
    int iteration = 0;
    int total_swaps = 0;
    
    while (improved && iteration < params.max_iterations) {
        improved = false;
        int iteration_swaps = 0;
        
        if (params.verbose) {
            std::cout << "\n--- Iteration " << (iteration + 1) << " ---" << std::endl;
        }
        
        // Phase 1: Hyperedge-aware priority moves
        if (params.use_hyperedge_aware) {
            auto priority_candidates = tracker.getPrioritySwapCandidates();
            
            if (params.verbose && !priority_candidates.empty()) {
                std::cout << "Evaluating " << priority_candidates.size() << " priority swap candidates..." << std::endl;
            }
            
            for (const auto& [pos1, pos2] : priority_candidates) {
                int delta;
                if (abs(pos1 - pos2) == 1) {
                    // Adjacent swap
                    delta = tracker.evaluateSwap(std::min(pos1, pos2));
                } else {
                    // Distant swap
                    delta = tracker.evaluateDistantSwap(pos1, pos2);
                }
                
                if (delta < 0) {
                    if (abs(pos1 - pos2) == 1) {
                        tracker.performSwap(std::min(pos1, pos2));
                    } else {
                        tracker.performDistantSwap(pos1, pos2);
                    }
                    
                    improved = true;
                    iteration_swaps++;
                    total_swaps++;
                    
                    if (params.verbose && iteration_swaps <= 5) {
                        std::cout << "  Priority swap (" << pos1 << "," << pos2 
                                  << ") improved peak by " << -delta 
                                  << " (new peak: " << tracker.getPeakCutwidth() << ")" << std::endl;
                    }
                }
            }
        }
        
        // Phase 2: Block moves (if enabled)
        if (params.max_block_size > 1) {
            if (params.verbose) {
                std::cout << "Evaluating block moves (max size: " << params.max_block_size << ")..." << std::endl;
            }
            
            // Track moves to prevent immediate reversals
            std::set<std::tuple<int, int, int>> recent_moves; // (start_pos, block_size, target_pos)
            
            for (int block_size = 2; block_size <= params.max_block_size; ++block_size) {
                for (int start_pos = 0; start_pos <= static_cast<int>(tracker.getOrdering().size()) - block_size; start_pos += block_size) {
                    // Try moving the block to different positions
                    for (int target_pos = 0; target_pos <= static_cast<int>(tracker.getOrdering().size()) - block_size; target_pos += block_size) {
                        if (abs(target_pos - start_pos) < block_size) continue; // Skip overlapping moves
                        
                        // Check if this move was recently performed (prevent immediate reversal)
                        auto move_tuple = std::make_tuple(start_pos, block_size, target_pos);
                        auto reverse_tuple = std::make_tuple(target_pos, block_size, start_pos);
                        if (recent_moves.count(reverse_tuple) > 0) continue;
                        
                        int delta = tracker.evaluateBlockMove(start_pos, block_size, target_pos);
                        if (delta < 0) {
                            tracker.performBlockMove(start_pos, block_size, target_pos);
                            recent_moves.insert(move_tuple);
                            improved = true;
                            iteration_swaps++;
                            total_swaps++;
                            
                            if (params.verbose && iteration_swaps <= 3) {
                                std::cout << "  Block move (pos " << start_pos 
                                          << ", size " << block_size 
                                          << " -> pos " << target_pos 
                                          << ") improved peak by " << -delta 
                                          << " (new peak: " << tracker.getPeakCutwidth() << ")" << std::endl;
                            }
                            break; // Only one block move per iteration
                        }
                    }
                    if (improved) break;
                }
                if (improved) break;
            }
        }
        
        // Phase 3: Extended distance swaps
        if (params.max_swap_distance > 1 && !improved) {
            if (params.verbose) {
                std::cout << "Evaluating distant swaps (max distance: " << params.max_swap_distance << ")..." << std::endl;
            }
            
            for (int distance = 2; distance <= params.max_swap_distance && !improved; ++distance) {
                for (int pos1 = 0; pos1 < static_cast<int>(tracker.getOrdering().size()) - distance && !improved; ++pos1) {
                    int pos2 = pos1 + distance;
                    
                    int delta = tracker.evaluateDistantSwap(pos1, pos2);
                    if (delta < 0) {
                        tracker.performDistantSwap(pos1, pos2);
                        
                        improved = true;
                        iteration_swaps++;
                        total_swaps++;
                        
                        if (params.verbose) {
                            std::cout << "  Distant swap (" << pos1 << "," << pos2 
                                      << ", distance " << distance 
                                      << ") improved peak by " << -delta 
                                      << " (new peak: " << tracker.getPeakCutwidth() << ")" << std::endl;
                        }
                    }
                }
            }
        }
        
        // Phase 4: Traditional adjacent swaps (fallback)
        if (!improved) {
            for (int k = 0; k < static_cast<int>(tracker.getOrdering().size()) - 1; ++k) {
                int delta = tracker.evaluateSwap(k);
                
                if (delta < 0) {
                    tracker.performSwap(k);
                    improved = true;
                    iteration_swaps++;
                    total_swaps++;
                    
                    if (params.verbose && iteration_swaps <= 3) {
                        std::cout << "  Adjacent swap at position " << k 
                                  << " improved peak by " << -delta 
                                  << " (new peak: " << tracker.getPeakCutwidth() << ")" << std::endl;
                    }
                }
            }
        }
        
        iteration++;
        if (params.verbose && iteration_swaps > 0) {
            std::cout << "Iteration " << iteration 
                      << ": " << iteration_swaps << " moves, peak = " 
                      << tracker.getPeakCutwidth() << std::endl;
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    int final_peak = tracker.getPeakCutwidth();
    
    if (params.verbose) {
        std::cout << "\n=== Enhanced Refinement Complete ===" << std::endl;
        std::cout << "Initial peak cutwidth: " << initial_peak << std::endl;
        std::cout << "Final peak cutwidth: " << final_peak << std::endl;
        std::cout << "Improvement: " << (initial_peak - final_peak);
        if (initial_peak > 0) {
            std::cout << " (" << std::fixed << std::setprecision(1) 
                      << (100.0 * (initial_peak - final_peak) / initial_peak) << "%)";
        }
        std::cout << std::endl;
        std::cout << "Total moves performed: " << total_swaps << std::endl;
        std::cout << "Iterations: " << iteration << std::endl;
        std::cout << "Total time: " << total_duration.count() << " ms" << std::endl;
    }
    
    // Prepare result
    PeakCutwidthResult result;
    result.ordering = tracker.getOrdering();
    result.initial_peak = initial_peak;
    result.final_peak = final_peak;
    result.swaps_performed = total_swaps;
    result.iterations_performed = iteration;
    result.converged = !improved;
    result.computation_time_ms = total_duration.count();
    
    return result;
}

// Simulated Annealing Implementation with Large Step Markov Chains

IncrementalHyperedgeTracker::IncrementalHyperedgeTracker(const Hypergraph& hypergraph) 
    : HypergraphCutwidthTracker(hypergraph), cached_peak_position(-1), cache_valid(false) {
    // Initialize move probabilities for LSMC
    move_probabilities = {0.4, 0.2, 0.2, 0.15, 0.05}; // SMALL_STEP, BLOCK_MOVE, HYPEREDGE_CLUSTER, SEGMENT_REVERSAL, RANDOM_SHUFFLE
}

void IncrementalHyperedgeTracker::updateAfterSwap(int pos1, int pos2) {
    // Invalidate cache if peak position might have changed
    invalidateCache();
}

void IncrementalHyperedgeTracker::updateAfterBlockMove(int start_pos, int block_size, int target_pos) {
    // Invalidate cache after large moves
    invalidateCache();
}

void IncrementalHyperedgeTracker::updateAfterSegmentReversal(int start_pos, int end_pos) {
    // Invalidate cache after segment reversals
    invalidateCache();
}

MoveType IncrementalHyperedgeTracker::selectMoveType(double temperature) {
    // Adjust probabilities based on temperature
    std::vector<double> temp_probabilities = move_probabilities;
    
    if (temperature > 10.0) {
        // Hot phase: favor large steps
        temp_probabilities = {0.2, 0.3, 0.3, 0.15, 0.05};
    } else if (temperature > 1.0) {
        // Medium phase: balanced
        temp_probabilities = {0.4, 0.2, 0.2, 0.15, 0.05};
    } else {
        // Cold phase: favor small steps
        temp_probabilities = {0.7, 0.1, 0.1, 0.08, 0.02};
    }
    
    // Sample from distribution
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    
    double r = dis(gen);
    double cumulative = 0.0;
    
    for (size_t i = 0; i < temp_probabilities.size(); ++i) {
        cumulative += temp_probabilities[i];
        if (r <= cumulative) {
            return static_cast<MoveType>(i);
        }
    }
    
    return MoveType::SMALL_STEP; // Fallback
}

std::pair<int, int> IncrementalHyperedgeTracker::generateBlockMove(double temperature) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    
    // Temperature-dependent block size
    int max_size = static_cast<int>(2 + temperature * 0.5);
    max_size = std::min(max_size, 20);
    max_size = std::min(max_size, hg.num_vertices / 4);
    
    int block_size = 2 + (gen() % std::max(1, max_size - 1));
    int start_pos = gen() % std::max(1, hg.num_vertices - block_size);
    
    return {start_pos, block_size};
}

std::pair<int, int> IncrementalHyperedgeTracker::generateSegmentReversal(double temperature) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    
    // Temperature-dependent segment length
    int max_length = static_cast<int>(5 + temperature * 1.5);
    max_length = std::min(max_length, 50);
    max_length = std::min(max_length, hg.num_vertices / 3);
    
    int segment_length = 5 + (gen() % std::max(1, max_length - 4));
    int start_pos = gen() % std::max(1, hg.num_vertices - segment_length);
    
    return {start_pos, start_pos + segment_length - 1};
}

std::vector<int> IncrementalHyperedgeTracker::generateHyperedgeClusterMove() {
    auto peak_hyperedges = getHyperedgesAtPeak();
    if (peak_hyperedges.empty()) {
        return {};
    }
    
    static std::random_device rd;
    static std::mt19937 gen(rd());
    
    // Select a random peak-contributing hyperedge
    int selected_he = peak_hyperedges[gen() % peak_hyperedges.size()];
    
    // Return vertices in this hyperedge
    return hg.hyperedges[selected_he];
}

std::vector<std::pair<int, int>> IncrementalHyperedgeTracker::getCachedPrioritySwapCandidates() {
    if (!cache_valid) {
        refreshCache();
    }
    return getPrioritySwapCandidates();
}

void IncrementalHyperedgeTracker::invalidateCache() {
    cache_valid = false;
}

void IncrementalHyperedgeTracker::refreshCache() {
    cached_peak_hyperedges = getHyperedgesAtPeak();
    cached_peak_position = getPeakPosition();
    cache_valid = true;
}

// Large move evaluation and execution functions
int evaluateBlockMoveForAnnealing(IncrementalHyperedgeTracker& tracker, int start_pos, int block_size) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    
    // Generate random target position
    int max_target = tracker.getOrdering().size() - block_size;
    if (max_target <= 0) return INT_MAX; // Invalid move
    
    int target_pos = gen() % max_target;
    
    // Avoid overlapping moves
    if (abs(target_pos - start_pos) < block_size) {
        return INT_MAX;
    }
    
    return tracker.evaluateBlockMove(start_pos, block_size, target_pos);
}

int evaluateSegmentReversal(IncrementalHyperedgeTracker& tracker, int start_pos, int end_pos) {
    if (start_pos >= end_pos || start_pos < 0 || end_pos >= tracker.getOrdering().size()) {
        return INT_MAX; // Invalid move
    }
    
    // Store current state
    auto current_ordering = tracker.getOrdering();
    int current_peak = tracker.getPeakCutwidth();
    
    // Create reversed segment
    std::vector<int> new_ordering = current_ordering;
    std::reverse(new_ordering.begin() + start_pos, new_ordering.begin() + end_pos + 1);
    
    // Temporarily apply the move to evaluate
    tracker.initialize(new_ordering);
    int new_peak = tracker.getPeakCutwidth();
    
    // Restore original state
    tracker.initialize(current_ordering);
    
    return new_peak - current_peak;
}

void performSegmentReversal(IncrementalHyperedgeTracker& tracker, int start_pos, int end_pos) {
    if (start_pos >= end_pos || start_pos < 0 || end_pos >= tracker.getOrdering().size()) {
        return; // Invalid move
    }
    
    auto current_ordering = tracker.getOrdering();
    std::reverse(current_ordering.begin() + start_pos, current_ordering.begin() + end_pos + 1);
    tracker.initialize(current_ordering);
    tracker.updateAfterSegmentReversal(start_pos, end_pos);
}

int evaluateHyperedgeClusterMove(IncrementalHyperedgeTracker& tracker, const std::vector<int>& vertices) {
    if (vertices.empty()) return INT_MAX;
    
    static std::random_device rd;
    static std::mt19937 gen(rd());
    
    // Store current state
    auto current_ordering = tracker.getOrdering();
    int current_peak = tracker.getPeakCutwidth();
    
    // Generate target position for clustering
    int target_start = gen() % std::max(1, static_cast<int>(tracker.getOrdering().size() - vertices.size() + 1));
    
    // Create new ordering with vertices clustered at target position
    std::vector<int> new_ordering;
    std::unordered_set<int> cluster_vertices(vertices.begin(), vertices.end());
    
    // Add vertices before cluster position (excluding cluster vertices)
    for (int i = 0; i < target_start; ++i) {
        if (cluster_vertices.find(current_ordering[i]) == cluster_vertices.end()) {
            new_ordering.push_back(current_ordering[i]);
        }
    }
    
    // Add cluster vertices
    for (int v : vertices) {
        new_ordering.push_back(v);
    }
    
    // Add remaining vertices (excluding cluster vertices)
    for (int i = target_start; i < current_ordering.size(); ++i) {
        if (cluster_vertices.find(current_ordering[i]) == cluster_vertices.end()) {
            new_ordering.push_back(current_ordering[i]);
        }
    }
    
    // Pad if necessary (shouldn't happen with correct logic)
    while (new_ordering.size() < current_ordering.size()) {
        for (int v : current_ordering) {
            if (std::find(new_ordering.begin(), new_ordering.end(), v) == new_ordering.end()) {
                new_ordering.push_back(v);
                break;
            }
        }
    }
    
    if (new_ordering.size() != current_ordering.size()) {
        return INT_MAX; // Invalid move
    }
    
    // Temporarily apply the move to evaluate
    tracker.initialize(new_ordering);
    int new_peak = tracker.getPeakCutwidth();
    
    // Restore original state
    tracker.initialize(current_ordering);
    
    return new_peak - current_peak;
}

void performHyperedgeClusterMove(IncrementalHyperedgeTracker& tracker, const std::vector<int>& vertices) {
    if (vertices.empty()) return;
    
    static std::random_device rd;
    static std::mt19937 gen(rd());
    
    auto current_ordering = tracker.getOrdering();
    
    // Generate target position for clustering
    int target_start = gen() % std::max(1, static_cast<int>(tracker.getOrdering().size() - vertices.size() + 1));
    
    // Create new ordering with vertices clustered at target position
    std::vector<int> new_ordering;
    std::unordered_set<int> cluster_vertices(vertices.begin(), vertices.end());
    
    // Add vertices before cluster position (excluding cluster vertices)
    for (int i = 0; i < target_start; ++i) {
        if (cluster_vertices.find(current_ordering[i]) == cluster_vertices.end()) {
            new_ordering.push_back(current_ordering[i]);
        }
    }
    
    // Add cluster vertices
    for (int v : vertices) {
        new_ordering.push_back(v);
    }
    
    // Add remaining vertices (excluding cluster vertices)
    for (int i = target_start; i < current_ordering.size(); ++i) {
        if (cluster_vertices.find(current_ordering[i]) == cluster_vertices.end()) {
            new_ordering.push_back(current_ordering[i]);
        }
    }
    
    // Ensure we have all vertices
    while (new_ordering.size() < current_ordering.size()) {
        for (int v : current_ordering) {
            if (std::find(new_ordering.begin(), new_ordering.end(), v) == new_ordering.end()) {
                new_ordering.push_back(v);
                break;
            }
        }
    }
    
    if (new_ordering.size() == current_ordering.size()) {
        tracker.initialize(new_ordering);
    }
}

// Main simulated annealing algorithm
AnnealingResult computeSimulatedAnnealing(
    const Hypergraph& hg,
    const std::vector<int>& initial_ordering,
    const AnnealingParams& params
) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    AnnealingResult result;
    
    if (params.verbose) {
        std::cout << "=== Simulated Annealing with LSMC ===" << std::endl;
        std::cout << "Initial temperature: " << params.initial_temperature << std::endl;
        std::cout << "Cooling rate: " << params.cooling_rate << std::endl;
        std::cout << "Max iterations: " << params.max_annealing_iterations << std::endl;
        std::cout << "Large steps enabled: " << (params.use_large_steps ? "Yes" : "No") << std::endl;
    }
    
    // Initialize tracker
    IncrementalHyperedgeTracker tracker(hg);
    tracker.initialize(initial_ordering);
    
    double temperature = params.initial_temperature;
    int initial_peak = tracker.getPeakCutwidth();
    int best_peak = initial_peak;
    std::vector<int> best_ordering = initial_ordering;
    
    // Statistics tracking
    int total_evaluations = 0;
    int accepted_moves = 0;
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    
    if (params.verbose) {
        std::cout << "Initial peak cutwidth: " << initial_peak << std::endl;
    }
    
    for (int iter = 0; iter < params.max_annealing_iterations; ++iter) {
        int iteration_accepted = 0;
        int iteration_evaluated = 0;
        int iteration_large_steps = 0;
        int iteration_large_accepted = 0;
        
        // Determine number of candidates for this iteration
        int num_candidates = std::min(params.candidates_per_iteration, 
                                     static_cast<int>(tracker.getOrdering().size() * 2));
        
        for (int candidate = 0; candidate < num_candidates; ++candidate) {
            MoveType move_type = MoveType::SMALL_STEP;
            
            // Select move type
            if (params.use_large_steps && dis(gen) < params.large_step_probability) {
                move_type = tracker.selectMoveType(temperature);
            }
            
            int delta = INT_MAX;
            bool is_large_step = (move_type != MoveType::SMALL_STEP);
            
            if (is_large_step) {
                iteration_large_steps++;
                result.large_steps_attempted++;
            }
            
            switch (move_type) {
                case MoveType::SMALL_STEP: {
                    // Traditional hyperedge-aware swaps
                    if (params.use_hyperedge_aware) {
                        auto candidates = tracker.getCachedPrioritySwapCandidates();
                        if (!candidates.empty()) {
                            auto [pos1, pos2] = candidates[gen() % candidates.size()];
                            if (abs(pos1 - pos2) == 1) {
                                delta = tracker.evaluateSwap(std::min(pos1, pos2));
                            } else {
                                delta = tracker.evaluateDistantSwap(pos1, pos2);
                            }
                            
                            if (delta < INT_MAX) {
                                iteration_evaluated++;
                                total_evaluations++;
                                
                                bool accept = false;
                                if (delta < 0) {
                                    accept = true;
                                    result.beneficial_swaps++;
                                } else if (delta == 0) {
                                    accept = (dis(gen) < 0.5);
                                    if (accept) result.neutral_swaps++;
                                } else {
                                    double p_accept = std::exp(-static_cast<double>(delta) / temperature);
                                    accept = (dis(gen) < p_accept);
                                    if (accept) result.harmful_swaps_accepted++;
                                }
                                
                                if (accept) {
                                    if (abs(pos1 - pos2) == 1) {
                                        tracker.performSwap(std::min(pos1, pos2));
                                    } else {
                                        tracker.performDistantSwap(pos1, pos2);
                                    }
                                    tracker.updateAfterSwap(pos1, pos2);
                                    iteration_accepted++;
                                    accepted_moves++;
                                    
                                    int current_peak = tracker.getPeakCutwidth();
                                    if (current_peak < best_peak) {
                                        best_peak = current_peak;
                                        best_ordering = tracker.getOrdering();
                                    }
                                }
                            }
                        }
                    }
                    break;
                }
                
                case MoveType::BLOCK_MOVE: {
                    auto [start_pos, block_size] = tracker.generateBlockMove(temperature);
                    delta = evaluateBlockMoveForAnnealing(tracker, start_pos, block_size);
                    
                    if (delta < INT_MAX) {
                        iteration_evaluated++;
                        total_evaluations++;
                        
                        bool accept = false;
                        if (delta < 0) {
                            accept = true;
                            result.beneficial_swaps++;
                        } else if (delta == 0) {
                            accept = (dis(gen) < 0.3);
                            if (accept) result.neutral_swaps++;
                        } else {
                            double p_accept = std::exp(-static_cast<double>(delta) / temperature);
                            accept = (dis(gen) < p_accept);
                            if (accept) result.harmful_swaps_accepted++;
                        }
                        
                        if (accept) {
                            // Find valid target position
                            int max_target = tracker.getOrdering().size() - block_size;
                            int target_pos = gen() % max_target;
                            while (abs(target_pos - start_pos) < block_size) {
                                target_pos = gen() % max_target;
                            }
                            
                            tracker.performBlockMove(start_pos, block_size, target_pos);
                            tracker.updateAfterBlockMove(start_pos, block_size, target_pos);
                            iteration_accepted++;
                            iteration_large_accepted++;
                            accepted_moves++;
                            result.large_steps_accepted++;
                            
                            int current_peak = tracker.getPeakCutwidth();
                            if (current_peak < best_peak) {
                                best_peak = current_peak;
                                best_ordering = tracker.getOrdering();
                            }
                        }
                    }
                    break;
                }
                
                case MoveType::SEGMENT_REVERSAL: {
                    auto [start_pos, end_pos] = tracker.generateSegmentReversal(temperature);
                    delta = evaluateSegmentReversal(tracker, start_pos, end_pos);
                    
                    if (delta < INT_MAX) {
                        iteration_evaluated++;
                        total_evaluations++;
                        
                        bool accept = false;
                        if (delta < 0) {
                            accept = true;
                            result.beneficial_swaps++;
                        } else if (delta == 0) {
                            accept = (dis(gen) < 0.3);
                            if (accept) result.neutral_swaps++;
                        } else {
                            double p_accept = std::exp(-static_cast<double>(delta) / temperature);
                            accept = (dis(gen) < p_accept);
                            if (accept) result.harmful_swaps_accepted++;
                        }
                        
                        if (accept) {
                            performSegmentReversal(tracker, start_pos, end_pos);
                            iteration_accepted++;
                            iteration_large_accepted++;
                            accepted_moves++;
                            result.large_steps_accepted++;
                            
                            int current_peak = tracker.getPeakCutwidth();
                            if (current_peak < best_peak) {
                                best_peak = current_peak;
                                best_ordering = tracker.getOrdering();
                            }
                        }
                    }
                    break;
                }
                
                case MoveType::HYPEREDGE_CLUSTER: {
                    auto cluster_vertices = tracker.generateHyperedgeClusterMove();
                    if (!cluster_vertices.empty()) {
                        delta = evaluateHyperedgeClusterMove(tracker, cluster_vertices);
                        
                        if (delta < INT_MAX) {
                            iteration_evaluated++;
                            total_evaluations++;
                            
                            bool accept = false;
                            if (delta < 0) {
                                accept = true;
                                result.beneficial_swaps++;
                            } else if (delta == 0) {
                                accept = (dis(gen) < 0.3);
                                if (accept) result.neutral_swaps++;
                            } else {
                                double p_accept = std::exp(-static_cast<double>(delta) / temperature);
                                accept = (dis(gen) < p_accept);
                                if (accept) result.harmful_swaps_accepted++;
                            }
                            
                            if (accept) {
                                performHyperedgeClusterMove(tracker, cluster_vertices);
                                iteration_accepted++;
                                iteration_large_accepted++;
                                accepted_moves++;
                                result.large_steps_accepted++;
                                
                                int current_peak = tracker.getPeakCutwidth();
                                if (current_peak < best_peak) {
                                    best_peak = current_peak;
                                    best_ordering = tracker.getOrdering();
                                }
                            }
                        }
                    }
                    break;
                }
                
                case MoveType::RANDOM_SHUFFLE: {
                    // Random shuffle of small subset
                    int shuffle_size = 3 + (gen() % 8); // 3-10 vertices
                    if (shuffle_size >= static_cast<int>(tracker.getOrdering().size())) {
                        shuffle_size = tracker.getOrdering().size() - 1;
                    }
                    if (shuffle_size < 2) break; // Skip if too small
                    
                    int start_pos = gen() % std::max(1, static_cast<int>(tracker.getOrdering().size() - shuffle_size));
                    
                    // Store current state before modification
                    auto original_ordering = tracker.getOrdering();
                    int current_peak = tracker.getPeakCutwidth();
                    
                    // Create shuffled version
                    auto shuffled_ordering = original_ordering;
                    std::shuffle(shuffled_ordering.begin() + start_pos, 
                               shuffled_ordering.begin() + start_pos + shuffle_size, gen);
                    
                    // Temporarily apply to evaluate
                    tracker.initialize(shuffled_ordering);
                    int new_peak = tracker.getPeakCutwidth();
                    delta = new_peak - current_peak;
                    
                    iteration_evaluated++;
                    total_evaluations++;
                    
                    bool accept = false;
                    if (delta < 0) {
                        accept = true;
                        result.beneficial_swaps++;
                    } else if (delta == 0) {
                        accept = (dis(gen) < 0.2);
                        if (accept) result.neutral_swaps++;
                    } else {
                        double p_accept = std::exp(-static_cast<double>(delta) / temperature);
                        accept = (dis(gen) < p_accept);
                        if (accept) result.harmful_swaps_accepted++;
                    }
                    
                    if (accept) {
                        // Keep the shuffled state
                        iteration_accepted++;
                        iteration_large_accepted++;
                        accepted_moves++;
                        result.large_steps_accepted++;
                        
                        if (new_peak < best_peak) {
                            best_peak = new_peak;
                            best_ordering = tracker.getOrdering();
                        }
                    } else {
                        // Restore original state
                        tracker.initialize(original_ordering);
                    }
                    break;
                }
            }
        }
        
        // Record iteration statistics
        result.peak_history.push_back(tracker.getPeakCutwidth());
        result.temperature_history.push_back(temperature);
        
        // Adaptive temperature adjustment
        double acceptance_rate = (iteration_evaluated > 0) ? 
            static_cast<double>(iteration_accepted) / iteration_evaluated : 0.0;
        
        if (params.adaptive_temperature && acceptance_rate < 0.1 && temperature > 1.0) {
            temperature *= 1.05; // Slow down cooling if too few acceptances
        }
        
        // Cool down
        temperature *= params.cooling_rate;
        
        // Early termination if acceptance rate too low
        if (acceptance_rate < params.min_acceptance_rate && iter > 5) {
            if (params.verbose) {
                std::cout << "Early termination at iteration " << iter 
                          << ": acceptance rate " << std::fixed << std::setprecision(1) 
                          << (100.0 * acceptance_rate) << "%" << std::endl;
            }
            result.converged = true;
            break;
        }
        
        if (params.verbose) {
            std::cout << "Iteration " << (iter + 1) << ": peak=" << tracker.getPeakCutwidth()
                      << ", best=" << best_peak
                      << ", T=" << std::fixed << std::setprecision(2) << temperature
                      << ", accept=" << std::setprecision(1) << (100.0 * acceptance_rate) << "%";
            if (iteration_large_steps > 0) {
                std::cout << ", large_steps=" << iteration_large_accepted << "/" << iteration_large_steps;
            }
            std::cout << std::endl;
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    
    // Use best solution found
    result.ordering = best_ordering;
    result.cutwidth_curve = tracker.getCutwidthCurve();
    result.initial_peak = initial_peak;
    result.final_peak = best_peak;
    result.best_peak_seen = best_peak;
    result.total_swaps_evaluated = total_evaluations;
    result.average_acceptance_rate = (total_evaluations > 0) ? 
        static_cast<double>(accepted_moves) / total_evaluations : 0.0;
    result.final_temperature = temperature;
    result.computation_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    
    if (params.verbose) {
        std::cout << "\n=== Simulated Annealing Complete ===" << std::endl;
        std::cout << "Initial peak: " << result.initial_peak << std::endl;
        std::cout << "Final peak: " << result.final_peak << std::endl;
        std::cout << "Best peak seen: " << result.best_peak_seen << std::endl;
        std::cout << "Improvement: " << (result.initial_peak - result.final_peak);
        if (result.initial_peak > 0) {
            std::cout << " (" << std::fixed << std::setprecision(1) 
                      << (100.0 * (result.initial_peak - result.final_peak) / result.initial_peak) << "%)";
        }
        std::cout << std::endl;
        std::cout << "Total evaluations: " << result.total_swaps_evaluated << std::endl;
        std::cout << "Beneficial moves: " << result.beneficial_swaps << std::endl;
        std::cout << "Harmful moves accepted: " << result.harmful_swaps_accepted << std::endl;
        std::cout << "Large steps attempted: " << result.large_steps_attempted << std::endl;
        std::cout << "Large steps accepted: " << result.large_steps_accepted << std::endl;
        std::cout << "Average acceptance rate: " << std::fixed << std::setprecision(1) 
                  << (100.0 * result.average_acceptance_rate) << "%" << std::endl;
        std::cout << "Final temperature: " << std::setprecision(3) << result.final_temperature << std::endl;
        std::cout << "Total time: " << result.computation_time_ms << " ms" << std::endl;
        std::cout << "Converged: " << (result.converged ? "Yes" : "No") << std::endl;
    }
    
    return result;
} 