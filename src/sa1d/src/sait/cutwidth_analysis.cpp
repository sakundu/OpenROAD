#include "sait/cutwidth_analysis.hpp"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <unordered_set>
#include <numeric>
#include <limits>

CutwidthResult computeCutwidthCurve(const Hypergraph& hg, const std::vector<int>& ordering, bool verbose) {
    CutwidthResult result;
    
    // Validate input
    if (ordering.size() != static_cast<size_t>(hg.num_vertices)) {
        std::cerr << "Error: Ordering size (" << ordering.size() 
                  << ") doesn't match number of vertices (" << hg.num_vertices << ")" << std::endl;
        return result;
    }
    
    if (hg.num_vertices <= 1) {
        // Trivial case
        result.cutwidth_curve.clear();
        result.peak_cutwidth = 0;
        result.peak_position = 0;
        result.average_cutwidth = 0.0;
        return result;
    }
    
    if (verbose) {
        std::cout << "Computing cutwidth curve using event-based sweep algorithm..." << std::endl;
    }
    
    // Create position mapping: vertex_id -> position in ordering
    std::vector<int> vertex_position(hg.num_vertices);
    for (size_t i = 0; i < ordering.size(); ++i) {
        vertex_position[ordering[i]] = i;
    }
    
    // Generate events for each hyperedge
    std::vector<CutEvent> events;
    events.reserve(2 * hg.num_hyperedges); // At most 2 events per hyperedge
    
    for (int he = 0; he < hg.num_hyperedges; ++he) {
        const auto& vertices = hg.hyperedges[he];
        
        if (vertices.size() <= 1) {
            // Single-vertex hyperedges never contribute to cuts
            continue;
        }
        
        // Find min and max positions of vertices in this hyperedge
        int min_pos = std::numeric_limits<int>::max();
        int max_pos = std::numeric_limits<int>::min();
        
        for (int vertex : vertices) {
            if (vertex < 0 || vertex >= hg.num_vertices) {
                std::cerr << "Warning: Invalid vertex " << vertex << " in hyperedge " << he << std::endl;
                continue;
            }
            int pos = vertex_position[vertex];
            min_pos = std::min(min_pos, pos);
            max_pos = std::max(max_pos, pos);
        }
        
        if (min_pos == max_pos) {
            // All vertices in same position (shouldn't happen with valid ordering)
            continue;
        }
        
        // Hyperedge starts being cut after position min_pos (when left partition has ≥1 vertex)
        // Hyperedge stops being cut after position max_pos (when left partition has all vertices)
        events.emplace_back(min_pos + 1, he, true);   // Start being cut at position min_pos + 1
        events.emplace_back(max_pos + 1, he, false);  // Stop being cut at position max_pos + 1
    }
    
    // Sort events by position
    std::sort(events.begin(), events.end());
    
    if (verbose) {
        std::cout << "Generated " << events.size() << " events for " << hg.num_hyperedges << " hyperedges" << std::endl;
    }
    
    // Sweep through positions and maintain cut count
    result.cutwidth_curve.resize(hg.num_vertices - 1, 0);
    
    int current_cut_count = 0;
    size_t event_index = 0;
    
    for (int k = 1; k < hg.num_vertices; ++k) {
        // Process all events at position k
        while (event_index < events.size() && events[event_index].position == k) {
            const CutEvent& event = events[event_index];
            
            if (event.is_start) {
                current_cut_count++;
            } else {
                current_cut_count--;
            }
            
            event_index++;
        }
        
        // Store cut size at position k
        result.cutwidth_curve[k - 1] = current_cut_count;
    }
    
    // Compute statistics
    if (!result.cutwidth_curve.empty()) {
        // Find peak cutwidth
        auto max_it = std::max_element(result.cutwidth_curve.begin(), result.cutwidth_curve.end());
        result.peak_cutwidth = *max_it;
        result.peak_position = std::distance(result.cutwidth_curve.begin(), max_it) + 1;
        
        // Compute average cutwidth
        long long sum = std::accumulate(result.cutwidth_curve.begin(), result.cutwidth_curve.end(), 0LL);
        result.average_cutwidth = static_cast<double>(sum) / result.cutwidth_curve.size();
    }
    
    if (verbose) {
        std::cout << "Cutwidth curve computation complete" << std::endl;
        std::cout << "Peak cutwidth: " << result.peak_cutwidth << " at position " << result.peak_position << std::endl;
        std::cout << "Average cutwidth: " << std::fixed << std::setprecision(2) << result.average_cutwidth << std::endl;
    }
    
    return result;
}

void writeCutwidthCSV(const CutwidthResult& result, const std::string& filename, 
                      const std::string& ordering_name) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot create CSV file " << filename << std::endl;
        return;
    }
    
    // Write header with metadata
    file << "# Cutwidth Curve Analysis\n";
    if (!ordering_name.empty()) {
        file << "# Ordering: " << ordering_name << "\n";
    }
    file << "# Peak cutwidth: " << result.peak_cutwidth << " at position " << result.peak_position << "\n";
    file << "# Average cutwidth: " << std::fixed << std::setprecision(4) << result.average_cutwidth << "\n";
    file << "# Total positions: " << result.cutwidth_curve.size() << "\n";
    file << "#\n";
    
    // Write CSV header
    file << "position_k,cut_size\n";
    
    // Write data
    for (size_t k = 0; k < result.cutwidth_curve.size(); ++k) {
        file << (k + 1) << "," << result.cutwidth_curve[k] << "\n";
    }
    
    file.close();
    std::cout << "Cutwidth curve written to " << filename << std::endl;
}

void printCutwidthSummary(const CutwidthResult& result, const std::string& ordering_name) {
    std::cout << "\n=== Cutwidth Analysis Summary";
    if (!ordering_name.empty()) {
        std::cout << " (" << ordering_name << ")";
    }
    std::cout << " ===\n";
    
    std::cout << "Total positions analyzed: " << result.cutwidth_curve.size() << "\n";
    std::cout << "Peak cutwidth: " << result.peak_cutwidth << " at position " << result.peak_position << "\n";
    std::cout << "Average cutwidth: " << std::fixed << std::setprecision(2) << result.average_cutwidth << "\n";
    
    if (!result.cutwidth_curve.empty()) {
        // Additional statistics
        int min_cutwidth = *std::min_element(result.cutwidth_curve.begin(), result.cutwidth_curve.end());
        
        // Find 10th, 50th, 90th percentiles
        std::vector<int> sorted_cuts = result.cutwidth_curve;
        std::sort(sorted_cuts.begin(), sorted_cuts.end());
        
        size_t n = sorted_cuts.size();
        int p10 = sorted_cuts[n * 10 / 100];
        int p50 = sorted_cuts[n * 50 / 100]; // median
        int p90 = sorted_cuts[n * 90 / 100];
        
        std::cout << "Min cutwidth: " << min_cutwidth << "\n";
        std::cout << "Percentiles - 10th: " << p10 << ", 50th: " << p50 << ", 90th: " << p90 << "\n";
        
        // Show first and last few values
        std::cout << "First 10 cut sizes: ";
        for (size_t i = 0; i < std::min(size_t(10), result.cutwidth_curve.size()); ++i) {
            std::cout << result.cutwidth_curve[i];
            if (i < std::min(size_t(9), result.cutwidth_curve.size() - 1)) std::cout << " ";
        }
        std::cout << "\n";
        
        if (result.cutwidth_curve.size() > 10) {
            std::cout << "Last 10 cut sizes: ";
            size_t start = result.cutwidth_curve.size() - std::min(size_t(10), result.cutwidth_curve.size());
            for (size_t i = start; i < result.cutwidth_curve.size(); ++i) {
                std::cout << result.cutwidth_curve[i];
                if (i < result.cutwidth_curve.size() - 1) std::cout << " ";
            }
            std::cout << "\n";
        }
    }
    
    std::cout << "============================================\n\n";
}

void compareCutwidthCurves(const std::vector<CutwidthResult>& results,
                          const std::vector<std::string>& names,
                          const std::string& output_filename) {
    if (results.empty() || results.size() != names.size()) {
        std::cerr << "Error: Invalid input for cutwidth curve comparison" << std::endl;
        return;
    }
    
    std::ofstream file(output_filename);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot create comparison CSV file " << output_filename << std::endl;
        return;
    }
    
    // Write header with metadata
    file << "# Cutwidth Curve Comparison\n";
    file << "# Number of orderings: " << results.size() << "\n";
    for (size_t i = 0; i < names.size(); ++i) {
        file << "# " << (i + 1) << ". " << names[i] 
             << " - Peak: " << results[i].peak_cutwidth 
             << ", Avg: " << std::fixed << std::setprecision(2) << results[i].average_cutwidth << "\n";
    }
    file << "#\n";
    
    // Find maximum curve length
    size_t max_length = 0;
    for (const auto& result : results) {
        max_length = std::max(max_length, result.cutwidth_curve.size());
    }
    
    // Write CSV header
    file << "position_k";
    for (const auto& name : names) {
        file << "," << name;
    }
    file << "\n";
    
    // Write data
    for (size_t k = 0; k < max_length; ++k) {
        file << (k + 1);
        for (const auto& result : results) {
            file << ",";
            if (k < result.cutwidth_curve.size()) {
                file << result.cutwidth_curve[k];
            } else {
                file << ""; // Empty for shorter curves
            }
        }
        file << "\n";
    }
    
    file.close();
    std::cout << "Cutwidth curve comparison written to " << output_filename << std::endl;
    
    // Print comparison summary
    std::cout << "\n=== Cutwidth Comparison Summary ===\n";
    std::cout << std::left << std::setw(20) << "Ordering" 
              << std::setw(12) << "Peak Cut" 
              << std::setw(12) << "Avg Cut" 
              << std::setw(12) << "Peak Pos" << "\n";
    std::cout << std::string(56, '-') << "\n";
    
    for (size_t i = 0; i < results.size(); ++i) {
        std::cout << std::left << std::setw(20) << names[i]
                  << std::setw(12) << results[i].peak_cutwidth
                  << std::setw(12) << std::fixed << std::setprecision(1) << results[i].average_cutwidth
                  << std::setw(12) << results[i].peak_position << "\n";
    }
    std::cout << "=========================================\n\n";
} 