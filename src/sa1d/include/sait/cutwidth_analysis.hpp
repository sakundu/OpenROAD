#pragma once

#include "sait/hypergraph.hpp"
#include <vector>
#include <string>

/**
 * @brief Structure to hold cutwidth analysis results
 */
struct CutwidthResult {
    std::vector<int> cutwidth_curve;  // cut(k) for k = 1 to n-1
    int peak_cutwidth;                // maximum cut size
    int peak_position;                // position where peak occurs
    double average_cutwidth;          // average cut size
    
    CutwidthResult() : peak_cutwidth(0), peak_position(0), average_cutwidth(0.0) {}
};

/**
 * @brief Event structure for the sweep algorithm
 */
struct CutEvent {
    int position;      // Position in the ordering where event occurs
    int hyperedge_id;  // Which hyperedge is affected
    bool is_start;     // true = hyperedge starts being cut, false = stops being cut
    
    CutEvent(int pos, int he_id, bool start) 
        : position(pos), hyperedge_id(he_id), is_start(start) {}
    
    // Comparator for sorting events
    bool operator<(const CutEvent& other) const {
        if (position != other.position) {
            return position < other.position;
        }
        // Process "stop being cut" events before "start being cut" at same position
        return !is_start && other.is_start;
    }
};

/**
 * @brief Compute cutwidth curve using efficient event-based sweep algorithm
 * @param hg Input hypergraph
 * @param ordering Vertex ordering (0-indexed vertex IDs in order)
 * @param verbose Whether to print progress messages (default: true)
 * @return CutwidthResult containing the complete analysis
 */
CutwidthResult computeCutwidthCurve(const Hypergraph& hg, const std::vector<int>& ordering, bool verbose = true);

/**
 * @brief Write cutwidth curve to CSV file
 * @param result Cutwidth analysis result
 * @param filename Output CSV file path
 * @param ordering_name Name/description of the ordering for metadata
 */
void writeCutwidthCSV(const CutwidthResult& result, const std::string& filename, 
                      const std::string& ordering_name = "");

/**
 * @brief Print cutwidth analysis summary
 * @param result Cutwidth analysis result
 * @param ordering_name Name/description of the ordering
 */
void printCutwidthSummary(const CutwidthResult& result, const std::string& ordering_name);

/**
 * @brief Compare multiple cutwidth curves
 * @param results Vector of cutwidth results to compare
 * @param names Names corresponding to each result
 * @param output_filename Output file for comparison CSV
 */
void compareCutwidthCurves(const std::vector<CutwidthResult>& results,
                          const std::vector<std::string>& names,
                          const std::string& output_filename); 