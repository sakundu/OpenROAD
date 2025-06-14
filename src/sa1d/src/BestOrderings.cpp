#include "sa1d/BestOrderings.h"
#include "sa1d/OptSA.h"
#include "sa1d/Objects.h"
#include "Worker.h"
#include "utl/Logger.h"

// SAIT includes for best-orderings algorithms
#include "sait/hypergraph.hpp"
#include "sait/fiedler_ordering.hpp"
#include "sait/rcm_ordering.hpp"
#include "sait/bfs_ordering.hpp"
#include "sait/dfs_ordering.hpp"
#include "sait/sfc_ordering.hpp"
#include "sait/dirichlet_ordering.hpp"
#include "sait/peak_cutwidth_ordering.hpp"
#include "sait/cutwidth_analysis.hpp"
#include "sait/random_ordering.hpp"

#include <chrono>
#include <algorithm>
#include <numeric>
#include <climits>
#include <omp.h>

namespace sa1d {

BestOrderingsInterface::BestOrderingsInterface(OptSA* opt_sa)
    : opt_sa_(opt_sa), db_(opt_sa->getDB()), block_(opt_sa->getBlock()), logger_(opt_sa->getLogger()) {
}

BestOrderingsInterface::~BestOrderingsInterface() = default;

BestOrderingsResult BestOrderingsInterface::computeBestOrderings(const BestOrderingsParams& params) {
    auto total_start_time = std::chrono::high_resolution_clock::now();
    BestOrderingsResult result;
    
    try {
        if (params.verbose && logger_) {
            logger_->info(utl::SA1D, 200, "=== BEST ORDERINGS COMPUTATION ===");
        }
        
        // Convert to SAIT hypergraph
        auto hg = convertToSAITHypergraph();
        if (!hg) {
            result.error_message = "Failed to convert to SAIT hypergraph";
            return result;
        }
        
        if (params.verbose && logger_) {
            logger_->info(utl::SA1D, 201, "[+] Hypergraph: {} vertices, {} hyperedges", 
                         hg->num_vertices, hg->num_hyperedges);
            logger_->info(utl::SA1D, 202, "[+] Parallel threads: {}", 
                         params.max_threads > 0 ? params.max_threads : omp_get_max_threads());
        }
        
        // Get available algorithms
        auto algorithms = getAvailableAlgorithms(params.include_advanced_methods);
        result.algorithms_tested = static_cast<int>(algorithms.size());
        
        if (params.verbose && logger_) {
            logger_->info(utl::SA1D, 203, "[+] Testing {} algorithms", algorithms.size());
        }
        
        // Set up parallel execution
        if (params.use_parallel && params.max_threads > 0) {
            omp_set_num_threads(params.max_threads);
        }
        
        // Prepare results storage
        std::vector<std::tuple<std::string, std::vector<int>, int, int, double>> algorithm_results(algorithms.size());
        
        // Process algorithms in parallel
        #pragma omp parallel for schedule(dynamic) if(params.use_parallel)
        for (size_t i = 0; i < algorithms.size(); ++i) {
            const auto& [name, compute_func, requires_terminal] = algorithms[i];
            
            auto start_time = std::chrono::high_resolution_clock::now();
            
            try {
                // Compute base ordering
                auto vertex_ordering = compute_func(*hg);
                
                if (vertex_ordering.empty()) {
                    algorithm_results[i] = std::make_tuple(name, std::vector<int>(), 0, INT_MAX, 0.0);
                    continue;
                }
                
                // Compute initial cutwidth
                auto initial_result = computeCutwidthCurve(*hg, vertex_ordering, false);
                int initial_peak = initial_result.peak_cutwidth;
                
                // Apply refinement if requested
                std::vector<int> final_ordering = vertex_ordering;
                if (params.apply_refinement) {
                    final_ordering = applyRefinement(*hg, vertex_ordering, name, params.use_constrained_refinement);
                }
                
                // Compute final cutwidth
                auto final_result = computeCutwidthCurve(*hg, final_ordering, false);
                int final_peak = final_result.peak_cutwidth;
                
                auto end_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                
                algorithm_results[i] = std::make_tuple(name, final_ordering, initial_peak, final_peak, duration.count());
                
                if (params.verbose && logger_) {
                    #pragma omp critical
                    {
                        logger_->info(utl::SA1D, 204, "[OK] {} completed (peak: {})", name, final_peak);
                    }
                }
                
            } catch (const std::exception& e) {
                algorithm_results[i] = std::make_tuple(name, std::vector<int>(), 0, INT_MAX, 0.0);
                if (params.verbose && logger_) {
                    #pragma omp critical
                    {
                        logger_->warn(utl::SA1D, 205, "[FAIL] {} failed: {}", name, e.what());
                    }
                }
            }
        }
        
        // Filter out failed results and compute HPWL for ranking
        std::vector<std::tuple<std::string, std::vector<int>, int, int, double, double>> valid_results_with_hpwl;
        for (const auto& algo_result : algorithm_results) {
            if (std::get<3>(algo_result) != INT_MAX && !std::get<1>(algo_result).empty()) {
                const auto& [name, vertex_ordering, initial_peak, final_peak, time] = algo_result;
                
                // Compute HPWL for ranking
                auto temp_ordering_info = convertToOrderingInfo(name, vertex_ordering, *hg, time);
                double hpwl = temp_ordering_info.initial_hpwl;
                
                valid_results_with_hpwl.push_back(std::make_tuple(name, vertex_ordering, initial_peak, final_peak, time, hpwl));
            }
        }
        
        // Sort by HPWL (lower is better) instead of peak cutwidth
        std::sort(valid_results_with_hpwl.begin(), valid_results_with_hpwl.end(), 
            [](const auto& a, const auto& b) { return std::get<5>(a) < std::get<5>(b); });
        
        // Convert top results to OrderingInfo format
        int top_count = std::min(params.top_count, static_cast<int>(valid_results_with_hpwl.size()));
        result.top_orderings.reserve(top_count);
        
        for (int i = 0; i < top_count; ++i) {
            const auto& [name, vertex_ordering, initial_peak, final_peak, time, hpwl] = valid_results_with_hpwl[i];
            
            auto ordering_info = convertToOrderingInfo(name, vertex_ordering, *hg, time);
            ordering_info.initial_peak_cutwidth = initial_peak;
            ordering_info.final_peak_cutwidth = final_peak;
            ordering_info.improvement_percentage = (initial_peak > 0) ? 
                100.0 * (initial_peak - final_peak) / initial_peak : 0.0;
            
            result.top_orderings.push_back(std::move(ordering_info));
        }
        
        auto total_end_time = std::chrono::high_resolution_clock::now();
        result.total_computation_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            total_end_time - total_start_time).count();
        
        result.success = !result.top_orderings.empty();
        
        if (params.verbose && logger_ && result.success) {
            logger_->info(utl::SA1D, 206, "=== TOP {} BEST ORDERINGS (Ranked by HPWL) ===", top_count);
            for (size_t i = 0; i < result.top_orderings.size(); ++i) {
                const auto& info = result.top_orderings[i];
                logger_->info(utl::SA1D, 207, "{}. {} - HPWL: {:.0f}, Peak: {} -> {} ({:.1f}% improvement), {:.0f}ms",
                             i + 1, info.algorithm_name, info.initial_hpwl, info.initial_peak_cutwidth, 
                             info.final_peak_cutwidth, info.improvement_percentage, info.computation_time_ms);
            }
            logger_->info(utl::SA1D, 208, "Total computation time: {:.0f}ms", result.total_computation_time_ms);
        }
        
    } catch (const std::exception& e) {
        result.error_message = std::string("Best orderings computation failed: ") + e.what();
        if (logger_) {
            logger_->error(utl::SA1D, 300, "{}", result.error_message);
        }
    }
    
    return result;
}

std::vector<BestOrderingsInterface::WorkerInitData> BestOrderingsInterface::prepareWorkerInitialization(
    const BestOrderingsResult& result, int num_workers) {
    
    std::vector<WorkerInitData> worker_data(num_workers);
    
    if (!result.success || result.top_orderings.empty()) {
        // All workers use random initialization
        for (int i = 0; i < num_workers; ++i) {
            worker_data[i].use_ordering = false;
            worker_data[i].algorithm_name = "random";
        }
        return worker_data;
    }
    
    // Distribute top orderings cyclically across ALL workers
    int top_count = static_cast<int>(result.top_orderings.size());
    
    for (int i = 0; i < num_workers; ++i) {
        // Cycle through the available top orderings
        int ordering_index = i % top_count;
        const auto& info = result.top_orderings[ordering_index];
        
        worker_data[i].cell_ordering = info.cell_ordering;
        worker_data[i].orientations = info.orientations;
        worker_data[i].algorithm_name = info.algorithm_name;
        worker_data[i].use_ordering = true;
    }
    
    if (logger_) {
        // Count how many workers get each ordering
        std::vector<int> counts(top_count, 0);
        for (int i = 0; i < num_workers; ++i) {
            counts[i % top_count]++;
        }
        
        logger_->info(utl::SA1D, 209, "Worker initialization: all {} workers use best orderings", num_workers);
        for (int i = 0; i < top_count; ++i) {
            const auto& info = result.top_orderings[i];
            logger_->info(utl::SA1D, 220, "  {} workers use {} ordering (HPWL: {:.0f})", counts[i], info.algorithm_name, info.initial_hpwl);
        }
    }
    
    return worker_data;
}

std::vector<BestOrderingsInterface::AlgorithmInfo> BestOrderingsInterface::getAvailableAlgorithms(bool include_advanced) {
    std::vector<AlgorithmInfo> algorithms;
    
    // Basic algorithms (always available) - return full ordering, filtering happens later
    algorithms.push_back({"Fiedler", 
        [](const ::Hypergraph& hg) { 
            return computeFiedlerOrdering(hg); 
        }, false});
    
    algorithms.push_back({"RCM", 
        [](const ::Hypergraph& hg) { 
            return computeRCMOrdering(hg); 
        }, false});
    
    algorithms.push_back({"RCM-Boost", 
        [](const ::Hypergraph& hg) { 
            return computeRCMOrderingBoost(hg); 
        }, false});
    
    algorithms.push_back({"BFS", 
        [](const ::Hypergraph& hg) { 
            return computeBFSOrdering(hg, -1); 
        }, false});
    
    algorithms.push_back({"DFS", 
        [](const ::Hypergraph& hg) { 
            return computeDFSOrdering(hg, -1); 
        }, false});
    
    algorithms.push_back({"Random", 
        [](const ::Hypergraph& hg) { 
            return computeRandomOrdering(hg); 
        }, false});
    
    // Space-filling curve algorithms
    algorithms.push_back({"SFC-Hilbert2D", 
        [](const ::Hypergraph& hg) { 
            return computeSFCOrdering(hg, SFCStrategy::HILBERT_2D); 
        }, false});
    
    algorithms.push_back({"SFC-ZOrder2D", 
        [](const ::Hypergraph& hg) { 
            return computeSFCOrdering(hg, SFCStrategy::ZORDER_2D); 
        }, false});
    
         // Advanced algorithms (if requested and I/O terminals exist)
     if (include_advanced && hasIOTerminals()) {
         algorithms.push_back({"Dirichlet", 
             [this](const ::Hypergraph& hg) { 
                 try {
                     auto fixed_vertices = convertToSAITFixedVertices();
                     if (fixed_vertices.empty()) {
                         return std::vector<int>();  // No fixed vertices available
                     }
                     
                     DirichletEmbeddingParams params;
                     params.verbose = false;
                     params.normalize_coordinates = true;
                     auto result = computeDirichletEmbedding(hg, fixed_vertices, params);
                     
                     return result.ordering;
                 } catch (const std::exception&) {
                     return std::vector<int>();  // Return empty on failure
                 }
             }, true});
             
         algorithms.push_back({"Soft-Penalty", 
             [this](const ::Hypergraph& hg) { 
                 try {
                     auto fixed_vertices = convertToSAITFixedVertices();
                     if (fixed_vertices.empty()) {
                         return std::vector<int>();
                     }
                     
                     SoftAnchoredEmbeddingParams params;
                     params.verbose = false;
                     params.normalize_coordinates = true;
                     params.method = SoftAnchoringMethod::PENALTY_METHOD;
                     params.penalty_parameter = 100.0;
                     auto result = computeSoftAnchoredEmbedding(hg, fixed_vertices, params);
                     
                     // Create ordering from coordinates
                     std::vector<std::pair<double, int>> coords_with_ids;
                     for (int v = 0; v < hg.num_vertices; ++v) {
                         coords_with_ids.emplace_back(result.coordinates[v], v);
                     }
                     std::sort(coords_with_ids.begin(), coords_with_ids.end());
                     
                     std::vector<int> full_ordering;
                     for (const auto& pair : coords_with_ids) {
                         full_ordering.push_back(pair.second);
                     }
                     
                     return full_ordering;
                 } catch (const std::exception&) {
                     return std::vector<int>();
                 }
             }, true});
             
         algorithms.push_back({"Soft-Springs", 
             [this](const ::Hypergraph& hg) { 
                 try {
                     auto fixed_vertices = convertToSAITFixedVertices();
                     if (fixed_vertices.empty()) {
                         return std::vector<int>();
                     }
                     
                     SoftAnchoredEmbeddingParams params;
                     params.verbose = false;
                     params.normalize_coordinates = true;
                     params.method = SoftAnchoringMethod::VIRTUAL_SPRINGS;
                     params.virtual_spring_weight = 100.0;
                     auto result = computeSoftAnchoredEmbedding(hg, fixed_vertices, params);
                     
                     // Create ordering from coordinates
                     std::vector<std::pair<double, int>> coords_with_ids;
                     for (int v = 0; v < hg.num_vertices; ++v) {
                         coords_with_ids.emplace_back(result.coordinates[v], v);
                     }
                     std::sort(coords_with_ids.begin(), coords_with_ids.end());
                     
                     std::vector<int> full_ordering;
                     for (const auto& pair : coords_with_ids) {
                         full_ordering.push_back(pair.second);
                     }
                     
                     return full_ordering;
                 } catch (const std::exception&) {
                     return std::vector<int>();
                 }
             }, true});
     }
    
    return algorithms;
}

std::unique_ptr<::Hypergraph> BestOrderingsInterface::convertToSAITHypergraph() {
    const auto& cells = opt_sa_->getCells();
    const auto& nets = opt_sa_->getNets();
    
    if (logger_) {
        logger_->info(utl::SA1D, 301, "Converting to SAIT hypergraph: {} cells, {} nets", 
                     cells.size(), nets.size());
    }
    
    if (cells.empty()) {
        if (logger_) {
            logger_->error(utl::SA1D, 302, "No cells found - SA1D database may not be initialized");
        }
        return nullptr;
    }
    
    if (nets.empty()) {
        if (logger_) {
            logger_->error(utl::SA1D, 303, "No nets found - SA1D database may not be initialized");
        }
        return nullptr;
    }
    
    // Count I/O terminals (nets with bterm_flag = true)
    int num_io_terminals = 0;
    for (const auto& net : nets) {
        if (net.bterm_flag) {
            num_io_terminals++;
        }
    }
    
    // Create hypergraph: vertices = cells + I/O terminals
    int total_vertices = cells.size() + num_io_terminals;
    auto hg = std::make_unique<Hypergraph>(total_vertices, nets.size());
    
    // Clear mapping tables
    cell_to_vertex_map_.clear();
    vertex_to_cell_map_.clear();
    io_net_to_vertex_map_.clear();
    vertex_to_io_net_map_.clear();
    
    // Create cell ID to vertex ID mapping (0-indexed for SAIT)
    // Cells get vertex IDs [0, cells.size())
    for (size_t cell_id = 0; cell_id < cells.size(); ++cell_id) {
        cell_to_vertex_map_[cell_id] = static_cast<int>(cell_id);
        vertex_to_cell_map_[static_cast<int>(cell_id)] = static_cast<int>(cell_id);
    }
    
    // Map I/O terminals to vertices [cells.size(), total_vertices)
    int io_vertex_id = cells.size();
    
    // First pass: assign vertex IDs to I/O terminals
    for (size_t net_id = 0; net_id < nets.size(); ++net_id) {
        const auto& net = nets[net_id];
        if (net.bterm_flag) {
            io_net_to_vertex_map_[net_id] = io_vertex_id;
            vertex_to_io_net_map_[io_vertex_id] = net_id;
            io_vertex_id++;
        }
    }
    
    // Convert nets to hyperedges
    for (size_t net_id = 0; net_id < nets.size(); ++net_id) {
        const auto& net = nets[net_id];
        std::vector<int> hyperedge_vertices;
        
        // Add all cells connected to this net
        for (const auto& term : net.getTerms()) {
            int cell_id = term.cell_id;
            if (cell_to_vertex_map_.find(cell_id) != cell_to_vertex_map_.end()) {
                int vertex_id = cell_to_vertex_map_[cell_id];
                hyperedge_vertices.push_back(vertex_id);
            }
        }
        
        // Add I/O terminal vertex if this net has bTerms
        if (net.bterm_flag) {
            auto it = io_net_to_vertex_map_.find(net_id);
            if (it != io_net_to_vertex_map_.end()) {
                hyperedge_vertices.push_back(it->second);
            }
        }
        
        // Only add hyperedge if it has at least 2 vertices
        if (hyperedge_vertices.size() >= 2) {
            hg->addHyperedge(hyperedge_vertices);
        }
    }
    
    // Build vertex-to-hyperedge mapping
    hg->buildVertexToHyperedgeMapping();
    
    return hg;
}

std::vector<int> BestOrderingsInterface::applyRefinement(const ::Hypergraph& hg, 
                                                        const std::vector<int>& vertex_ordering,
                                                        const std::string& algorithm_name,
                                                        bool use_constrained) {
    // For now, use simple greedy refinement
    // In full implementation, this would use the sophisticated refinement from SAIT
    
    try {
        // Apply peak cutwidth optimization
        PeakCutwidthParams params;
        params.verbose = false;
        params.max_iterations = 20;
        params.use_random_start = false;
        params.use_hyperedge_aware = true;
        
        auto result = computeEnhancedRefinement(hg, vertex_ordering, params);
        return result.ordering;
        
    } catch (const std::exception&) {
        // Fall back to original ordering if refinement fails
        return vertex_ordering;
    }
}

BestOrderingsResult::OrderingInfo BestOrderingsInterface::convertToOrderingInfo(
    const std::string& algorithm_name,
    const std::vector<int>& vertex_ordering,
    const ::Hypergraph& hg,
    double computation_time_ms) {
    
    BestOrderingsResult::OrderingInfo info;
    info.algorithm_name = algorithm_name;
    info.computation_time_ms = computation_time_ms;
    
    // Convert vertex ordering to cell ordering (filter out I/O terminals)  
    const auto& cells = opt_sa_->getCells();
    int num_cells = static_cast<int>(cells.size());
    
    // Filter the full vertex ordering to get only cell vertices
    std::vector<int> filtered_ordering = filterCellVertices(vertex_ordering, num_cells);
    
    info.cell_ordering = filtered_ordering;
    
    // Initialize orientations
    info.orientations.clear();
    info.orientations.reserve(info.cell_ordering.size());
    
    for (int cell_id : info.cell_ordering) {
        if (cell_id < static_cast<int>(cells.size())) {
            const auto& cell = cells[cell_id];
            if (cell.db_inst) {
                info.orientations.push_back(cell.db_inst->getOrient());
            } else {
                info.orientations.push_back(odb::dbOrientType::R0);
            }
        }
    }
    
    // Compute HPWL
    info.initial_hpwl = computeHPWL(info.cell_ordering);
    info.final_hpwl = info.initial_hpwl;  // Same for now
    
    // Compute overlap metrics
    info.initial_peak_cutwidth = computeOverlap(info.cell_ordering);
    info.final_peak_cutwidth = info.initial_peak_cutwidth;  // Same for now
    
    return info;
}

double BestOrderingsInterface::computeHPWL(const std::vector<int>& cell_ordering) {
    if (cell_ordering.empty()) return 0.0;
    
    // Create orientations vector  
    std::vector<odb::dbOrientType> orients;
    orients.reserve(cell_ordering.size());
    
    const auto& cells = opt_sa_->getCells();
    for (int cell_id : cell_ordering) {
        if (cell_id < static_cast<int>(cells.size())) {
            const auto& cell = cells[cell_id];
            if (cell.db_inst) {
                orients.push_back(cell.db_inst->getOrient());
            } else {
                orients.push_back(odb::dbOrientType::R0);
            }
        }
    }
    
    SAWorker worker(opt_sa_, logger_, 0);
    worker.initCellOrder(cell_ordering, orients);
    return static_cast<double>(worker.getTotalHPWL());
}

int BestOrderingsInterface::computeOverlap(const std::vector<int>& cell_ordering) {
    if (cell_ordering.empty()) return 0;
    
    // Create orientations vector  
    std::vector<odb::dbOrientType> orients;
    orients.reserve(cell_ordering.size());
    
    const auto& cells = opt_sa_->getCells();
    for (int cell_id : cell_ordering) {
        if (cell_id < static_cast<int>(cells.size())) {
            const auto& cell = cells[cell_id];
            if (cell.db_inst) {
                orients.push_back(cell.db_inst->getOrient());
            } else {
                orients.push_back(odb::dbOrientType::R0);
            }
        }
    }
    
    // Create a temporary worker to compute overlap
    SAWorker temp_worker(opt_sa_, logger_, -1); // Use -1 to indicate temp worker
    temp_worker.setEnableOverlap(true);
    temp_worker.initCellOrder(cell_ordering, orients);
    
    return temp_worker.getPeakOverlap();
}

bool BestOrderingsInterface::validateOrdering(const std::vector<int>& ordering) {
    const auto& cells = opt_sa_->getCells();
    
    if (ordering.size() != cells.size()) {
        return false;
    }
    
    // Check that all cell IDs are valid and unique
    std::vector<bool> seen(cells.size(), false);
    for (int cell_id : ordering) {
        if (cell_id < 0 || cell_id >= static_cast<int>(cells.size())) {
            return false;
        }
        if (seen[cell_id]) {
            return false;  // Duplicate
        }
        seen[cell_id] = true;
    }
    
    return true;
}

bool BestOrderingsInterface::hasIOTerminals() const {
    const auto& nets = opt_sa_->getNets();
    for (const auto& net : nets) {
        if (net.bterm_flag) {
            return true;
        }
    }
    return false;
}

std::vector<int> BestOrderingsInterface::filterCellVertices(const std::vector<int>& vertex_ordering, int num_cells) const {
    std::vector<int> cell_ordering;
    cell_ordering.reserve(num_cells);
    
    // Convert vertex IDs to cell IDs using the mapping
    for (int vertex_id : vertex_ordering) {
        // Check if this vertex corresponds to a cell (not I/O terminal)
        auto it = vertex_to_cell_map_.find(vertex_id);
        if (it != vertex_to_cell_map_.end()) {
            int cell_id = it->second;
            if (cell_id >= 0 && cell_id < num_cells) {
                cell_ordering.push_back(cell_id);
            }
        }
    }
    
    // Ensure we have exactly the right number of cells
    if (static_cast<int>(cell_ordering.size()) < num_cells) {
        std::vector<bool> included(num_cells, false);
        for (int cell_id : cell_ordering) {
            if (cell_id >= 0 && cell_id < num_cells) {
                included[cell_id] = true;
            }
        }
        
        // Add missing cells at the end in order
        for (int cell_id = 0; cell_id < num_cells; ++cell_id) {
            if (!included[cell_id]) {
                cell_ordering.push_back(cell_id);
            }
        }
    }
    
    // Ensure exactly num_cells elements
    if (static_cast<int>(cell_ordering.size()) > num_cells) {
        cell_ordering.resize(num_cells);
    }
    
    return cell_ordering;
}

std::vector<::FixedVertex> BestOrderingsInterface::extractIOCoordinates() const {
    std::vector<::FixedVertex> fixed_vertices;
    const auto& nets = opt_sa_->getNets();
    
    for (const auto& entry : io_net_to_vertex_map_) {
        size_t net_id = entry.first;
        int vertex_id = entry.second;
        
        if (net_id < nets.size()) {
            const auto& net = nets[net_id];
            if (net.bterm_flag) {
                double x_coord = (net.bterm_box.xMin() + net.bterm_box.xMax()) / 2.0;
                // SAIT FixedVertex only takes (id, x) - it's for 1D embeddings
                fixed_vertices.emplace_back(vertex_id, x_coord);
            }
        }
    }
    
    return fixed_vertices;
}

std::vector<::FixedVertex> BestOrderingsInterface::convertToSAITFixedVertices() const {
    std::vector<::FixedVertex> sait_fixed_vertices;
    const auto& nets = opt_sa_->getNets();
    
    for (const auto& entry : io_net_to_vertex_map_) {
        size_t net_id = entry.first;
        int vertex_id = entry.second;
        
        if (net_id < nets.size()) {
            const auto& net = nets[net_id];
            if (net.bterm_flag) {
                double x_coord = (net.bterm_box.xMin() + net.bterm_box.xMax()) / 2.0;
                // Use SAIT FixedVertex constructor: FixedVertex(int id, double x)
                sait_fixed_vertices.emplace_back(vertex_id, x_coord);
            }
        }
    }
    
    return sait_fixed_vertices;
}

} // namespace sa1d 