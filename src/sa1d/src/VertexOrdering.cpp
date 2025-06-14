#include "sa1d/VertexOrdering.h"
#include "sa1d/OptSA.h"
#include "sa1d/Objects.h"
#include "Worker.h"
#include "utl/Logger.h"

// SAIT includes
#include "sait/hypergraph.hpp"
#include "sait/fiedler_ordering.hpp"
#include "sait/rcm_ordering.hpp"
#include "sait/random_ordering.hpp"

#include <chrono>
#include <algorithm>
#include <numeric>
#include <random>

namespace sa1d {

VertexOrderingInterface::VertexOrderingInterface(OptSA* opt_sa)
    : opt_sa_(opt_sa), db_(opt_sa->getDB()), block_(opt_sa->getBlock()), logger_(opt_sa->getLogger()) {
}

VertexOrderingInterface::~VertexOrderingInterface() = default;

VertexOrderingResult VertexOrderingInterface::computeOrdering(const VertexOrderingParams& params) {
    auto start_time = std::chrono::high_resolution_clock::now();
    VertexOrderingResult result;
    result.algorithm_used = orderingMethodToString(params.method);
    
    try {
        if (params.verbose && logger_) {
            logger_->info(utl::SA1D, 100, "Computing vertex ordering using {}", result.algorithm_used);
        }
        
        // Compute initial HPWL for comparison
        result.initial_hpwl = computeCurrentHPWL();
        
        // Use SAIT for advanced algorithms
        if (params.method == OrderingMethod::FIEDLER || 
            params.method == OrderingMethod::RCM ||
            params.method == OrderingMethod::RCM_BOOST) {
            
            // Convert to SAIT hypergraph
            auto hg = convertToSAITHypergraph();
            
            if (!hg || !validateHypergraph(*hg)) {
                result.error_message = "Failed to convert to valid SAIT hypergraph";
                return result;
            }
            
            if (params.verbose && logger_) {
                logger_->info(utl::SA1D, 101, "Converted to hypergraph: {} vertices, {} hyperedges",
                             hg->num_vertices, hg->num_hyperedges);
            }
            
            result.vertices_processed = hg->num_vertices;
            result.hyperedges_processed = hg->num_hyperedges;
            
            // Run SAIT algorithm
            result = runSAITAlgorithm(*hg, params);
            
        } else {
            // Use built-in methods for simple algorithms
            switch (params.method) {
                case OrderingMethod::RANDOM:
                    result = computeRandomOrdering();
                    break;
                    
                case OrderingMethod::SIZE_BASED:
                    result = computeSizeBasedOrdering();
                    break;
                    
                default:
                    result.error_message = "Unsupported ordering method";
                    return result;
            }
        }
        
        // Compute final HPWL and overlap if ordering succeeded
        if (result.success && !result.cell_ordering.empty()) {
            result.final_hpwl = computeOrderingHPWL(result.cell_ordering);
            result.initial_overlap = computeCurrentOverlap();
            result.final_overlap = computeOrderingOverlap(result.cell_ordering);
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        result.computation_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time).count();
        
        if (params.verbose && logger_ && result.success) {
            double improvement = result.initial_hpwl > 0 ? 
                100.0 * (result.initial_hpwl - result.final_hpwl) / result.initial_hpwl : 0.0;
            logger_->info(utl::SA1D, 102, "Vertex ordering complete: {} cells, HPWL {:.0f} -> {:.0f} ({:+.1f}%), {:.1f} ms",
                         result.cell_ordering.size(), result.initial_hpwl, result.final_hpwl, 
                         improvement, result.computation_time_ms);
        }
        
    } catch (const std::exception& e) {
        result.error_message = std::string("Exception in vertex ordering: ") + e.what();
        if (logger_) {
            logger_->error(utl::SA1D, 200, "{}", result.error_message);
        }
    }
    
    return result;
}

std::unique_ptr<Hypergraph> VertexOrderingInterface::convertToSAITHypergraph() {
    const auto& cells = opt_sa_->getCells();
    const auto& nets = opt_sa_->getNets();
    
    if (cells.empty() || nets.empty()) {
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
    int valid_hyperedges = 0;
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
            valid_hyperedges++;
        }
    }
    
    // Build vertex-to-hyperedge mapping
    hg->buildVertexToHyperedgeMapping();
    
    if (logger_) {
        logger_->info(utl::SA1D, 103, "Hypergraph conversion: {} cells + {} I/O terminals -> {} vertices, {} nets -> {} valid hyperedges",
                     cells.size(), num_io_terminals, hg->num_vertices, nets.size(), valid_hyperedges);
    }
    
    return hg;
}

VertexOrderingResult VertexOrderingInterface::runSAITAlgorithm(const Hypergraph& hg, const VertexOrderingParams& params) {
    VertexOrderingResult result;
    
    try {
        std::vector<int> vertex_ordering;
        
        switch (params.method) {
            case OrderingMethod::FIEDLER: {
                if (logger_) {
                    logger_->info(utl::SA1D, 104, "Running SAIT Fiedler ordering...");
                }
                vertex_ordering = computeFiedlerOrdering(hg);
                break;
            }
            
            case OrderingMethod::RCM: {
                if (logger_) {
                    logger_->info(utl::SA1D, 105, "Running SAIT RCM ordering...");
                }
                vertex_ordering = computeRCMOrdering(hg);
                break;
            }
            
            case OrderingMethod::RCM_BOOST: {
                if (logger_) {
                    logger_->info(utl::SA1D, 106, "Running SAIT RCM-Boost ordering...");
                }
                vertex_ordering = computeRCMOrderingBoost(hg);
                break;
            }
            
            default:
                result.error_message = "Unsupported SAIT algorithm";
                return result;
        }
        
        if (vertex_ordering.empty()) {
            result.error_message = "SAIT algorithm returned empty ordering";
            return result;
        }
        
        if (static_cast<int>(vertex_ordering.size()) != hg.num_vertices) {
            result.error_message = "SAIT algorithm returned incomplete ordering";
            return result;
        }
        
        // Convert back to SA1D format
        result = convertFromSAITOrdering(vertex_ordering);
        
        if (logger_) {
            logger_->info(utl::SA1D, 107, "SAIT algorithm completed: {} vertices ordered", vertex_ordering.size());
        }
        
    } catch (const std::exception& e) {
        result.error_message = std::string("SAIT algorithm failed: ") + e.what();
        if (logger_) {
            logger_->error(utl::SA1D, 201, "{}", result.error_message);
        }
    }
    
    return result;
}

VertexOrderingResult VertexOrderingInterface::convertFromSAITOrdering(const std::vector<int>& vertex_ordering) {
    VertexOrderingResult result;
    const auto& cells = opt_sa_->getCells();
    
    // Convert vertex ordering to cell ordering (filter out I/O terminals)
    result.cell_ordering.clear();
    result.cell_ordering.reserve(vertex_ordering.size());
    
    for (int vertex_id : vertex_ordering) {
        // Check if this is a cell vertex (not an I/O terminal)
        auto it = vertex_to_cell_map_.find(vertex_id);
        if (it != vertex_to_cell_map_.end()) {
            int cell_id = it->second;
            if (cell_id >= 0 && cell_id < static_cast<int>(cells.size())) {
                result.cell_ordering.push_back(cell_id);
            }
        }
        // Skip I/O terminal vertices (they are fixed and don't need ordering)
    }
    
    // Initialize orientations
    result.orientations.clear();
    result.orientations.reserve(result.cell_ordering.size());
    
    for (int cell_id : result.cell_ordering) {
        if (cell_id < static_cast<int>(cells.size())) {
            const auto& cell = cells[cell_id];
            if (cell.db_inst) {
                result.orientations.push_back(cell.db_inst->getOrient());
            } else {
                result.orientations.push_back(odb::dbOrientType::R0);
            }
        }
    }
    
    result.success = true;
    return result;
}

bool VertexOrderingInterface::validateHypergraph(const Hypergraph& hg) {
    if (hg.num_vertices <= 0) {
        if (logger_) {
            logger_->error(utl::SA1D, 202, "Hypergraph has no vertices");
        }
        return false;
    }
    
    if (hg.num_hyperedges <= 0) {
        if (logger_) {
            logger_->error(utl::SA1D, 203, "Hypergraph has no hyperedges");
        }
        return false;
    }
    
    // Check basic connectivity
    if (hg.hyperedges.empty()) {
        if (logger_) {
            logger_->error(utl::SA1D, 204, "Hypergraph hyperedges vector is empty");
        }
        return false;
    }
    
    // Check for reasonable hypergraph structure
    int empty_hyperedges = 0;
    int single_vertex_hyperedges = 0;
    
    for (const auto& hyperedge : hg.hyperedges) {
        if (hyperedge.empty()) {
            empty_hyperedges++;
        } else if (hyperedge.size() == 1) {
            single_vertex_hyperedges++;
        }
    }
    
    if (empty_hyperedges > 0 && logger_) {
        logger_->warn(utl::SA1D, 108, "Hypergraph has {} empty hyperedges", empty_hyperedges);
    }
    
    if (single_vertex_hyperedges > 0 && logger_) {
        logger_->warn(utl::SA1D, 109, "Hypergraph has {} single-vertex hyperedges", single_vertex_hyperedges);
    }
    
    return true;
}

// Keep existing simple methods
VertexOrderingResult VertexOrderingInterface::computeRandomOrdering() {
    VertexOrderingResult result;
    const auto& cells = opt_sa_->getCells();
    
    if (logger_) {
        logger_->info(utl::SA1D, 120, "Computing random ordering for {} cells", cells.size());
    }
    
    if (cells.empty()) {
        result.error_message = "No cells found";
        return result;
    }
    
    // Create cell ordering: 0, 1, 2, ..., n-1
    result.cell_ordering.resize(cells.size());
    std::iota(result.cell_ordering.begin(), result.cell_ordering.end(), 0);
    
    // Shuffle randomly
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(result.cell_ordering.begin(), result.cell_ordering.end(), g);
    
    // Get orientations (preserve existing or use default)
    result.orientations.reserve(result.cell_ordering.size());
    for (int cell_id : result.cell_ordering) {
        if (cell_id < static_cast<int>(cells.size())) {
            const auto& cell = cells[cell_id];
            if (cell.db_inst) {
                result.orientations.push_back(cell.db_inst->getOrient());
            } else {
                result.orientations.push_back(odb::dbOrientType::R0);
            }
        }
    }
    
    result.success = true;
    return result;
}

VertexOrderingResult VertexOrderingInterface::computeSizeBasedOrdering() {
    VertexOrderingResult result;
    const auto& cells = opt_sa_->getCells();
    
    if (logger_) {
        logger_->info(utl::SA1D, 122, "Computing size-based ordering for {} cells", cells.size());
    }
    
    if (cells.empty()) {
        result.error_message = "No cells found";
        return result;
    }
    
    // Create cell ordering with size information
    std::vector<std::pair<int, int>> cell_size_pairs;
    cell_size_pairs.reserve(cells.size());
    
    for (size_t cell_id = 0; cell_id < cells.size(); ++cell_id) {
        const auto& cell = cells[cell_id];
        cell_size_pairs.emplace_back(static_cast<int>(cell_id), cell.getWidth());
    }
    
    // Sort by width (smallest first)
    std::sort(cell_size_pairs.begin(), cell_size_pairs.end(),
              [](const auto& a, const auto& b) {
                  return a.second < b.second;
              });
    
    // Extract ordering
    result.cell_ordering.reserve(cell_size_pairs.size());
    for (const auto& pair : cell_size_pairs) {
        result.cell_ordering.push_back(pair.first);
    }
    
    // Get orientations
    result.orientations.reserve(result.cell_ordering.size());
    for (int cell_id : result.cell_ordering) {
        const auto& cell = cells[cell_id];
        if (cell.db_inst) {
            result.orientations.push_back(cell.db_inst->getOrient());
        } else {
            result.orientations.push_back(odb::dbOrientType::R0);
        }
    }
    
    result.success = true;
    return result;
}

double VertexOrderingInterface::computeCurrentHPWL() {
    // Use SA1D's existing HPWL computation
    std::vector<int> cell_order;
    std::vector<odb::dbOrientType> orients;
    opt_sa_->cellOrdering(cell_order, orients);
    
    SAWorker worker(opt_sa_, logger_, 0);
    worker.initCellOrder(cell_order, orients);
    return static_cast<double>(worker.getTotalHPWL());
}

double VertexOrderingInterface::computeOrderingHPWL(const std::vector<int>& cell_ordering) {
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

std::string VertexOrderingInterface::orderingMethodToString(OrderingMethod method) {
    switch (method) {
        case OrderingMethod::RANDOM: return "random";
        case OrderingMethod::SIZE_BASED: return "size-based";
        case OrderingMethod::FIEDLER: return "fiedler";
        case OrderingMethod::RCM: return "rcm";
        case OrderingMethod::RCM_BOOST: return "rcm-boost";
        default: return "unknown";
    }
}

bool VertexOrderingInterface::hasIOTerminals() const {
    const auto& nets = opt_sa_->getNets();
    for (const auto& net : nets) {
        if (net.bterm_flag) {
            return true;
        }
    }
    return false;
}

int VertexOrderingInterface::computeCurrentOverlap() {
    // Get current cell ordering from OptSA
    std::vector<int> current_order;
    std::vector<odb::dbOrientType> current_orients;
    opt_sa_->cellOrdering(current_order, current_orients);
    
    return computeOrderingOverlap(current_order);
}

int VertexOrderingInterface::computeOrderingOverlap(const std::vector<int>& cell_ordering) {
    const auto& cells = opt_sa_->getCells();
    const auto& nets = opt_sa_->getNets();
    
    if (cell_ordering.empty() || nets.empty()) {
        return 0;
    }
    
    // Create orientations vector if not provided
    std::vector<odb::dbOrientType> orients;
    orients.reserve(cell_ordering.size());
    
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

} // namespace sa1d 