#pragma once
#include "fqltl.hpp"
#include "data_object.hpp"
#include <glm/glm.hpp>
#include <spot/tl/parse.hh>
#include <spot/kripke/kripkegraph.hh>
#include <spot/twaalgos/translate.hh>
#include <spot/twaalgos/emptiness.hh>
#include <vector>
#include <map>
#include <string>
#include <functional>
#include "utils.hpp"
#include <stdexcept>
#include <cmath>
#include <iostream>

namespace fqltl {

// ============================================================================
// Helper: Linear interpolation
// ============================================================================

inline glm::vec3 lerp(const glm::vec3& a, const glm::vec3& b, float t) {
    return a + t * (b - a);
}

inline float lerpAngle(float a, float b, float t) {
    return a + t * (b - a);
}

// ============================================================================
// FqltlEvaluator
// ============================================================================

/**
 * @brief Evaluates a quantified LTL formula against a single planning cycle.
 *
 * Process:
 * 1. Parse the formula into AST
 * 2. Extract runtime domain (objects and paths) from PlanningRuntimeData
 * 3. Build a sequence of RuntimeState for each trajectory point (interpolating object positions)
 * 4. Ground the formula using the grounder
 * 5. Build a Kripke structure from the grounded formula and state sequence
 * 6. Run Spot model checking
 */
class FqltlEvaluator {
public:
    /**
     * @brief Evaluate a quantified LTL formula against runtime data from a single planning cycle.
     *
     * @param formula_str The quantified LTL formula (e.g.,
     *        "[] (forall obj. forall path@obj. (...))").
     * @param runtime_data The runtime data containing:
     *        - ego planned trajectory
     *        - detected objects with predicted paths
     *        - ego size and center offset
     * @return true if the formula is satisfied (spec safe), false otherwise,
     *  and fills collision_indices with the indices of trajectory points that violate the spec (if any).
     *
     * @throws std::runtime_error if formula parsing or grounding fails
     */
    static bool evaluate(const std::string& formula_str,
                         const PlanningRuntimeData& runtime_data,
                         std::set<size_t>& collision_indices) {
        try {
            // Parse the formula into AST
            FormulaPtr formula = parseFormula(formula_str);

            // Extract runtime domain from perceived objects and paths
            RuntimeDomain domain = extractRuntimeDomain(runtime_data);

            // Build state sequence for the entire planned trajectory
            std::vector<RuntimeState> states = buildTrajectoryStates(runtime_data);

            if (states.empty()) {
                // No trajectory points to verify
                return true;
            }
            
            // Ground the formula against the domain
            Grounder grounder;
            GroundingResult grounding = grounder.ground(formula, domain);

            // Get the propositional LTL string and evaluators
            const std::string& ltl_string = grounding.ltl_string;
            const auto& evaluators = grounding.evaluators;

            // Parse the grounded LTL formula using Spot
            spot::parsed_formula pf = spot::parse_infix_psl(ltl_string);
            if (pf.format_errors(std::cerr)) {
                throw std::runtime_error(
                    "Failed to parse grounded LTL formula: " + ltl_string);
            }

            // Create a Kripke graph for model checking
            auto bdd_dict = spot::make_bdd_dict();
            auto kripke_graph = spot::make_kripke_graph(bdd_dict);

            // Register all atomic propositions in the Kripke graph
            std::map<std::string, unsigned> ap_ids;
            for (const auto& [atom_name, _] : evaluators) {
                ap_ids[atom_name] = kripke_graph->register_ap(atom_name);
            }

            // Build Kripke graph states (one per trajectory point)
            std::vector<unsigned> state_ids;
            state_ids.reserve(states.size());            

            for (size_t i = 0; i < states.size(); ++i) {
                const RuntimeState& state = states[i];

                // Construct the BDD label for this state
                bdd label = bddtrue;

                for (const auto& [atom_name, evaluator] : evaluators) {
                    bool value = evaluator(state);
                    unsigned ap_id = ap_ids[atom_name];
                    if (value) {
                        label &= bdd_ithvar(ap_id);
                    } else {
                        label &= bdd_nithvar(ap_id);
                    }

                    // check if the AP is a collision predicate and if it holds, record the index
                    if (startsWith(atom_name, "collision") && value) {
                        collision_indices.insert(i);
                    }
                }

                unsigned state_id = kripke_graph->new_state(label);
                state_ids.push_back(state_id);

                // Add transitions: i -> i+1 (linear chain)
                if (i > 0) {
                    kripke_graph->new_edge(state_ids[i - 1], state_id);
                }
            }

            // Add self-loop at the last state (to satisfy liveness properties)
            if (!state_ids.empty()) {
                kripke_graph->new_edge(state_ids.back(), state_ids.back());
            }

            // Mark the first state as initial
            if (!state_ids.empty()) {
                kripke_graph->set_init_state(state_ids.front());
            }

            // Translate the negation of the formula (to check for emptiness)
            spot::formula negated_formula = spot::formula::Not(pf.f);
            auto translator = spot::translator(bdd_dict);
            auto automaton = translator.run(negated_formula);

            // Check for intersecting run between Kripke graph and automaton
            // If there IS an intersecting run, the negated formula is satisfied -> original formula is FALSE
            return !kripke_graph->intersecting_run(automaton);
        } catch (const std::exception& e) {
            throw std::runtime_error(
                std::string("FqltlEvaluator::evaluate failed: ") + e.what());
        }
    }

    static bool evaluate(const std::string& formula_str,
                         const PlanningRuntimeData& runtime_data) {
        std::set<size_t> collision_indices; // not used in this implementation, but can be tracked within the evaluator if needed
        return evaluate(formula_str, runtime_data, collision_indices);
    }

private:
    /**
     * @brief Extract the RuntimeDomain from PlanningRuntimeData.
     *
     * Objects are extracted from perceived_objects, indexed by their id.
     * For each object, paths are indexed as {0, 1, 2, ...}.
     */
    static RuntimeDomain extractRuntimeDomain(
            const PlanningRuntimeData& runtime_data) {
        RuntimeDomain domain;

        // Extract object IDs
        for (const auto& obj : runtime_data.perceived_objects()) {
            domain.objects.push_back(obj.id());

            // Extract path indices for this object
            std::vector<std::string> path_ids;
            for (size_t i = 0; i < obj.predicted_paths().size(); ++i) {
                path_ids.push_back("path" + std::to_string(i));
            }
            domain.obj_paths[obj.id()] = path_ids;
        }

        return domain;
    }

    /**
     * @brief Build a sequence of RuntimeState, one per trajectory point.
     *
     * For each planned trajectory point:
     * - time = trajectory_point.time_from_start
     * - For each object and its predicted path:
     *   - Interpolate object position at this time
     *   - Compute collision with ego trajectory point
     *   - Store in RuntimeState maps using qualified path IDs
     */
    static std::vector<RuntimeState> buildTrajectoryStates(
            const PlanningRuntimeData& runtime_data) {
        std::vector<RuntimeState> states;

        const auto& trajectory = runtime_data.ego_planned_traj();
        const auto& traj_points = trajectory.points();
        const auto& objects = runtime_data.perceived_objects();
        const glm::vec2& ego_size = runtime_data.ego_size;
        const glm::vec2& ego_center_offset = runtime_data.ego_center_offset;

        if (traj_points.empty())
            return states; // Empty trajectory

        // For each trajectory point, build a RuntimeState
        for (size_t i = 0; i < traj_points.size(); ++i) {
            const auto& ego_point = traj_points[i];
            RuntimeState state;

            // Set time
            state.time = ego_point.time_from_start;
            // Set ego speed and acceleration
            state.ego_speed = sqrt(ego_point.long_vel * ego_point.long_vel + ego_point.lat_vel * ego_point.lat_vel);
            state.ego_acceleration = ego_point.acc;

            // Set existence probabilities for all objects
            for (const auto& obj : objects) {
                state.object_existence_prob[obj.id()] = obj.probability();
            }

            // For each object and each of its predicted paths
            for (const auto& obj : objects) {
                const std::string& obj_id = obj.id();
                const auto& paths = obj.predicted_paths();

                for (size_t path_idx = 0; path_idx < paths.size(); ++path_idx) {
                    const auto& path = paths[path_idx];
                    std::string qualified_path_id = obj_id + "__path" + std::to_string(path_idx);

                    // Store path confidence
                    state.path_confidence[qualified_path_id] = path.confidence();

                    // Interpolate object position at this time
                    glm::vec3 obj_pos_3d;
                    float obj_heading;
                    bool interp_ok = interpolatePathPosition(
                        path, ego_point.time_from_start, obj_pos_3d, obj_heading);

                    if (!interp_ok) {
                        // Time is beyond path points; mark as no collision
                        state.path_collision[qualified_path_id] = false;
                        state.path_distance[qualified_path_id] = glm::distance(ego_point.position, obj.position());
                        continue;
                    }
                    
                    // Compute distance to ego
                    state.path_distance[qualified_path_id] = glm::distance(ego_point.position, obj_pos_3d);

                    // Check collision
                    bool collision = checkCollision(
                        glm::vec2(ego_point.position.x, ego_point.position.y),
                        ego_point.heading,
                        ego_size,
                        ego_center_offset,
                        glm::vec2(obj_pos_3d.x, obj_pos_3d.y),
                        obj_heading,
                        obj.local_vertices(),
                        ego_point.position.z,
                        obj_pos_3d.z);

                    state.path_collision[qualified_path_id] = collision;
                }
            }

            states.push_back(state);
        }

        return states;
    }

    /**
     * @brief Interpolate object position on a predicted path at time t.
     *
     * @param path The predicted travel path
     * @param time The time value (seconds from start)
     * @param[out] position Interpolated 3D position
     * @param[out] heading Interpolated heading (yaw in degrees)
     * @return true if interpolation succeeded, false if time is beyond path
     */
    static bool interpolatePathPosition(
            const PredictedTravelPath& path,
            float time,
            glm::vec3& position,
            float& heading) {
        const auto& path_points = path.path_points();
        float time_step = path.time_step();

        if (path_points.empty()) {
            return false;
        }

        // Compute which segment of the path corresponds to time t
        size_t segment_index = static_cast<size_t>(time / time_step);
        float remainder_time = time - segment_index * time_step;

        // Check if we have the next point
        if (segment_index >= path_points.size() - 1) {
            // Time is beyond the path
            if (path_points.size() == 1) {
                // Only one point; use it
                position = path_points.back().position;
                heading = path_points.back().heading;
                return true;
            }
            // Multiple points but time is beyond; fail
            return false;
        }

        // Interpolate between two consecutive points
        const auto& pt0 = path_points[segment_index];
        const auto& pt1 = path_points[segment_index + 1];

        float interp_factor = remainder_time / time_step;
        if (interp_factor > 1.0f) interp_factor = 1.0f;

        position = lerp(pt0.position, pt1.position, interp_factor);
        heading = lerpAngle(pt0.heading, pt1.heading, interp_factor);

        return true;

    }

    /**
     * @brief Check if ego trajectory point collides with object's predicted path point.
     *
     * @param ego_pos Ego position at this time step
     * @param ego_heading Ego heading (yaw in degrees)
     * @param ego_size Ego bounding box size (length x width)
     * @param ego_center_offset Ego center offset relative to position
     * @param obj_pos Object position at this time step
     * @param obj_heading Object heading (yaw in degrees)
     * @param obj_local_vertices Object bounding box vertices (in local frame)
     * @param ego_z Ego z-coordinate
     * @param obj_z Object z-coordinate
     * @return true if collision or same-level check fails, false otherwise
     */
    static bool checkCollision(
            const glm::vec2& ego_pos, float ego_heading,
            const glm::vec2& ego_size, const glm::vec2& ego_center_offset,
            const glm::vec2& obj_pos, float obj_heading,
            const std::vector<glm::vec2>& obj_local_vertices,
            float ego_z, float obj_z) {
        // Check z-coordinate difference (only consider collision if on same level)
        // Using a threshold of 3.0 meters (same as in planning_shield.cpp)
        bool same_z_level = std::abs(ego_z - obj_z) < 3.0f;
        
        if (!same_z_level) {
            return false; // No collision if at different z-levels
        }

        // Get ego and object vertices in world frame
        auto ego_vertices = getEgoWorldVertices(
            ego_pos, ego_heading, ego_size, ego_center_offset);
        auto obj_vertices = getObjectWorldVertices(
            obj_pos, obj_heading, obj_local_vertices);

        // Check collision using existing collision detection function
        return isCollision(ego_vertices, obj_vertices);
    }
};

} // namespace fqltl
