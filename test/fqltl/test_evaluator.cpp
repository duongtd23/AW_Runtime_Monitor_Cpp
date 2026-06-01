#include <gtest/gtest.h>
#include "fqltl/evaluator.hpp"
#include "fqltl/data_object.hpp"
#include <glm/glm.hpp>

using namespace fqltl;

// ============================================================================
// Test Fixture: EvaluatorTest
// ============================================================================

class EvaluatorTest : public ::testing::Test {
protected:
    /**
     * Build a simple ego trajectory with n points.
     * Each point is at (x, y, z) = (t, 0, 0), heading=0, t from 0 to n-1.
     */
    PlannedTraj buildSimpleEgoTrajectory(int num_points, float time_step = 0.1) {
        PlannedTraj traj;
        for (int i = 0; i < num_points; ++i) {
            float t = i * time_step;
            PlannedTrajPoint point;
            point.position = glm::vec3(t, 0.0f, 0.0f);
            point.heading = 0.0f;
            point.long_vel = t/2.0f;
            point.lat_vel = 0.0f;
            point.acc = t/4.0f;
            point.time_from_start = t;
            traj.points().push_back(point);
        }
        return traj;
    }

    /**
     * Build a predicted path for an object starting at (start_x, start_y).
     * Points are spaced at time_step intervals.
     */
    PredictedTravelPath buildStraightPath(
        float start_x, float start_y, float start_z,
        float vx, float vy,  // velocity components
        int num_points,
        float time_step = 0.1) {
        PredictedTravelPath path;
        path.set_confidence(0.9f);
        path.set_time_step(time_step);
        for (int i = 0; i < num_points; ++i) {
            float t = i * time_step;
            PredictedTravelPathPoint pp;
            pp.position = glm::vec3(
                start_x + vx * t,
                start_y + vy * t,
                start_z
            );
            pp.heading = 0.0f;
            path.path_points().push_back(pp);
        }
        return path;
    }

    /**
     * Create a PlanningRuntimeData with given ego trajectory and objects.
     */
    PlanningRuntimeData createRuntimeData(
        const PlannedTraj& ego_traj,
        const std::vector<PerceivedObject>& objects) {
        PlanningRuntimeData data(
            glm::vec2(4.5f, 1.8f),    // ego_size (length x width)
            glm::vec2(0.0f, 0.0f)     // ego_center_offset
        );
        data.ego_planned_traj() = ego_traj;
        data.perceived_objects() = objects;
        return data;
    }

    /**
     * Create a PerceivedObject at a fixed position with a single predicted path.
     */
    PerceivedObject createObject(
        const std::string& id,
        float prob,
        const glm::vec3& position,
        const PredictedTravelPath& path) {
        PerceivedObject obj;
        obj.set_id(id);
        obj.set_probability(prob);
        obj.set_position(position);
        obj.set_heading(0.0f);
        obj.set_velocity(glm::vec3(0.0f, 0.0f, 0.0f));
        // Simple rectangular bounding box (local frame)
        obj.local_vertices().push_back(glm::vec2(-1.0f, -1.0f));
        obj.local_vertices().push_back(glm::vec2(1.0f, -1.0f));
        obj.local_vertices().push_back(glm::vec2(1.0f, 1.0f));
        obj.local_vertices().push_back(glm::vec2(-1.0f, 1.0f));
        obj.predicted_paths().push_back(path);
        return obj;
    }
};

// ============================================================================
// Test Cases
// ============================================================================

/**
 * Test 1: Empty trajectory should be safe (vacuous truth)
 */
TEST_F(EvaluatorTest, EmptyTrajectory) {
    PlannedTraj empty_traj;
    std::vector<PerceivedObject> no_objects;
    auto data = createRuntimeData(empty_traj, no_objects);

    // Any formula should be vacuously true on empty trajectory
    std::string formula = "time <= 10.0";
    bool result = FqltlEvaluator::evaluate(formula, data);
    EXPECT_TRUE(result);
}

/**
 * Test 2: No objects, simple time-based formula should be safe
 */
TEST_F(EvaluatorTest, NoObjectsTimeCheck) {
    auto ego_traj = buildSimpleEgoTrajectory(10, 0.1f);  // 10 points, 0 to 0.9s
    std::vector<PerceivedObject> no_objects;
    auto data = createRuntimeData(ego_traj, no_objects);

    // Formula: always time <= 2.0 (should be true for entire trajectory)
    std::string formula = "[] (time <= 2.0)";
    bool result = FqltlEvaluator::evaluate(formula, data);
    EXPECT_TRUE(result);
}

/**
 * Test 3: No objects, time-exceeded formula should be unsafe
 */
TEST_F(EvaluatorTest, NoObjectsTimeExceeded) {
    auto ego_traj = buildSimpleEgoTrajectory(15, 0.1f);  // 15 points, 0 to 1.4s
    std::vector<PerceivedObject> no_objects;
    auto data = createRuntimeData(ego_traj, no_objects);

    // Formula: always time <= 1.0 (should be false after t=1.0)
    std::string formula = "[] (time <= 1.0)";
    bool result = FqltlEvaluator::evaluate(formula, data);
    EXPECT_FALSE(result);
}

/**
 * Test 4: Single object, existence probability filter
 */
TEST_F(EvaluatorTest, ExistenceProbabilityFilter) {
    auto ego_traj = buildSimpleEgoTrajectory(10, 0.1f);
    
    // Object with low existence probability (0.2)
    auto path = buildStraightPath(10.0f, 10.0f, 0.0f, 0.0f, 0.0f, 10);
    auto obj = createObject("obj1", 0.2f, glm::vec3(10.0f, 10.0f, 0.0f), path);
    
    auto data = createRuntimeData(ego_traj, {obj});

    // Formula: forall obj. (existenceProb(obj) > 0.5 -> ...)
    // Should be safe because obj1's prob (0.2) is not > 0.5 (antecedent false)
    std::string formula = 
        "[] (forall obj. (existenceProb(obj) > 0.5 -> time <= 2.0))";
    bool result = FqltlEvaluator::evaluate(formula, data);
    EXPECT_TRUE(result);
}

/**
 * Test 5: Single object, existence probability constraint violated
 */
TEST_F(EvaluatorTest, ExistenceProbConstraintViolated) {
    auto ego_traj = buildSimpleEgoTrajectory(15, 0.1f);  // 0 to 1.4s
    
    // Object with high existence probability (0.8)
    auto path = buildStraightPath(10.0f, 10.0f, 0.0f, 0.0f, 0.0f, 15);
    auto obj = createObject("obj1", 0.8f, glm::vec3(10.0f, 10.0f, 0.0f), path);
    
    auto data = createRuntimeData(ego_traj, {obj});

    // Formula: always (if obj exists with prob > 0.5, then time <= 1.0)
    // Should be unsafe because time exceeds 1.0 while obj is present
    std::string formula = 
        "[] (forall obj. (existenceProb(obj) > 0.5 -> time <= 1.0))";
    bool result = FqltlEvaluator::evaluate(formula, data);
    EXPECT_FALSE(result);
}

/**
 * Test 6: Single object with confidence-based path selection
 */
TEST_F(EvaluatorTest, ConfidenceFilter) {
    auto ego_traj = buildSimpleEgoTrajectory(10, 0.1f);
    
    // Object with one low-confidence path (0.1)
    auto low_conf_path = buildStraightPath(10.0f, 10.0f, 0.0f, 0.0f, 0.0f, 10);
    low_conf_path.set_confidence(0.1f);
    
    auto obj = createObject("obj1", 0.9f, glm::vec3(10.0f, 10.0f, 0.0f), low_conf_path);
    
    auto data = createRuntimeData(ego_traj, {obj});

    // Formula: forall path, if confidence > 0.5, then constraint holds
    // Should be safe because path's confidence (0.1) is not > 0.5
    std::string formula = 
        "[] (forall obj. forall path@obj. (confidence(path) > 0.5 -> time <= 2.0))";
    bool result = FqltlEvaluator::evaluate(formula, data);
    EXPECT_TRUE(result);
}

/**
 * Test 7: Single object moving along safe path (no collision)
 */
TEST_F(EvaluatorTest, SafeNonCollidingPath) {
    auto ego_traj = buildSimpleEgoTrajectory(10, 0.1f);  // ego at x=0..0.9
    
    // Object far away, moving in parallel (no collision)
    auto path = buildStraightPath(10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10);  // at x=10
    auto obj = createObject("obj1", 0.9f, glm::vec3(10.0f, 0.0f, 0.0f), path);
    
    auto data = createRuntimeData(ego_traj, {obj});

    // Formula: forall paths, no collision
    std::string formula = 
        "[] (forall obj. forall path@obj. ~collision(obj, path))";
    bool result = FqltlEvaluator::evaluate(formula, data);
    EXPECT_TRUE(result);
}

/**
 * Test 8: Multiple objects and paths
 */
TEST_F(EvaluatorTest, MultipleObjectsAndPaths) {
    auto ego_traj = buildSimpleEgoTrajectory(10, 0.1f);
    
    // Object 1 with two paths (both safe)
    auto path1 = buildStraightPath(10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10);
    auto path2 = buildStraightPath(10.0f, 5.0f, 0.0f, 0.0f, 0.0f, 10);
    auto obj1 = createObject("obj1", 0.9f, glm::vec3(10.0f, 0.0f, 0.0f), path1);
    obj1.predicted_paths().push_back(path2);
    
    // Object 2 (single path, safe)
    auto path3 = buildStraightPath(-10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10);
    auto obj2 = createObject("obj2", 0.8f, glm::vec3(-10.0f, 0.0f, 0.0f), path3);
    
    auto data = createRuntimeData(ego_traj, {obj1, obj2});

    // Formula: all objects on all paths don't collide
    std::string formula = 
        "[] (forall obj. forall path@obj. ~collision(obj, path))";
    bool result = FqltlEvaluator::evaluate(formula, data);
    EXPECT_TRUE(result);
}

/**
 * Test 9: Combined constraints (doc example simplified)
 */
TEST_F(EvaluatorTest, CombinedConstraints) {
    auto ego_traj = buildSimpleEgoTrajectory(20, 0.1f);
    
    auto path = buildStraightPath(10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 20);
    path.set_confidence(0.8f);
    
    auto obj = createObject("obj1", 0.7f, glm::vec3(10.0f, 0.0f, 0.0f), path);
    
    auto data = createRuntimeData(ego_traj, {obj});

    // Formula: always (if obj exists AND confidence > 0.5, then no collision)
    std::string formula = 
        "[] (forall obj. forall path@obj. "
        "((existenceProb(obj) > 0.5 /\\ confidence(path) > 0.5) "
        "-> ~collision(obj, path)))";
    bool result = FqltlEvaluator::evaluate(formula, data);
    EXPECT_TRUE(result);
}

/**
 * Test 10: Eventually operator
 */
TEST_F(EvaluatorTest, EventuallyOperator) {
    auto ego_traj = buildSimpleEgoTrajectory(10, 0.1f);
    std::vector<PerceivedObject> no_objects;
    auto data = createRuntimeData(ego_traj, no_objects);

    // Formula: eventually time > 0.5 (should be true)
    std::string formula = "<> (time > 0.5)";
    bool result = FqltlEvaluator::evaluate(formula, data);
    EXPECT_TRUE(result);
}

/**
 * Test 11: Eventually operator with time constraint
 */
TEST_F(EvaluatorTest, EventuallyTimeConstraint) {
    auto ego_traj = buildSimpleEgoTrajectory(5, 0.1f);  // 0 to 0.4s
    std::vector<PerceivedObject> no_objects;
    auto data = createRuntimeData(ego_traj, no_objects);

    // Formula: eventually time > 1.0 (should be false, max time is 0.4)
    std::string formula = "<> (time > 1.0)";
    bool result = FqltlEvaluator::evaluate(formula, data);
    EXPECT_FALSE(result);
}

/**
 * Test 12: Parse error handling
 */
TEST_F(EvaluatorTest, ParseErrorHandling) {
    auto ego_traj = buildSimpleEgoTrajectory(10, 0.1f);
    std::vector<PerceivedObject> no_objects;
    auto data = createRuntimeData(ego_traj, no_objects);

    // Invalid formula (unmatched parenthesis)
    std::string formula = "[] (time <= 1.0";
    EXPECT_THROW(
        FqltlEvaluator::evaluate(formula, data),
        std::runtime_error
    );
}

/**
 * Test 13: Unbound variable in quantifier scope
 */
TEST_F(EvaluatorTest, UnboundVariableInScope) {
    auto ego_traj = buildSimpleEgoTrajectory(10, 0.1f);
    auto path = buildStraightPath(10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10);
    auto obj = createObject("obj1", 0.9f, glm::vec3(10.0f, 0.0f, 0.0f), path);
    auto data = createRuntimeData(ego_traj, {obj});

    // Valid: obj is bound by forall quantifier
    std::string formula = 
        "[] (forall obj. existenceProb(obj) > 0.5)";
    // Should not throw; should evaluate correctly
    EXPECT_NO_THROW(
        FqltlEvaluator::evaluate(formula, data)
    );
}

/**
 * Test 14: Conjunction of constraints
 */
TEST_F(EvaluatorTest, ConjunctionOfConstraints) {
    auto ego_traj = buildSimpleEgoTrajectory(10, 0.1f);
    auto path = buildStraightPath(10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10);
    auto obj = createObject("obj1", 0.9f, glm::vec3(10.0f, 0.0f, 0.0f), path);
    auto data = createRuntimeData(ego_traj, {obj});

    // Formula: (no collision) AND (time < 2.0)
    std::string formula = 
        "[] ((forall obj. forall path@obj. ~collision(obj, path)) /\\ (time < 2.0))";
    bool result = FqltlEvaluator::evaluate(formula, data);
    EXPECT_TRUE(result);
}

/**
 * Test 15: Disjunction of constraints
 */
TEST_F(EvaluatorTest, DisjunctionOfConstraints) {
    auto ego_traj = buildSimpleEgoTrajectory(10, 0.1f);
    std::vector<PerceivedObject> no_objects;
    auto data = createRuntimeData(ego_traj, no_objects);

    // Formula: (time > 1.0) OR (time < 0.5) - should be true for some points
    std::string formula = 
        "[] ((time > 1.0) \\/ (time < 0.5))";
    // This is always globally true because every point satisfies one or the other
    // Actually, for middle points (0.5 <= t <= 1.0), neither is true
    // So this should be false
    bool result = FqltlEvaluator::evaluate(formula, data);
    EXPECT_FALSE(result);
}

/**
 * Test 16: Implication operator
 */
TEST_F(EvaluatorTest, ImplicationOperator) {
    auto ego_traj = buildSimpleEgoTrajectory(10, 0.1f);
    std::vector<PerceivedObject> no_objects;
    auto data = createRuntimeData(ego_traj, no_objects);

    // Formula: (time < 0.5) -> (time < 1.0) - vacuously true when antecedent false
    std::string formula = 
        "[] ((time < 0.5) -> (time < 1.0))";
    bool result = FqltlEvaluator::evaluate(formula, data);
    EXPECT_TRUE(result);
}

/**
 * Test 17: Negation
 */
TEST_F(EvaluatorTest, NegationOperator) {
    auto ego_traj = buildSimpleEgoTrajectory(10, 0.1f);
    std::vector<PerceivedObject> no_objects;
    auto data = createRuntimeData(ego_traj, no_objects);

    // Formula: NOT (time > 2.0) = (time <= 2.0)
    std::string formula = 
        "[] ~(time > 2.0)";
    bool result = FqltlEvaluator::evaluate(formula, data);
    EXPECT_TRUE(result);
}

/**
 * Test 18: Single object moving opposite
 */
TEST_F(EvaluatorTest, UnsafeCollidingPath) {
    auto ego_traj = buildSimpleEgoTrajectory(10, 0.1f);  // ego at x=0..0.9
    
    // Object far away, moving in parallel (no collision)
    auto path = buildStraightPath(7.0f, 0.0f, 0.0f, -1.0f, 0.0f, 10);
    auto obj = createObject("obj1", 0.9f, glm::vec3(7.0f, 0.0f, 0.0f), path);
    
    auto data = createRuntimeData(ego_traj, {obj});

    // Formula: forall paths, no collision
    std::string formula = 
        "[] (forall obj. forall path@obj. ~collision(obj, path))";
    bool result = FqltlEvaluator::evaluate(formula, data);
    EXPECT_TRUE(result);
}

/**
 * Test 19: Multiple formulas on the same data
 */
TEST_F(EvaluatorTest, MultipleFormulas) {
    auto ego_traj = buildSimpleEgoTrajectory(10, 1);
    auto path = buildStraightPath(10.0f, 0.0f, 0.0f,  -1.0f, 0.0f,  10,  1);
    auto obj = createObject("obj1", 0.9f, glm::vec3(10.0f, 0.0f, 0.0f), path);

    auto data = createRuntimeData(ego_traj, {obj});

    std::string formula1 = "[] (forall obj. forall path@obj. ~collision(obj, path))";
    bool result1 = FqltlEvaluator::evaluate(formula1, data);
    EXPECT_FALSE(result1);

    std::string formula2 = "[] (forall obj. forall path@obj. (time <= 3.0 -> ~collision(obj, path)) )";
    bool result2 = FqltlEvaluator::evaluate(formula2, data);
    EXPECT_TRUE(result2);

    std::string formula3 = "[] (forall obj. forall path@obj. (time <= 3.0 -> speed < 2.0) )";
    bool result3 = FqltlEvaluator::evaluate(formula3, data);
    EXPECT_TRUE(result3);

    std::string formula4 = "[] (forall obj. forall path@obj. (time <= 5.0 -> speed < 2.0) )";
    bool result4 = FqltlEvaluator::evaluate(formula4, data);
    EXPECT_FALSE(result4);

    std::string formula5 = "[] (forall obj. forall path@obj. (time <= 4.0 -> acceleration < 2.0) )";
    bool result5 = FqltlEvaluator::evaluate(formula5, data);
    EXPECT_TRUE(result5);

    std::string formula6 = "[] (forall obj. forall path@obj. (time <= 8.0 -> acceleration < 2.0) )";
    bool result6 = FqltlEvaluator::evaluate(formula6, data);
    EXPECT_FALSE(result6);

    std::string formula7 = "[] (forall obj. forall path@obj. (time <= 3.0 -> distance(obj, path) > 3.5) )";
    bool result7 = FqltlEvaluator::evaluate(formula7, data);
    EXPECT_TRUE(result7);

    std::string formula8 = "[] (forall obj. forall path@obj. (time <= 4.0 -> distance(obj, path) > 3.0) )";
    bool result8 = FqltlEvaluator::evaluate(formula8, data);
    EXPECT_FALSE(result8);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
