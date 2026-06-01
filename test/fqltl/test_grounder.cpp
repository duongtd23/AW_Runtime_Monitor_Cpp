#include <gtest/gtest.h>
#include "fqltl/fqltl.hpp"
#include <algorithm>

using namespace fqltl;

// ============================================================================
// Helpers
// ============================================================================

/// Build a RuntimeDomain matching the design-document example:
///   car1  → {path1, path2}
///   ped2  → {path1}
static RuntimeDomain makeDocDomain() {
    RuntimeDomain d;
    d.objects = {"car1", "ped2"};
    d.obj_paths["car1"] = {"path1", "path2"};
    d.obj_paths["ped2"] = {"path1"};
    return d;
}

/// Parse and ground the doc-example formula against the doc-example domain.
static GroundingResult groundDocExample() {
    const std::string formula_str =
        "[] (\n"
        "    forall obj.\n"
        "    forall path@obj.\n"
        "    (\n"
        "        existenceProb(obj) > 0.5\n"
        "        /\\ confidence(path) >= 0.1\n"
        "        /\\ time <= 3.0\n"
        "        -> ~ collision(obj, path)\n"
        "    )\n"
        ")";
    FormulaPtr f = parseFormula(formula_str);
    return groundFormula(f, makeDocDomain());
}

// ============================================================================
// Empty domain tests
// ============================================================================

TEST(GrounderTest, EmptyDomain_ForallObj_VacuouslyTrue) {
    // forall obj. collision(obj, path) with no objects → "1"
    RuntimeDomain empty;
    FormulaPtr f = parseFormula("forall obj. true");
    auto gr = groundFormula(f, empty);
    EXPECT_EQ(gr.ltl_string, "1");
    EXPECT_TRUE(gr.evaluators.empty());
}

TEST(GrounderTest, EmptyDomain_ExistsObj_VacuouslyFalse) {
    RuntimeDomain empty;
    FormulaPtr f = parseFormula("exists obj. true");
    auto gr = groundFormula(f, empty);
    EXPECT_EQ(gr.ltl_string, "0");
}

TEST(GrounderTest, EmptyPaths_ForallPath_VacuouslyTrue) {
    RuntimeDomain d;
    d.objects = {"car1"};
    d.obj_paths["car1"] = {}; // no paths
    FormulaPtr f = parseFormula("forall obj. forall path@obj. true");
    auto gr = groundFormula(f, d);
    // forall obj with car1 → forall path@car1 with no paths → "1"
    EXPECT_EQ(gr.ltl_string, "1");
}

// ============================================================================
// Single object / single path
// ============================================================================

TEST(GrounderTest, SingleObjectSinglePath_ForallForall) {
    RuntimeDomain d;
    d.objects = {"car1"};
    d.obj_paths["car1"] = {"path1"};

    FormulaPtr f = parseFormula("forall obj. forall path@obj. collision(obj, path)");
    auto gr = groundFormula(f, d);

    // Should produce a single atom (no outer conjunctions needed)
    EXPECT_EQ(gr.ltl_string, "collision__car1__path1");
    EXPECT_EQ(gr.evaluators.size(), 1u);
    ASSERT_TRUE(gr.evaluators.count("collision__car1__path1") > 0);
}

TEST(GrounderTest, SingleObjectSinglePath_ExistsExists) {
    RuntimeDomain d;
    d.objects = {"car1"};
    d.obj_paths["car1"] = {"path1"};

    FormulaPtr f = parseFormula("exists obj. exists path@obj. collision(obj, path)");
    auto gr = groundFormula(f, d);
    EXPECT_EQ(gr.ltl_string, "collision__car1__path1");
}

// ============================================================================
// Two objects, multiple paths
// ============================================================================

TEST(GrounderTest, TwoObjectsMultiplePaths_AtomCount) {
    auto gr = groundDocExample();

    // Expected atoms per object/path combination:
    //   car1/path1: existenceprob__car1, confidence__car1__path1, time, collision__car1__path1
    //   car1/path2: existenceprob__car1 (shared), confidence__car1__path2, collision__car1__path2
    //   ped2/path1: existenceprob__ped2, confidence__ped2__path1, collision__ped2__path1
    //   time: shared across all
    //
    // Unique atoms:
    //   existenceprob__car1__gt__0_5
    //   existenceprob__ped2__gt__0_5
    //   confidence__car1__path1__ge__0_1
    //   confidence__car1__path2__ge__0_1
    //   confidence__ped2__path1__ge__0_1
    //   time__le__3
    //   collision__car1__path1
    //   collision__car1__path2
    //   collision__ped2__path1
    EXPECT_EQ(gr.evaluators.size(), 9u);
}

TEST(GrounderTest, TwoObjectsMultiplePaths_AtomNames) {
    auto gr = groundDocExample();

    EXPECT_TRUE(gr.evaluators.count("existenceprob__car1__gt__0_5") > 0);
    EXPECT_TRUE(gr.evaluators.count("existenceprob__ped2__gt__0_5") > 0);
    EXPECT_TRUE(gr.evaluators.count("confidence__car1__path1__ge__0_1") > 0);
    EXPECT_TRUE(gr.evaluators.count("confidence__car1__path2__ge__0_1") > 0);
    EXPECT_TRUE(gr.evaluators.count("confidence__ped2__path1__ge__0_1") > 0);
    EXPECT_TRUE(gr.evaluators.count("time__le__3") > 0);
    EXPECT_TRUE(gr.evaluators.count("collision__car1__path1") > 0);
    EXPECT_TRUE(gr.evaluators.count("collision__car1__path2") > 0);
    EXPECT_TRUE(gr.evaluators.count("collision__ped2__path1") > 0);
}

TEST(GrounderTest, LTLStringContainsGloballyOperator) {
    auto gr = groundDocExample();
    // The outermost operator must be G(...)
    EXPECT_EQ(gr.ltl_string.substr(0, 2), "G(");
}

// ============================================================================
// Evaluator correctness
// ============================================================================

/// Build a RuntimeState where all conditions hold and no collision occurs.
static RuntimeState makeSafeState() {
    RuntimeState s;
    s.time = 1.0; // <= 3.0

    s.object_existence_prob["car1"] = 0.8; // > 0.5
    s.object_existence_prob["ped2"] = 0.7; // > 0.5

    s.path_confidence["car1__path1"] = 0.6;  // >= 0.1
    s.path_confidence["car1__path2"] = 0.4;  // >= 0.1
    s.path_confidence["ped2__path1"] = 1.0;  // >= 0.1

    s.path_collision["car1__path1"] = false;
    s.path_collision["car1__path2"] = false;
    s.path_collision["ped2__path1"] = false;
    return s;
}

TEST(GrounderTest, Evaluators_SafeState) {
    auto gr = groundDocExample();
    RuntimeState s = makeSafeState();

    EXPECT_TRUE( gr.evaluators.at("existenceprob__car1__gt__0_5")(s));
    EXPECT_TRUE( gr.evaluators.at("existenceprob__ped2__gt__0_5")(s));
    EXPECT_TRUE( gr.evaluators.at("confidence__car1__path1__ge__0_1")(s));
    EXPECT_TRUE( gr.evaluators.at("confidence__car1__path2__ge__0_1")(s));
    EXPECT_TRUE( gr.evaluators.at("confidence__ped2__path1__ge__0_1")(s));
    EXPECT_TRUE( gr.evaluators.at("time__le__3")(s));
    EXPECT_FALSE(gr.evaluators.at("collision__car1__path1")(s));
    EXPECT_FALSE(gr.evaluators.at("collision__car1__path2")(s));
    EXPECT_FALSE(gr.evaluators.at("collision__ped2__path1")(s));
}

TEST(GrounderTest, Evaluators_CollisionTrue) {
    auto gr = groundDocExample();
    RuntimeState s = makeSafeState();
    s.path_collision["car1__path1"] = true;

    EXPECT_TRUE(gr.evaluators.at("collision__car1__path1")(s));
    EXPECT_FALSE(gr.evaluators.at("collision__car1__path2")(s));
}

TEST(GrounderTest, Evaluators_TimeExceeded) {
    auto gr = groundDocExample();
    RuntimeState s = makeSafeState();
    s.time = 5.0; // > 3.0

    EXPECT_FALSE(gr.evaluators.at("time__le__3")(s));
}

TEST(GrounderTest, Evaluators_LowExistenceProb) {
    auto gr = groundDocExample();
    RuntimeState s = makeSafeState();
    s.object_existence_prob["car1"] = 0.3; // <= 0.5

    EXPECT_FALSE(gr.evaluators.at("existenceprob__car1__gt__0_5")(s));
}

TEST(GrounderTest, Evaluators_LowConfidence) {
    auto gr = groundDocExample();
    RuntimeState s = makeSafeState();
    s.path_confidence["ped2__path1"] = 0.05; // < 0.1

    EXPECT_FALSE(gr.evaluators.at("confidence__ped2__path1__ge__0_1")(s));
}

// ============================================================================
// Grounding with existential quantifiers
// ============================================================================

TEST(GrounderTest, ExistsObjFormula) {
    RuntimeDomain d;
    d.objects = {"car1", "ped2"};

    // "exists obj. existenceProb(obj) > 0.5"
    // → "(existenceprob__car1__gt__0_5 | existenceprob__ped2__gt__0_5)"
    FormulaPtr f = parseFormula("exists obj. existenceProb(obj) > 0.5");
    auto gr = groundFormula(f, d);

    EXPECT_EQ(gr.evaluators.size(), 2u);
    EXPECT_TRUE(gr.ltl_string.find(" | ") != std::string::npos);

    RuntimeState s;
    s.object_existence_prob["car1"] = 0.3;
    s.object_existence_prob["ped2"] = 0.8;

    // car1 fails but ped2 passes
    EXPECT_FALSE(gr.evaluators.at("existenceprob__car1__gt__0_5")(s));
    EXPECT_TRUE( gr.evaluators.at("existenceprob__ped2__gt__0_5")(s));
}

TEST(GrounderTest, ExistsPathFormula) {
    RuntimeDomain d;
    d.objects = {"car1"};
    d.obj_paths["car1"] = {"path1", "path2"};

    // "forall obj. exists path@obj. collision(obj, path)"
    FormulaPtr f = parseFormula("forall obj. exists path@obj. collision(obj, path)");
    auto gr = groundFormula(f, d);

    // Should contain " | " inside (exists over paths)
    EXPECT_TRUE(gr.ltl_string.find(" | ") != std::string::npos);
}

// ============================================================================
// LTL structure: G / F wrapping
// ============================================================================

TEST(GrounderTest, GloballyWrapper) {
    RuntimeDomain d;
    d.objects = {"car1"};
    d.obj_paths["car1"] = {"path1"};

    FormulaPtr f = parseFormula("[] forall obj. forall path@obj. collision(obj, path)");
    auto gr = groundFormula(f, d);
    EXPECT_EQ(gr.ltl_string, "G(collision__car1__path1)");
}

TEST(GrounderTest, EventuallyWrapper) {
    RuntimeDomain d;
    d.objects = {"car1"};

    // TimeAP needs no variable binding — a clean Eventually wrap test.
    FormulaPtr f = parseFormula("<> time <= 3.0");
    auto gr = groundFormula(f, d);
    EXPECT_EQ(gr.ltl_string, "F(time__le__3)");
}

// ============================================================================
// Unbound variable throws at grounding time
// ============================================================================

TEST(GrounderTest, UnboundObjectVariable_Throws) {
    RuntimeDomain d;
    d.objects = {"car1"};
    d.obj_paths["car1"] = {"path1"};

    // path_var is bound but obj_var "wrongName" is never bound
    FormulaPtr f = parseFormula("forall obj. forall path@wrongName. true");
    EXPECT_THROW(groundFormula(f, d), std::runtime_error);
}
