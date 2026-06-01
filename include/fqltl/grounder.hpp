#pragma once
#include "ast.hpp"
#include <functional>
#include <map>
#include <string>
#include <vector>
#include <stdexcept>

/// @brief Grounding engine: eliminates quantifiers from an fqltl formula,
///        producing a propositional LTL formula string and AP evaluators.
///
/// At each planning cycle the runtime provides a finite set of objects and
/// predicted paths.  The grounder substitutes every quantifier with a finite
/// Boolean combination:
///
///   forall x. φ(x)  →  φ(a1) & φ(a2) & ... & φ(an)
///   exists x. φ(x)  →  φ(a1) | φ(a2) | ... | φ(an)
///
/// The result is an ordinary propositional LTL formula that can be handed
/// directly to Spot (or any other LTL model checker).
///
/// ## Qualified path IDs
///
/// A path variable is bound to a *qualified* ID:
///   qualified_path_id = obj_id + "__" + local_path_id
/// Example: "car1__path1"
///
/// RuntimeState uses the same qualified IDs as map keys.
///
/// ## Output
///
/// GroundingResult contains:
///   - ltl_string   : Spot-compatible propositional LTL formula string.
///                    Operators: G F X U ! & | ->  and atoms are lowercase
///                    identifiers.
///   - evaluators   : atom_name → function(RuntimeState) → bool.
///                    The grounder registers one evaluator per unique atom.

namespace fqltl {

// ============================================================================
// RuntimeDomain — finite sets of objects and paths known at grounding time
// ============================================================================

/// Describes the finite domain available at one planning cycle.
struct RuntimeDomain {
    /// Ordered list of object IDs (e.g. {"car1", "ped2"}).
    std::vector<std::string> objects;

    /// obj_id → ordered list of local path IDs (e.g. "car1" → {"path1","path2"}).
    std::map<std::string, std::vector<std::string>> obj_paths;
};

// ============================================================================
// GroundingResult
// ============================================================================

/// Result of grounding a quantified formula against a RuntimeDomain.
struct GroundingResult {
    /// Spot-compatible propositional LTL formula string.
    /// Uses:  G  F  X  U  !  &  |  ->  1  0
    std::string ltl_string;

    /// Maps each propositional atom name to its boolean evaluator.
    /// The evaluator returns the truth value for a given RuntimeState.
    std::map<std::string, std::function<bool(const RuntimeState&)>> evaluators;
};

// ============================================================================
// Grounder
// ============================================================================

class Grounder {
public:
    /// Ground @p formula against @p domain.
    /// Returns a GroundingResult with the propositional LTL string and
    /// per-atom evaluators.
    GroundingResult ground(const FormulaPtr& formula,
                           const RuntimeDomain& domain) {
        GroundingResult result;
        VarBindings bindings;
        result.ltl_string = groundImpl(formula, bindings, domain, result);
        return result;
    }

private:
    /// Recursively ground a formula node.
    ///
    /// @param formula   Current AST node.
    /// @param bindings  Current variable bindings (passed by value so each
    ///                  quantifier branch gets an independent copy).
    /// @param domain    The runtime domain (objects / paths).
    /// @param result    Accumulator for atom evaluators (mutated in-place).
    /// @return          Spot-compatible sub-formula string.
    std::string groundImpl(const FormulaPtr& formula,
                           VarBindings      bindings,
                           const RuntimeDomain& domain,
                           GroundingResult& result) {
        using K = Formula::Kind;
        switch (formula->kind()) {

            // --- Literals ------------------------------------------------
            case K::True:  return "1";
            case K::False: return "0";

            // --- Unary ---------------------------------------------------
            case K::Not: {
                auto& n = static_cast<const NotFormula&>(*formula);
                return "!(" + groundImpl(n.child, bindings, domain, result) + ")";
            }
            case K::Globally: {
                auto& n = static_cast<const GloballyFormula&>(*formula);
                return "G(" + groundImpl(n.child, bindings, domain, result) + ")";
            }
            case K::Eventually: {
                auto& n = static_cast<const EventuallyFormula&>(*formula);
                return "F(" + groundImpl(n.child, bindings, domain, result) + ")";
            }
            case K::Next: {
                auto& n = static_cast<const NextFormula&>(*formula);
                return "X(" + groundImpl(n.child, bindings, domain, result) + ")";
            }

            // --- Binary ---------------------------------------------------
            case K::And: {
                auto& n = static_cast<const AndFormula&>(*formula);
                return "(" + groundImpl(n.left,  bindings, domain, result)
                     + " & "
                     + groundImpl(n.right, bindings, domain, result) + ")";
            }
            case K::Or: {
                auto& n = static_cast<const OrFormula&>(*formula);
                return "(" + groundImpl(n.left,  bindings, domain, result)
                     + " | "
                     + groundImpl(n.right, bindings, domain, result) + ")";
            }
            case K::Implies: {
                auto& n = static_cast<const ImpliesFormula&>(*formula);
                return "(" + groundImpl(n.left,  bindings, domain, result)
                     + " -> "
                     + groundImpl(n.right, bindings, domain, result) + ")";
            }
            case K::Until: {
                auto& n = static_cast<const UntilFormula&>(*formula);
                return "(" + groundImpl(n.left,  bindings, domain, result)
                     + " U "
                     + groundImpl(n.right, bindings, domain, result) + ")";
            }

            // --- Quantifiers ---------------------------------------------
            case K::ForallObj: {
                auto& n = static_cast<const ForallObjFormula&>(*formula);
                if (domain.objects.empty()) return "1"; // vacuously true
                return joinGroundedBodies(
                    n.var_name, n.body, domain.objects,
                    bindings, domain, result, /*is_forall=*/true);
            }
            case K::ExistsObj: {
                auto& n = static_cast<const ExistsObjFormula&>(*formula);
                if (domain.objects.empty()) return "0"; // vacuously false
                return joinGroundedBodies(
                    n.var_name, n.body, domain.objects,
                    bindings, domain, result, /*is_forall=*/false);
            }
            case K::ForallPath: {
                auto& n = static_cast<const ForallPathFormula&>(*formula);
                return groundPathQuantifier(n.path_var, n.obj_var, n.body,
                                            bindings, domain, result,
                                            /*is_forall=*/true);
            }
            case K::ExistsPath: {
                auto& n = static_cast<const ExistsPathFormula&>(*formula);
                return groundPathQuantifier(n.path_var, n.obj_var, n.body,
                                            bindings, domain, result,
                                            /*is_forall=*/false);
            }

            // --- Atomic proposition -------------------------------------
            case K::AtomicProp: {
                auto& n = static_cast<const AtomicPropFormula&>(*formula);
                std::string atom = n.ap->groundedAtomName(bindings);

                // Register evaluator (once per unique atom name).
                if (result.evaluators.find(atom) == result.evaluators.end()) {
                    VarBindings captured = bindings;
                    auto ap_copy         = n.ap; // shared_ptr copy, cheap
                    result.evaluators[atom] =
                        [ap_copy, captured](const RuntimeState& s) {
                            return ap_copy->evaluate(s, captured);
                        };
                }
                return atom;
            }

            default:
                throw std::runtime_error(
                    "fqltl::Grounder: unhandled formula kind");
        }
    }

    /// Ground a universal/existential object quantifier.
    /// For each concrete ID in @p ids, binds @p var_name to that ID and
    /// grounds the body; then joins the results with "&" or "|".
    std::string joinGroundedBodies(
        const std::string&         var_name,
        const FormulaPtr&          body,
        const std::vector<std::string>& ids,
        VarBindings                bindings,
        const RuntimeDomain&       domain,
        GroundingResult&           result,
        bool                       is_forall) {

        std::vector<std::string> parts;
        parts.reserve(ids.size());

        for (const auto& id : ids) {
            VarBindings b = bindings;
            b[var_name]   = id;
            parts.push_back(groundImpl(body, b, domain, result));
        }

        const std::string op = is_forall ? " & " : " | ";
        if (parts.size() == 1) return parts[0];

        std::string joined = "(" + parts[0];
        for (size_t i = 1; i < parts.size(); ++i)
            joined += op + parts[i];
        joined += ")";
        return joined;
    }

    /// Ground a path quantifier (forall/exists path_var @ obj_var).
    std::string groundPathQuantifier(
        const std::string& path_var,
        const std::string& obj_var,
        const FormulaPtr&  body,
        VarBindings        bindings,
        const RuntimeDomain& domain,
        GroundingResult&   result,
        bool               is_forall) {

        auto oi = bindings.find(obj_var);
        if (oi == bindings.end())
            throw std::runtime_error(
                "fqltl::Grounder: unbound object variable '" + obj_var + "'");
        const std::string& obj_id = oi->second;

        auto pi = domain.obj_paths.find(obj_id);
        if (pi == domain.obj_paths.end() || pi->second.empty())
            return is_forall ? "1" : "0"; // vacuously true/false

        // Build qualified path IDs: obj_id + "__" + local_path_id
        std::vector<std::string> qualified;
        qualified.reserve(pi->second.size());
        for (const auto& local_id : pi->second)
            qualified.push_back(obj_id + "__" + local_id);

        return joinGroundedBodies(path_var, body, qualified,
                                  bindings, domain, result, is_forall);
    }
};

/// Convenience free function: ground a formula against a domain.
inline GroundingResult groundFormula(const FormulaPtr&    formula,
                                     const RuntimeDomain& domain) {
    return Grounder().ground(formula, domain);
}

} // namespace fqltl
