#pragma once
#include "ap.hpp"
#include <memory>
#include <string>

/// @brief Abstract Syntax Tree (AST) node classes for fqltl formulas.
///
/// The AST mirrors the formula grammar:
///
///   formula  ::= implies
///   implies  ::= or  ( '->'  or  )*            (right-associative)
///   or       ::= and ( '\\/' and )*
///   and      ::= until ( '/\\' until )*
///   until    ::= unary ( 'U' unary )?           (right-associative)
///   unary    ::= '~' unary
///              | '[]' unary
///              | '<>' unary
///              | 'X'  unary
///              | quantifier
///              | primary
///   quant    ::= ('forall'|'exists') IDENT '.' formula          -- objects
///              | ('forall'|'exists') IDENT '@' IDENT '.' formula -- paths
///   primary  ::= '(' formula ')' | 'true' | 'false' | atomic_prop
///
/// Operator precedence (lowest to highest): -> \/ /\ U ~ [] <> X

namespace fqltl {

class Formula;
using FormulaPtr = std::shared_ptr<Formula>;

// ============================================================================
// Formula — abstract base
// ============================================================================

class Formula {
public:
    enum class Kind {
        True, False,
        Not, And, Or, Implies,
        Globally, Eventually, Next, Until,
        ForallObj, ForallPath,
        ExistsObj, ExistsPath,
        AtomicProp
    };

    virtual ~Formula() = default;
    virtual Kind        kind()     const = 0;
    virtual std::string toString() const = 0;
};

// ============================================================================
// Leaf nodes
// ============================================================================

class TrueFormula : public Formula {
public:
    Kind        kind()     const override { return Kind::True; }
    std::string toString() const override { return "true"; }
};

class FalseFormula : public Formula {
public:
    Kind        kind()     const override { return Kind::False; }
    std::string toString() const override { return "false"; }
};

// ============================================================================
// Unary nodes
// ============================================================================

class UnaryFormula : public Formula {
public:
    FormulaPtr child;
    explicit UnaryFormula(FormulaPtr c) : child(std::move(c)) {}
};

class NotFormula : public UnaryFormula {
public:
    using UnaryFormula::UnaryFormula;
    Kind        kind()     const override { return Kind::Not; }
    std::string toString() const override { return "~" + child->toString(); }
};

class GloballyFormula : public UnaryFormula {
public:
    using UnaryFormula::UnaryFormula;
    Kind        kind()     const override { return Kind::Globally; }
    std::string toString() const override {
        return "[](" + child->toString() + ")";
    }
};

class EventuallyFormula : public UnaryFormula {
public:
    using UnaryFormula::UnaryFormula;
    Kind        kind()     const override { return Kind::Eventually; }
    std::string toString() const override {
        return "<>(" + child->toString() + ")";
    }
};

class NextFormula : public UnaryFormula {
public:
    using UnaryFormula::UnaryFormula;
    Kind        kind()     const override { return Kind::Next; }
    std::string toString() const override {
        return "X(" + child->toString() + ")";
    }
};

// ============================================================================
// Binary nodes
// ============================================================================

class BinaryFormula : public Formula {
public:
    FormulaPtr left;
    FormulaPtr right;
    BinaryFormula(FormulaPtr l, FormulaPtr r)
        : left(std::move(l)), right(std::move(r)) {}
};

class AndFormula : public BinaryFormula {
public:
    using BinaryFormula::BinaryFormula;
    Kind        kind()     const override { return Kind::And; }
    std::string toString() const override {
        return "(" + left->toString() + " /\\ " + right->toString() + ")";
    }
};

class OrFormula : public BinaryFormula {
public:
    using BinaryFormula::BinaryFormula;
    Kind        kind()     const override { return Kind::Or; }
    std::string toString() const override {
        return "(" + left->toString() + " \\/ " + right->toString() + ")";
    }
};

class ImpliesFormula : public BinaryFormula {
public:
    using BinaryFormula::BinaryFormula;
    Kind        kind()     const override { return Kind::Implies; }
    std::string toString() const override {
        return "(" + left->toString() + " -> " + right->toString() + ")";
    }
};

class UntilFormula : public BinaryFormula {
public:
    using BinaryFormula::BinaryFormula;
    Kind        kind()     const override { return Kind::Until; }
    std::string toString() const override {
        return "(" + left->toString() + " U " + right->toString() + ")";
    }
};

// ============================================================================
// Quantifier nodes
// ============================================================================

/// forall var_name . body  — universally quantifies over all objects.
class ForallObjFormula : public Formula {
public:
    std::string var_name; ///< bound variable name
    FormulaPtr  body;

    ForallObjFormula(std::string var, FormulaPtr b)
        : var_name(std::move(var)), body(std::move(b)) {}

    Kind        kind()     const override { return Kind::ForallObj; }
    std::string toString() const override {
        return "forall " + var_name + ". " + body->toString();
    }
};

/// forall path_var @ obj_var . body  — universally quantifies over paths of
/// an already-bound object variable.
class ForallPathFormula : public Formula {
public:
    std::string path_var; ///< new bound variable for paths
    std::string obj_var;  ///< previously-bound object variable
    FormulaPtr  body;

    ForallPathFormula(std::string pvar, std::string ovar, FormulaPtr b)
        : path_var(std::move(pvar)), obj_var(std::move(ovar)),
          body(std::move(b)) {}

    Kind        kind()     const override { return Kind::ForallPath; }
    std::string toString() const override {
        return "forall " + path_var + "@" + obj_var + ". " + body->toString();
    }
};

/// exists var_name . body — existentially quantifies over objects.
class ExistsObjFormula : public Formula {
public:
    std::string var_name;
    FormulaPtr  body;

    ExistsObjFormula(std::string var, FormulaPtr b)
        : var_name(std::move(var)), body(std::move(b)) {}

    Kind        kind()     const override { return Kind::ExistsObj; }
    std::string toString() const override {
        return "exists " + var_name + ". " + body->toString();
    }
};

/// exists path_var @ obj_var . body — existentially quantifies over paths.
class ExistsPathFormula : public Formula {
public:
    std::string path_var;
    std::string obj_var;
    FormulaPtr  body;

    ExistsPathFormula(std::string pvar, std::string ovar, FormulaPtr b)
        : path_var(std::move(pvar)), obj_var(std::move(ovar)),
          body(std::move(b)) {}

    Kind        kind()     const override { return Kind::ExistsPath; }
    std::string toString() const override {
        return "exists " + path_var + "@" + obj_var + ". " + body->toString();
    }
};

// ============================================================================
// AtomicPropFormula — leaf wrapping an AP object
// ============================================================================

/// Wraps an AP object as a formula node.
class AtomicPropFormula : public Formula {
public:
    std::shared_ptr<AP> ap;

    explicit AtomicPropFormula(std::shared_ptr<AP> a) : ap(std::move(a)) {}

    Kind        kind()     const override { return Kind::AtomicProp; }
    std::string toString() const override { return ap->toString(); }
};

// ============================================================================
// Factory helpers
// ============================================================================

inline FormulaPtr makeTrue()    { return std::make_shared<TrueFormula>();  }
inline FormulaPtr makeFalse()   { return std::make_shared<FalseFormula>(); }

inline FormulaPtr makeNot(FormulaPtr c) {
    return std::make_shared<NotFormula>(std::move(c));
}
inline FormulaPtr makeGlobally(FormulaPtr c) {
    return std::make_shared<GloballyFormula>(std::move(c));
}
inline FormulaPtr makeEventually(FormulaPtr c) {
    return std::make_shared<EventuallyFormula>(std::move(c));
}
inline FormulaPtr makeNext(FormulaPtr c) {
    return std::make_shared<NextFormula>(std::move(c));
}
inline FormulaPtr makeAnd(FormulaPtr l, FormulaPtr r) {
    return std::make_shared<AndFormula>(std::move(l), std::move(r));
}
inline FormulaPtr makeOr(FormulaPtr l, FormulaPtr r) {
    return std::make_shared<OrFormula>(std::move(l), std::move(r));
}
inline FormulaPtr makeImplies(FormulaPtr l, FormulaPtr r) {
    return std::make_shared<ImpliesFormula>(std::move(l), std::move(r));
}
inline FormulaPtr makeUntil(FormulaPtr l, FormulaPtr r) {
    return std::make_shared<UntilFormula>(std::move(l), std::move(r));
}
inline FormulaPtr makeForallObj(std::string var, FormulaPtr body) {
    return std::make_shared<ForallObjFormula>(std::move(var), std::move(body));
}
inline FormulaPtr makeForallPath(std::string pv, std::string ov,
                                  FormulaPtr body) {
    return std::make_shared<ForallPathFormula>(
        std::move(pv), std::move(ov), std::move(body));
}
inline FormulaPtr makeExistsObj(std::string var, FormulaPtr body) {
    return std::make_shared<ExistsObjFormula>(std::move(var), std::move(body));
}
inline FormulaPtr makeExistsPath(std::string pv, std::string ov,
                                  FormulaPtr body) {
    return std::make_shared<ExistsPathFormula>(
        std::move(pv), std::move(ov), std::move(body));
}
inline FormulaPtr makeAtomicProp(std::shared_ptr<AP> ap) {
    return std::make_shared<AtomicPropFormula>(std::move(ap));
}

} // namespace fqltl
