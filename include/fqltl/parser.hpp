#pragma once
#include "lexer.hpp"
#include "ast.hpp"
#include <stdexcept>
#include <string>
#include <memory>

/// @brief Recursive-descent parser for fqltl formula strings.
///
/// Grammar (operator precedence, lowest → highest):
///
///   formula  ::= implies
///   implies  ::= or ( '->'  or  )*              right-associative
///   or       ::= and ( '\\/' and )*
///   and      ::= until ( '/\\' until )*
///   until    ::= unary ( 'U'  unary )?           right-associative
///   unary    ::= '~'  unary
///              | '[]' unary
///              | '<>' unary
///              | 'X'  unary
///              | quantifier
///              | primary
///   quant    ::= ('forall'|'exists') IDENT '.' formula
///              | ('forall'|'exists') IDENT '@' IDENT '.' formula
///   primary  ::= '(' formula ')' | 'true' | 'false' | atomic_prop
///
///   atomic_prop ::=
///       'collision'    '(' IDENT ',' IDENT ')'
///     | 'existenceProb' '(' IDENT ')' comp_op NUMBER
///     | 'confidence'   '(' IDENT ')' comp_op NUMBER
///     | 'time'                        comp_op NUMBER
///
///   comp_op  ::= '>' | '>=' | '<' | '<=' | '==' | '='
///
/// Usage:
///   fqltl::FormulaPtr f = fqltl::Parser("[] (forall obj. ...)").parse();

namespace fqltl {

// ============================================================================
// ParseError
// ============================================================================

/// Exception thrown when the parser encounters a syntax error.
class ParseError : public std::runtime_error {
public:
    explicit ParseError(const std::string& msg)
        : std::runtime_error("fqltl::ParseError: " + msg) {}
};

// ============================================================================
// Parser
// ============================================================================

class Parser {
public:
    /// Construct a parser for the given formula string.
    explicit Parser(const std::string& input)
        : lexer_(input) {
        advance(); // prime the first token
    }

    /// Parse the full formula and return its AST root.
    /// Throws ParseError on any syntax error.
    FormulaPtr parse() {
        FormulaPtr f = parseFormula();
        if (current_.type != TokenType::END)
            throw ParseError("unexpected token '" + current_.text +
                             "' after formula");
        return f;
    }

private:
    Lexer lexer_;
    Token current_;

    // --- Token stream helpers ----------------------------------------------

    void advance() { current_ = lexer_.nextToken(); }

    void expect(TokenType t) {
        if (current_.type != t)
            throw ParseError(
                std::string("expected '") + tokenTypeName(t) +
                "', got '" + current_.text + "'");
        advance();
    }

    // --- Grammar rules -----------------------------------------------------

    // formula ::= implies
    FormulaPtr parseFormula() { return parseImplies(); }

    // implies ::= or ( '->' or )*   [right-associative]
    FormulaPtr parseImplies() {
        FormulaPtr left = parseOr();
        if (current_.type == TokenType::IMPLIES) {
            advance();
            FormulaPtr right = parseImplies(); // right-recursive
            return makeImplies(std::move(left), std::move(right));
        }
        return left;
    }

    // or ::= and ( '\\/' and )*
    FormulaPtr parseOr() {
        FormulaPtr left = parseAnd();
        while (current_.type == TokenType::OR) {
            advance();
            left = makeOr(std::move(left), parseAnd());
        }
        return left;
    }

    // and ::= until ( '/\\' until )*
    FormulaPtr parseAnd() {
        FormulaPtr left = parseUntil();
        while (current_.type == TokenType::AND) {
            advance();
            left = makeAnd(std::move(left), parseUntil());
        }
        return left;
    }

    // until ::= unary ( 'U' unary )?   [right-associative]
    FormulaPtr parseUntil() {
        FormulaPtr left = parseUnary();
        if (current_.type == TokenType::UNTIL) {
            advance();
            FormulaPtr right = parseUnary();
            return makeUntil(std::move(left), std::move(right));
        }
        return left;
    }

    // unary ::= '~' unary | '[]' unary | '<>' unary | 'X' unary
    //         | quantifier | primary
    FormulaPtr parseUnary() {
        switch (current_.type) {
            case TokenType::NOT: {
                advance();
                return makeNot(parseUnary());
            }
            case TokenType::GLOBALLY: {
                advance();
                return makeGlobally(parseUnary());
            }
            case TokenType::EVENTUALLY: {
                advance();
                return makeEventually(parseUnary());
            }
            case TokenType::NEXT: {
                advance();
                return makeNext(parseUnary());
            }
            case TokenType::FORALL: {
                advance();
                return parseQuantifier(/*is_forall=*/true);
            }
            case TokenType::EXISTS: {
                advance();
                return parseQuantifier(/*is_forall=*/false);
            }
            default:
                return parsePrimary();
        }
    }

    /// Parse the rest of a quantifier after the 'forall'/'exists' keyword has
    /// been consumed.
    ///
    ///   IDENT '.' formula          → object quantifier
    ///   IDENT '@' IDENT '.' formula → path quantifier
    FormulaPtr parseQuantifier(bool is_forall) {
        if (current_.type != TokenType::IDENT)
            throw ParseError(
                std::string("expected variable name after '") +
                (is_forall ? "forall" : "exists") + "'");

        std::string first_var = current_.text;
        advance();

        if (current_.type == TokenType::AT) {
            // path quantifier:  path_var @ obj_var . body
            advance();
            if (current_.type != TokenType::IDENT)
                throw ParseError("expected object variable name after '@'");
            std::string obj_var = current_.text;
            advance();
            expect(TokenType::DOT);
            FormulaPtr body = parseFormula(); // extends as far right as possible
            return is_forall
                ? makeForallPath(first_var, obj_var, std::move(body))
                : makeExistsPath(first_var, obj_var, std::move(body));
        } else {
            // object quantifier:  var . body
            expect(TokenType::DOT);
            FormulaPtr body = parseFormula();
            return is_forall
                ? makeForallObj(first_var, std::move(body))
                : makeExistsObj(first_var, std::move(body));
        }
    }

    // primary ::= '(' formula ')' | 'true' | 'false' | atomic_prop
    FormulaPtr parsePrimary() {
        if (current_.type == TokenType::LPAREN) {
            advance();
            FormulaPtr f = parseFormula();
            expect(TokenType::RPAREN);
            return f;
        }
        if (current_.type == TokenType::TRUE) {
            advance();
            return makeTrue();
        }
        if (current_.type == TokenType::FALSE) {
            advance();
            return makeFalse();
        }
        return parseAtomicProp();
    }

    // atomic_prop ::= 'collision' '(' IDENT ',' IDENT ')'
    //               | 'existenceProb' '(' IDENT ')' comp_op NUMBER
    //               | 'confidence'   '(' IDENT ')' comp_op NUMBER
    //               | 'time' comp_op NUMBER
    FormulaPtr parseAtomicProp() {
        if (current_.type != TokenType::IDENT)
            throw ParseError(
                "expected atomic proposition name, got '" +
                current_.text + "'");

        std::string name = current_.text;
        advance();

        // --- collision(obj, path) ----------------------------------
        if (name == "collision") {
            expect(TokenType::LPAREN);
            if (current_.type != TokenType::IDENT)
                throw ParseError("expected object variable in collision(obj, path)");
            std::string obj_var = current_.text;
            advance();
            expect(TokenType::COMMA);
            if (current_.type != TokenType::IDENT)
                throw ParseError("expected path variable in collision(obj, path)");
            std::string path_var = current_.text;
            advance();
            expect(TokenType::RPAREN);
            return makeAtomicProp(
                std::make_shared<CollisionAP>(obj_var, path_var));
        }

        // --- existenceProb(obj) op threshold --------------------------
        if (name == "existenceProb") {
            expect(TokenType::LPAREN);
            if (current_.type != TokenType::IDENT)
                throw ParseError("expected variable in existenceProb(var)");
            std::string obj_var = current_.text;
            advance();
            expect(TokenType::RPAREN);
            CompOp op = parseCompOp();
            if (current_.type != TokenType::NUMBER)
                throw ParseError("expected number after comparison op in existenceProb");
            double threshold = current_.num_val;
            advance();
            return makeAtomicProp(
                std::make_shared<ExistenceProbAP>(obj_var, op, threshold));
        }

        // --- confidence(path) op threshold -----------------------------
        if (name == "confidence") {
            expect(TokenType::LPAREN);
            if (current_.type != TokenType::IDENT)
                throw ParseError("expected variable in confidence(var)");
            std::string path_var = current_.text;
            advance();
            expect(TokenType::RPAREN);
            CompOp op = parseCompOp();
            if (current_.type != TokenType::NUMBER)
                throw ParseError("expected number after comparison op in confidence");
            double threshold = current_.num_val;
            advance();
            return makeAtomicProp(
                std::make_shared<ConfidenceAP>(path_var, op, threshold));
        }

        // --- time op threshold ------------------------------------------
        if (name == "time") {
            CompOp op = parseCompOp();
            if (current_.type != TokenType::NUMBER)
                throw ParseError("expected number after comparison op in time");
            double threshold = current_.num_val;
            advance();
            return makeAtomicProp(std::make_shared<TimeAP>(op, threshold));
        }

        // --- distance(obj, path) op threshold ---------------------------
        if (name == "distance") {
            expect(TokenType::LPAREN);
            if (current_.type != TokenType::IDENT)
                throw ParseError("expected object variable in distance(obj, path)");
            std::string obj_var = current_.text;
            advance();
            expect(TokenType::COMMA);
            if (current_.type != TokenType::IDENT)
                throw ParseError("expected path variable in distance(obj, path)");
            std::string path_var = current_.text;
            advance();
            expect(TokenType::RPAREN);
            CompOp op = parseCompOp();
            if (current_.type != TokenType::NUMBER)
                throw ParseError("expected number after comparison op in distance");
            double threshold = current_.num_val;
            advance();
            return makeAtomicProp(
                std::make_shared<DistanceAP>(obj_var, path_var, op, threshold));
        }

        // --- egoSpeed op threshold -------------------------------------
        if (name == "speed") {
            CompOp op = parseCompOp();
            if (current_.type != TokenType::NUMBER)
                throw ParseError("expected number after comparison op in speed");
            double threshold = current_.num_val;
            advance();
            return makeAtomicProp(
                std::make_shared<EgoSpeedAP>(op, threshold));
        }

        // --- egoAcceleration op threshold ------------------------------
        if (name == "acceleration") {
            CompOp op = parseCompOp();
            if (current_.type != TokenType::NUMBER)
                throw ParseError("expected number after comparison op in acceleration");
            double threshold = current_.num_val;
            advance();
            return makeAtomicProp(
                std::make_shared<EgoAccelerationAP>(op, threshold));
        }

        throw ParseError("unknown atomic proposition: '" + name + "'");
    }

    /// Parse a comparison operator token and return the corresponding CompOp.
    CompOp parseCompOp() {
        CompOp op;
        switch (current_.type) {
            case TokenType::GT:  op = CompOp::GT;  break;
            case TokenType::GEQ: op = CompOp::GEQ; break;
            case TokenType::LT:  op = CompOp::LT;  break;
            case TokenType::LEQ: op = CompOp::LEQ; break;
            case TokenType::EQ:  op = CompOp::EQ;  break;
            default:
                throw ParseError(
                    "expected comparison operator, got '" + current_.text + "'");
        }
        advance();
        return op;
    }
};

/// Convenience free function: parse a formula string and return its AST.
inline FormulaPtr parseFormula(const std::string& input) {
    return Parser(input).parse();
}

} // namespace fqltl
