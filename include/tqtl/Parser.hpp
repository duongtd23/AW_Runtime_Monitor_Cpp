#ifndef TQTL_PARSER_HPP
#define TQTL_PARSER_HPP

#include "Formula.hpp"
#include "Predicate.hpp"
#include <string>
#include <memory>
#include <stdexcept>
#include <cctype>
#include <vector>
#include <sstream>

namespace tqtl {

/**
 * @brief Exception thrown when parsing fails.
 */
class ParseError : public std::runtime_error {
public:
    ParseError(const std::string& message, size_t position)
        : std::runtime_error(message + " at position " + std::to_string(position)),
          position_(position) {}
    
    size_t getPosition() const { return position_; }

private:
    size_t position_;
};

/**
 * @brief Token types for the lexer.
 */
enum class TokenType {
    // Literals
    TRUE,           // true, ⊤, T
    FALSE,          // false, ⊥, F
    
    // Logical operators
    NOT,            // !, ¬, not, ~
    AND,            // &&, ∧, and, &
    OR,             // ||, ∨, or, |
    IMPLIES,        // ->, →, =>
    
    // Temporal operators
    UNTIL,          // U
    EVENTUALLY,     // <>, ◇, F (eventually/finally)
    ALWAYS,         // [], □, G (globally)
    
    // Quantifiers
    EXISTS,         // exists, ∃, E
    FORALL,         // forall, ∀, A
    
    // Time operators
    FREEZE,         // . (after variable name)
    AT,             // @
    LEQ,            // <=, ≤
    LT,             // <
    GEQ,            // >=, ≥
    GT,             // >
    EQ,             // =, ==
    NEQ,            // !=, ≠
    
    // Predicate keywords
    CLASS,          // C, Class
    PROB,           // P, Prob
    DIST,           // dist, distance
    IOU,            // IoU, iou
    PRESENT,        // present

    // Predicates related to ego vehicle
    DIST_EGO,       // dist_ego
    ANGLE_EGO,      // angle_ego

    // Punctuation
    LPAREN,         // (
    RPAREN,         // )
    COMMA,          // ,
    DOT,            // .
    PLUS,           // +
    MINUS,          // -
    
    // Values
    IDENTIFIER,     // variable names, class names
    NUMBER,         // numeric literals
    STRING,         // string literals (for class names)
    
    // Special
    END_OF_INPUT,
    UNKNOWN
};

/**
 * @brief Token structure.
 */
struct Token {
    TokenType type;
    std::string value;
    size_t position;
    
    Token(TokenType t = TokenType::UNKNOWN, std::string v = "", size_t pos = 0)
        : type(t), value(std::move(v)), position(pos) {}
};

/**
 * @brief Lexer for tokenizing TQTL formula strings.
 */
class Lexer {
public:
    explicit Lexer(const std::string& input) : input_(input), pos_(0) {}
    
    Token nextToken() {
        skipWhitespace();
        
        if (pos_ >= input_.size()) {
            return Token(TokenType::END_OF_INPUT, "", pos_);
        }
        
        size_t startPos = pos_;
        char c = input_[pos_];
        
        // Single character tokens
        switch (c) {
            case '(': pos_++; return Token(TokenType::LPAREN, "(", startPos);
            case ')': pos_++; return Token(TokenType::RPAREN, ")", startPos);
            case ',': pos_++; return Token(TokenType::COMMA, ",", startPos);
            case '+': pos_++; return Token(TokenType::PLUS, "+", startPos);
            case '~': pos_++; return Token(TokenType::NOT, "~", startPos);
        }
        
        // Unicode operators
        if (c == '\xE2') { // UTF-8 multi-byte
            std::string unicode = peekUnicode();
            if (unicode == "⊤") { pos_ += 3; return Token(TokenType::TRUE, "⊤", startPos); }
            if (unicode == "⊥") { pos_ += 3; return Token(TokenType::FALSE, "⊥", startPos); }
            if (unicode == "¬") { pos_ += 2; return Token(TokenType::NOT, "¬", startPos); }
            if (unicode == "∧") { pos_ += 3; return Token(TokenType::AND, "∧", startPos); }
            if (unicode == "∨") { pos_ += 3; return Token(TokenType::OR, "∨", startPos); }
            if (unicode == "→") { pos_ += 3; return Token(TokenType::IMPLIES, "→", startPos); }
            if (unicode == "◇") { pos_ += 3; return Token(TokenType::EVENTUALLY, "◇", startPos); }
            if (unicode == "□") { pos_ += 3; return Token(TokenType::ALWAYS, "□", startPos); }
            if (unicode == "∃") { pos_ += 3; return Token(TokenType::EXISTS, "∃", startPos); }
            if (unicode == "∀") { pos_ += 3; return Token(TokenType::FORALL, "∀", startPos); }
            if (unicode == "≤") { pos_ += 3; return Token(TokenType::LEQ, "≤", startPos); }
            if (unicode == "≥") { pos_ += 3; return Token(TokenType::GEQ, "≥", startPos); }
            if (unicode == "≠") { pos_ += 3; return Token(TokenType::NEQ, "≠", startPos); }
        }
        
        // Two-character operators
        if (pos_ + 1 < input_.size()) {
            std::string two = input_.substr(pos_, 2);
            if (two == "&&") { pos_ += 2; return Token(TokenType::AND, "&&", startPos); }
            if (two == "||") { pos_ += 2; return Token(TokenType::OR, "||", startPos); }
            if (two == "->") { pos_ += 2; return Token(TokenType::IMPLIES, "->", startPos); }
            if (two == "=>") { pos_ += 2; return Token(TokenType::IMPLIES, "=>", startPos); }
            if (two == "<=") { pos_ += 2; return Token(TokenType::LEQ, "<=", startPos); }
            if (two == ">=") { pos_ += 2; return Token(TokenType::GEQ, ">=", startPos); }
            if (two == "==") { pos_ += 2; return Token(TokenType::EQ, "==", startPos); }
            if (two == "!=") { pos_ += 2; return Token(TokenType::NEQ, "!=", startPos); }
            if (two == "<>") { pos_ += 2; return Token(TokenType::EVENTUALLY, "<>", startPos); }
            if (two == "[]") { pos_ += 2; return Token(TokenType::ALWAYS, "[]", startPos); }
        }
        
        // Single character operators (after checking two-char)
        switch (c) {
            case '!': pos_++; return Token(TokenType::NOT, "!", startPos);
            case '&': pos_++; return Token(TokenType::AND, "&", startPos);
            case '|': pos_++; return Token(TokenType::OR, "|", startPos);
            case '<': pos_++; return Token(TokenType::LT, "<", startPos);
            case '>': pos_++; return Token(TokenType::GT, ">", startPos);
            case '=': pos_++; return Token(TokenType::EQ, "=", startPos);
            case '@': pos_++; return Token(TokenType::AT, "@", startPos);
            case '.': pos_++; return Token(TokenType::DOT, ".", startPos);
            case '-': 
                // Check if it's a negative number
                if (pos_ + 1 < input_.size() && std::isdigit(input_[pos_ + 1])) {
                    return scanNumber();
                }
                pos_++; 
                return Token(TokenType::MINUS, "-", startPos);
        }
        
        // Numbers
        if (std::isdigit(c)) {
            return scanNumber();
        }
        
        // String literals
        if (c == '"' || c == '\'') {
            return scanString();
        }
        
        // Identifiers and keywords
        if (std::isalpha(c) || c == '_') {
            return scanIdentifier();
        }
        
        // Unknown character
        pos_++;
        return Token(TokenType::UNKNOWN, std::string(1, c), startPos);
    }
    
    Token peek() {
        size_t savedPos = pos_;
        Token t = nextToken();
        pos_ = savedPos;
        return t;
    }
    
    size_t getPosition() const { return pos_; }

private:
    std::string input_;
    size_t pos_;
    
    void skipWhitespace() {
        while (pos_ < input_.size() && std::isspace(input_[pos_])) {
            pos_++;
        }
    }
    
    std::string peekUnicode() {
        // Try to read a UTF-8 character
        if (pos_ >= input_.size()) return "";
        
        unsigned char c = input_[pos_];
        size_t len = 1;
        if ((c & 0xE0) == 0xC0) len = 2;
        else if ((c & 0xF0) == 0xE0) len = 3;
        else if ((c & 0xF8) == 0xF0) len = 4;
        
        if (pos_ + len > input_.size()) return "";
        return input_.substr(pos_, len);
    }
    
    Token scanNumber() {
        size_t start = pos_;
        bool hasDecimal = false;
        
        if (input_[pos_] == '-') pos_++;
        
        while (pos_ < input_.size()) {
            char c = input_[pos_];
            if (std::isdigit(c)) {
                pos_++;
            } else if (c == '.' && !hasDecimal) {
                // Check if next char is a digit (not a method call)
                if (pos_ + 1 < input_.size() && std::isdigit(input_[pos_ + 1])) {
                    hasDecimal = true;
                    pos_++;
                } else {
                    break;
                }
            } else {
                break;
            }
        }
        
        return Token(TokenType::NUMBER, input_.substr(start, pos_ - start), start);
    }
    
    Token scanString() {
        size_t start = pos_;
        char quote = input_[pos_++];
        
        std::string value;
        while (pos_ < input_.size() && input_[pos_] != quote) {
            if (input_[pos_] == '\\' && pos_ + 1 < input_.size()) {
                pos_++;
                value += input_[pos_++];
            } else {
                value += input_[pos_++];
            }
        }
        
        if (pos_ < input_.size()) pos_++; // skip closing quote
        
        return Token(TokenType::STRING, value, start);
    }
    
    Token scanIdentifier() {
        size_t start = pos_;
        
        while (pos_ < input_.size() && 
               (std::isalnum(input_[pos_]) || input_[pos_] == '_')) {
            pos_++;
        }
        
        std::string value = input_.substr(start, pos_ - start);
        
        // Check for keywords
        if (value == "true" || value == "T") return Token(TokenType::TRUE, value, start);
        if (value == "false" || value == "F") return Token(TokenType::FALSE, value, start);
        if (value == "not") return Token(TokenType::NOT, value, start);
        if (value == "and") return Token(TokenType::AND, value, start);
        if (value == "or") return Token(TokenType::OR, value, start);
        if (value == "implies") return Token(TokenType::IMPLIES, value, start);
        if (value == "U") return Token(TokenType::UNTIL, value, start);
        if (value == "F" || value == "eventually") return Token(TokenType::EVENTUALLY, value, start);
        if (value == "G" || value == "always") return Token(TokenType::ALWAYS, value, start);
        if (value == "exists" || value == "E") return Token(TokenType::EXISTS, value, start);
        if (value == "forall" || value == "A") return Token(TokenType::FORALL, value, start);
        if (value == "C" || value == "Class") return Token(TokenType::CLASS, value, start);
        if (value == "P" || value == "Prob") return Token(TokenType::PROB, value, start);
        if (value == "dist" || value == "distance") return Token(TokenType::DIST, value, start);
        if (value == "present") return Token(TokenType::PRESENT, value, start);
        if (value == "IoU" || value == "iou") return Token(TokenType::IOU, value, start);
        if (value == "dist_ego") return Token(TokenType::DIST_EGO, value, start);
        if (value == "angle_ego") return Token(TokenType::ANGLE_EGO, value, start);
        
        return Token(TokenType::IDENTIFIER, value, start);
    }
};

/**
 * @brief Parser for TQTL formula strings.
 * 
 * Grammar (simplified):
 *   formula     := implication
 *   implication := disjunction (('->' | '=>' | '→') disjunction)*
 *   disjunction := conjunction (('||' | '∨' | 'or') conjunction)*
 *   conjunction := until (('&&' | '∧' | 'and') until)*
 *   until       := unary ('U' unary)*
 *   unary       := ('!' | '¬' | 'not') unary
 *                | ('[]' | '□' | 'G' | 'always') unary
 *                | ('<>' | '◇' | 'F' | 'eventually') unary
 *                | quantified
 *   quantified  := ('∃' | 'exists') IDENT '@' IDENT '.' formula
 *                | ('∀' | 'forall') IDENT '@' IDENT '.' formula
 *                | freeze
 *   freeze      := IDENT '.' formula
 *                | primary
 *   primary     := 'true' | '⊤' | 'T'
 *                | 'false' | '⊥' | 'F'
 *                | predicate
 *                | timeConstraint
 *                | '(' formula ')'
 *   predicate   := 'C' '(' IDENT ',' IDENT ')' ('=' | '!=') className
 *                | 'P' '(' IDENT ',' IDENT ')' ('<' | '>' | '<=' | '>=') NUMBER
 *                | 'exists' '(' IDENT '@' IDENT ')'
 *   timeConstraint := IDENT '<=' IDENT ('+' | '-') NUMBER
 *                   | IDENT '<=' IDENT
 */
class Parser {
public:
    /**
     * @brief Parse a TQTL formula string into an AST.
     * 
     * @param input The formula string
     * @return The parsed formula
     * @throws ParseError if parsing fails
     */
    static FormulaPtr parse(const std::string& input) {
        Parser parser(input);
        FormulaPtr result = parser.parseFormula();
        
        // Ensure we consumed all input
        Token t = parser.lexer_.peek();
        if (t.type != TokenType::END_OF_INPUT) {
            throw ParseError("Unexpected token: " + t.value, t.position);
        }
        
        return result;
    }

private:
    Lexer lexer_;
    Token currentToken_;
    
    explicit Parser(const std::string& input) : lexer_(input) {
        advance();
    }
    
    void advance() {
        currentToken_ = lexer_.nextToken();
    }
    
    bool check(TokenType type) const {
        return currentToken_.type == type;
    }
    
    bool match(TokenType type) {
        if (check(type)) {
            advance();
            return true;
        }
        return false;
    }
    
    Token expect(TokenType type, const std::string& message) {
        if (!check(type)) {
            throw ParseError(message + ", got: " + currentToken_.value, 
                           currentToken_.position);
        }
        Token t = currentToken_;
        advance();
        return t;
    }
    
    FormulaPtr parseFormula() {
        return parseImplication();
    }
    
    FormulaPtr parseImplication() {
        FormulaPtr left = parseDisjunction();
        
        while (check(TokenType::IMPLIES)) {
            advance();
            FormulaPtr right = parseDisjunction();
            left = formula::Implies(left, right);
        }
        
        return left;
    }
    
    FormulaPtr parseDisjunction() {
        FormulaPtr left = parseConjunction();
        
        while (check(TokenType::OR)) {
            advance();
            FormulaPtr right = parseConjunction();
            left = formula::Or(left, right);
        }
        
        return left;
    }
    
    FormulaPtr parseConjunction() {
        FormulaPtr left = parseUntil();
        
        while (check(TokenType::AND)) {
            advance();
            FormulaPtr right = parseUntil();
            left = formula::And(left, right);
        }
        
        return left;
    }
    
    FormulaPtr parseUntil() {
        FormulaPtr left = parseUnary();
        
        while (check(TokenType::UNTIL)) {
            advance();
            FormulaPtr right = parseUnary();
            left = formula::Until(left, right);
        }
        
        return left;
    }
    
    FormulaPtr parseUnary() {
        if (check(TokenType::NOT)) {
            advance();
            return formula::Not(parseUnary());
        }
        
        if (check(TokenType::ALWAYS)) {
            advance();
            return formula::Always(parseUnary());
        }
        
        if (check(TokenType::EVENTUALLY)) {
            advance();
            return formula::Eventually(parseUnary());
        }
        
        return parseQuantified();
    }
    
    FormulaPtr parseQuantified() {
        if (check(TokenType::EXISTS)) {
            advance();
            std::string objVar = expect(TokenType::IDENTIFIER, "Expected object variable").value;
            expect(TokenType::AT, "Expected '@'");
            FrameExpr frameExpr = parseFrameExpr();
            expect(TokenType::DOT, "Expected '.'");
            FormulaPtr body = parseFormula();
            return formula::Exists(objVar, frameExpr, body);
        }
        
        if (check(TokenType::FORALL)) {
            advance();
            std::string objVar = expect(TokenType::IDENTIFIER, "Expected object variable").value;
            expect(TokenType::AT, "Expected '@'");
            FrameExpr frameExpr = parseFrameExpr();
            expect(TokenType::DOT, "Expected '.'");
            FormulaPtr body = parseFormula();
            return formula::ForAll(objVar, frameExpr, body);
        }
        
        return parseFreeze();
    }

    /**
     * @brief Parse a frame expression: either "x+n" or "x-n", without parentheses
     * 
     * FrameExpr ::= IDENTIFIER ('+' | '-') NUMBER
     */
    FrameExpr parseFrameExprWithoutParentThes() {
        std::string varName = expect(TokenType::IDENTIFIER, "Expected time variable in frame expression").value;
        int offset = 0;
        if (check(TokenType::PLUS)) {
            advance();
            Token num = expect(TokenType::NUMBER, "Expected offset number");
            offset = std::stoi(num.value);
        } else if (check(TokenType::MINUS)) {
            advance();
            Token num = expect(TokenType::NUMBER, "Expected offset number");
            offset = -std::stoi(num.value);
        }
        return FrameExpr(varName, offset);
    }
    
    /**
     * @brief Parse a frame expression: either "x" or "(x+n)" or "(x-n)"
     * 
     * FrameExpr ::= IDENTIFIER | '(' IDENTIFIER ('+' | '-') NUMBER ')'
     */
    FrameExpr parseFrameExpr() {
        // Check if it's a parenthesized expression (x+n) or (x-n)
        if (check(TokenType::LPAREN)) {
            advance();  // consume '('
            auto result = parseFrameExprWithoutParentThes();
            expect(TokenType::RPAREN, "Expected ')' after frame expression");
            return result;
        }
        
        // Simple identifier (no offset)
        std::string varName = expect(TokenType::IDENTIFIER, "Expected time variable").value;
        return FrameExpr(varName, 0);
    }

    /**
     * @brief Parse a frame expression: "x+n" or (x-n), which is only allow inside function parameters;
     *  in addition to "x", "(x+n)", "(x-n)"
     */
    FrameExpr parseFrameExprInsideFuncParams() {
        // Check if it's a parenthesized expression (x+n) or (x-n)
        if (check(TokenType::LPAREN)) {
            advance();  // consume '('
            auto result = parseFrameExprWithoutParentThes();
            expect(TokenType::RPAREN, "Expected ')' after frame expression");
            return result;
        }
        
        // If it's an expression x+n or x-n without parentheses
        return parseFrameExprWithoutParentThes();
    }
    
    FormulaPtr parseFreeze() {
        // Check if this is a freeze: IDENT '.' formula
        // But be careful not to consume predicates like C(x, id)
        if (check(TokenType::IDENTIFIER)) {
            Token ident = currentToken_;
            advance();
            
            if (check(TokenType::DOT)) {
                advance();
                FormulaPtr body = parseFormula();
                return formula::Freeze(ident.value, body);
            }
            
            // Check for time constraint: x <= y + n
            if (check(TokenType::LEQ) || check(TokenType::LT) || 
                check(TokenType::GEQ) || check(TokenType::GT)) {
                return parseTimeConstraint(ident.value);
            }
            
            // Not a freeze or time constraint, must be an error or part of something else
            throw ParseError("Unexpected identifier: " + ident.value, ident.position);
        }
        
        return parsePrimary();
    }
    
    FormulaPtr parseTimeConstraint(const std::string& leftVar) {
        TokenType op = currentToken_.type;
        advance();
        
        std::string rightVar = expect(TokenType::IDENTIFIER, "Expected time variable").value;
        
        int offset = 0;
        if (check(TokenType::PLUS)) {
            advance();
            Token num = expect(TokenType::NUMBER, "Expected offset number");
            offset = std::stoi(num.value);
        } else if (check(TokenType::MINUS)) {
            advance();
            Token num = expect(TokenType::NUMBER, "Expected offset number");
            offset = -std::stoi(num.value);
        }
        
        // Create appropriate constraint based on operator
        switch (op) {
            case TokenType::LEQ:
                return formula::TimeLeq(leftVar, rightVar, offset);
            case TokenType::LT:
                return formula::TimeLt(leftVar, rightVar, offset);
            case TokenType::GEQ:
                return formula::TimeGeq(leftVar, rightVar, offset);
            case TokenType::GT:
                return formula::TimeGt(leftVar, rightVar, offset);
            default:
                throw ParseError("Invalid time constraint operator", currentToken_.position);
        }
    }
    
    /**
     * @brief Parse a comparison operator and return the corresponding ComparisonOp.
     * @throws ParseError if no valid comparison operator is found.
     */
    ComparisonOp parseComparisonOp() {
        if (match(TokenType::GEQ)) return ComparisonOp::GE;
        if (match(TokenType::GT))  return ComparisonOp::GT;
        if (match(TokenType::LEQ)) return ComparisonOp::LE;
        if (match(TokenType::LT))  return ComparisonOp::LT;
        if (match(TokenType::EQ))  return ComparisonOp::EQ;
        if (match(TokenType::NEQ)) return ComparisonOp::NE;
        throw ParseError("Expected comparison operator", currentToken_.position);
    }
    
    FormulaPtr parsePrimary() {
        if (check(TokenType::TRUE)) {
            advance();
            return formula::True();
        }
        
        if (check(TokenType::FALSE)) {
            advance();
            return formula::False();
        }
        
        if (check(TokenType::LPAREN)) {
            advance();
            FormulaPtr inner = parseFormula();
            expect(TokenType::RPAREN, "Expected ')'");
            return inner;
        }
        
        // Predicates
        if (check(TokenType::CLASS)) {
            return parseClassPredicate();
        }
        
        if (check(TokenType::PROB)) {
            return parseProbabilityPredicate();
        }
        
        if (check(TokenType::DIST)) {
            return parseDistancePredicate();
        }
        
        if (check(TokenType::IOU)) {
            return parseIoUPredicate();
        }

        if (check(TokenType::PRESENT)) {
            return parsePresentPredicate();
        }

        if (check(TokenType::DIST_EGO)) {
            return parseEgoDistancePredicate();
        }

        if (check(TokenType::ANGLE_EGO)) {
            return parseEgoAnglePredicate();
        }
        
        throw ParseError("Unexpected token: " + currentToken_.value, currentToken_.position);
    }
    
    FormulaPtr parseClassPredicate() {
        advance(); // consume 'C' or 'Class'
        expect(TokenType::LPAREN, "Expected '('");
        FrameExpr frameExpr = parseFrameExprInsideFuncParams();
        expect(TokenType::COMMA, "Expected ','");
        std::string objVar = expect(TokenType::IDENTIFIER, "Expected object variable").value;
        expect(TokenType::RPAREN, "Expected ')'");
        
        bool isEquals = true;
        if (check(TokenType::EQ)) {
            advance();
            isEquals = true;
        } else if (check(TokenType::NEQ)) {
            advance();
            isEquals = false;
        } else {
            throw ParseError("Expected '=' or '!='", currentToken_.position);
        }
        
        // Parse class name
        std::string className;
        if (check(TokenType::IDENTIFIER)) {
            className = currentToken_.value;
            advance();
        } else if (check(TokenType::STRING)) {
            className = currentToken_.value;
            advance();
        } else {
            throw ParseError("Expected class name", currentToken_.position);
        }
        
        ObjectClass cls = stringToObjectClass(className);
        
        if (isEquals) {
            return predicate::ClassEquals(frameExpr, objVar, cls);
        } else {
            return predicate::ClassNotEquals(frameExpr, objVar, cls);
        }
    }
    
    FormulaPtr parseProbabilityPredicate() {
        advance(); // consume 'P' or 'Prob'
        expect(TokenType::LPAREN, "Expected '('");
        FrameExpr frameExpr = parseFrameExprInsideFuncParams();
        expect(TokenType::COMMA, "Expected ','");
        std::string objVar = expect(TokenType::IDENTIFIER, "Expected object variable").value;
        expect(TokenType::RPAREN, "Expected ')'");
        
        ComparisonOp op = parseComparisonOp();
        
        Token numToken = expect(TokenType::NUMBER, "Expected probability threshold");
        double threshold = std::stod(numToken.value);
        
        return std::make_shared<ProbabilityPredicate>(frameExpr, objVar, op, threshold);
    }
    
    FormulaPtr parseDistancePredicate() {
        advance(); // consume 'dist'

        std::string objVar2;
        expect(TokenType::LPAREN, "Expected '('");
        FrameExpr frameExpr1 = parseFrameExprInsideFuncParams();
        expect(TokenType::COMMA, "Expected ','");
        std::string objVar1 = expect(TokenType::IDENTIFIER, "Expected object variable").value;
        expect(TokenType::COMMA, "Expected ','");
        FrameExpr frameExpr2 = parseFrameExprInsideFuncParams();
        if (check(TokenType::RPAREN)) {
            // same object, different times/frames
            advance();
        } else {
            expect(TokenType::COMMA, "Expected ','");
            objVar2 = expect(TokenType::IDENTIFIER, "Expected object variable").value;
            expect(TokenType::RPAREN, "Expected ')'");
        }

        ComparisonOp op = parseComparisonOp();
        
        Token numToken = expect(TokenType::NUMBER, "Expected distance threshold");
        double threshold = std::stod(numToken.value);
        
        if (objVar2.empty())
            return predicate::Distance(frameExpr1, frameExpr2, objVar1, op, threshold);
        return predicate::DistancePos(frameExpr1, objVar1, frameExpr2, objVar2, op, threshold);
    }
    
    FormulaPtr parseIoUPredicate() {
        advance(); // consume 'IoU'
        expect(TokenType::LPAREN, "Expected '('");
        FrameExpr frameExpr1 = parseFrameExprInsideFuncParams();
        expect(TokenType::COMMA, "Expected ','");
        std::string objVar1 = expect(TokenType::IDENTIFIER, "Expected object variable").value;
        expect(TokenType::COMMA, "Expected ','");
        FrameExpr frameExpr2 = parseFrameExprInsideFuncParams();
        expect(TokenType::COMMA, "Expected ','");
        std::string objVar2 = expect(TokenType::IDENTIFIER, "Expected object variable").value;
        expect(TokenType::RPAREN, "Expected ')'");
        
        ComparisonOp op = parseComparisonOp();
        
        Token numToken = expect(TokenType::NUMBER, "Expected IoU threshold");
        double threshold = std::stod(numToken.value);
        
        return std::make_shared<IoUPredicate>(
            frameExpr1, objVar1, frameExpr2, objVar2, op, threshold);
    }

    FormulaPtr parsePresentPredicate() {
        advance(); // consume 'present'
        expect(TokenType::LPAREN, "Expected '('");
        FrameExpr frameExpr = parseFrameExprInsideFuncParams();
        expect(TokenType::COMMA, "Expected ','");
        std::string objectVar = expect(TokenType::IDENTIFIER, "Expected object variable").value;
        expect(TokenType::RPAREN, "Expected ')'");
        
        return std::make_shared<ObjectPresentPredicate>(frameExpr, objectVar);
    }

    FormulaPtr parseEgoDistancePredicate() {
        advance(); // consume 'dist_ego'
        expect(TokenType::LPAREN, "Expected '('");
        FrameExpr frameExpr = parseFrameExprInsideFuncParams();
        expect(TokenType::COMMA, "Expected ','");
        std::string objVar = expect(TokenType::IDENTIFIER, "Expected object variable").value;
        expect(TokenType::RPAREN, "Expected ')'");
        
        ComparisonOp op = parseComparisonOp();
        
        Token numToken = expect(TokenType::NUMBER, "Expected distance threshold");
        double threshold = std::stod(numToken.value);
        
        return std::make_shared<DistanceToEgoPredicate>(frameExpr, objVar, op, threshold);
    }

    FormulaPtr parseEgoAnglePredicate() {
        advance(); // consume 'angle_ego'
        expect(TokenType::LPAREN, "Expected '('");
        FrameExpr frameExpr = parseFrameExprInsideFuncParams();
        expect(TokenType::COMMA, "Expected ','");
        std::string objVar = expect(TokenType::IDENTIFIER, "Expected object variable").value;
        expect(TokenType::RPAREN, "Expected ')'");
        
        ComparisonOp op = parseComparisonOp();
        
        Token numToken = expect(TokenType::NUMBER, "Expected angle threshold");
        double threshold = std::stod(numToken.value);
        
        return std::make_shared<EgoViewAnglePredicate>(frameExpr, objVar, op, threshold);
    }
};

// ============================================================================
// Convenience Functions
// ============================================================================

/**
 * @brief Parse a TQTL formula string.
 * 
 * @param formulaString The formula string to parse
 * @return The parsed formula AST
 * @throws ParseError if parsing fails
 * 
 * Example usage:
 * @code
 * auto f = parseFormula("[]( x.∃id@x. C(x,id)=Car && P(x,id)>0.8 )");
 * @endcode
 */
inline FormulaPtr parseFormula(const std::string& formulaString) {
    return Parser::parse(formulaString);
}

/**
 * @brief Try to parse a formula, returning nullopt on failure.
 * 
 * @param formulaString The formula string to parse
 * @param errorMessage Output parameter for error message on failure
 * @return The parsed formula, or nullopt if parsing failed
 */
inline std::optional<FormulaPtr> tryParseFormula(const std::string& formulaString, 
                                                   std::string* errorMessage = nullptr) {
    try {
        return Parser::parse(formulaString);
    } catch (const ParseError& e) {
        if (errorMessage) {
            *errorMessage = e.what();
        }
        return std::nullopt;
    }
}

} // namespace tqtl

#endif // TQTL_PARSER_HPP
