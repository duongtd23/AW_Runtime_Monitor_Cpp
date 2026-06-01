#pragma once
#include <string>
#include <ostream>

/// @brief Token types and Token struct for the fqltl lexer.
///
/// fqltl — Finite-domain Quantified LTL
/// Quantifiers are syntactic constructs that are grounded into propositional
/// LTL at runtime before being passed to an LTL model checker (e.g. Spot).

namespace fqltl {

/// All token types emitted by the Lexer.
enum class TokenType {
    // --- Boolean literals -------------------------------------------------
    TRUE,    ///< true
    FALSE,   ///< false

    // --- Logical operators ------------------------------------------------
    NOT,     ///< ~  !  not
    AND,     ///< /\  &&  and
    OR,      ///< \/  ||  or
    IMPLIES, ///< ->  =>

    // --- Temporal operators -----------------------------------------------
    GLOBALLY,    ///< []  G
    EVENTUALLY,  ///< <>  F
    NEXT,        ///< X
    UNTIL,       ///< U

    // --- Quantifiers ------------------------------------------------------
    FORALL,  ///< forall
    EXISTS,  ///< exists

    // --- Comparison operators ---------------------------------------------
    GT,   ///< >
    GEQ,  ///< >=
    LT,   ///< <
    LEQ,  ///< <=
    EQ,   ///< ==  =

    // --- Punctuation ------------------------------------------------------
    LPAREN,  ///< (
    RPAREN,  ///< )
    COMMA,   ///< ,
    DOT,     ///< .
    AT,      ///< @

    // --- Values -----------------------------------------------------------
    IDENT,   ///< identifier (variable or AP function name)
    NUMBER,  ///< numeric literal

    // --- Sentinel ---------------------------------------------------------
    END      ///< end of input
};

/// A single token produced by the Lexer.
struct Token {
    TokenType   type    = TokenType::END;
    std::string text;          ///< original source text of this token
    double      num_val = 0.0; ///< numeric value when type == NUMBER

    Token() = default;
    Token(TokenType t, std::string s, double v = 0.0)
        : type(t), text(std::move(s)), num_val(v) {}
};

/// Human-readable name for a TokenType (useful in error messages).
inline const char* tokenTypeName(TokenType t) {
    switch (t) {
        case TokenType::TRUE:       return "true";
        case TokenType::FALSE:      return "false";
        case TokenType::NOT:        return "~";
        case TokenType::AND:        return "/\\";
        case TokenType::OR:         return "\\/";
        case TokenType::IMPLIES:    return "->";
        case TokenType::GLOBALLY:   return "[]";
        case TokenType::EVENTUALLY: return "<>";
        case TokenType::NEXT:       return "X";
        case TokenType::UNTIL:      return "U";
        case TokenType::FORALL:     return "forall";
        case TokenType::EXISTS:     return "exists";
        case TokenType::GT:         return ">";
        case TokenType::GEQ:        return ">=";
        case TokenType::LT:         return "<";
        case TokenType::LEQ:        return "<=";
        case TokenType::EQ:         return "=";
        case TokenType::LPAREN:     return "(";
        case TokenType::RPAREN:     return ")";
        case TokenType::COMMA:      return ",";
        case TokenType::DOT:        return ".";
        case TokenType::AT:         return "@";
        case TokenType::IDENT:      return "<ident>";
        case TokenType::NUMBER:     return "<number>";
        case TokenType::END:        return "<end>";
        default:                    return "<unknown>";
    }
}

inline std::ostream& operator<<(std::ostream& os, const Token& tok) {
    os << tokenTypeName(tok.type);
    if (tok.type == TokenType::IDENT)  os << "(" << tok.text    << ")";
    if (tok.type == TokenType::NUMBER) os << "(" << tok.num_val << ")";
    return os;
}

} // namespace fqltl
