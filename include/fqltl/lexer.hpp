#pragma once
#include "token.hpp"
#include <string>
#include <vector>
#include <stdexcept>
#include <cctype>

/// @brief Lexer (tokenizer) for fqltl formula strings.
///
/// Recognized syntax elements:
///
///   Literals:    true   false
///   Logical:     ~  !  not     /\  &&  and     \/  ||  or     ->  =>
///   Temporal:    []  G          <>  F          X            U
///   Quantifiers: forall   exists
///   Comparison:  >  >=  <  <=  ==  =
///   Punctuation: (  )  ,  .  @
///   Values:      identifiers   numeric literals (int / float / sci-notation)
///
/// Note: single-letter temporal operators X, U, G, F are recognized as
/// keywords and cannot be used as variable names.

namespace fqltl {

class Lexer {
public:
    explicit Lexer(const std::string& input)
        : input_(input), pos_(0) {}

    /// Return the next token and advance the read position.
    Token nextToken() {
        skipWhitespaceAndComments();

        if (pos_ >= input_.size())
            return Token(TokenType::END, "");

        const size_t start = pos_;
        const char   c     = input_[pos_];

        // --- Two-character operators (must be checked before single-char) ---
        if (pos_ + 1 < input_.size()) {
            const std::string two = input_.substr(pos_, 2);
            if (two == "/\\") { pos_ += 2; return tok(TokenType::AND,      two); }
            if (two == "\\/") { pos_ += 2; return tok(TokenType::OR,       two); }
            if (two == "->")  { pos_ += 2; return tok(TokenType::IMPLIES,  two); }
            if (two == "=>")  { pos_ += 2; return tok(TokenType::IMPLIES,  two); }
            if (two == ">=")  { pos_ += 2; return tok(TokenType::GEQ,      two); }
            if (two == "<=")  { pos_ += 2; return tok(TokenType::LEQ,      two); }
            if (two == "==")  { pos_ += 2; return tok(TokenType::EQ,       two); }
            if (two == "[]")  { pos_ += 2; return tok(TokenType::GLOBALLY, two); }
            if (two == "<>")  { pos_ += 2; return tok(TokenType::EVENTUALLY, two); }
        }

        // --- Single-character tokens ----------------------------------------
        switch (c) {
            case '(': ++pos_; return tok(TokenType::LPAREN, "(");
            case ')': ++pos_; return tok(TokenType::RPAREN, ")");
            case ',': ++pos_; return tok(TokenType::COMMA,  ",");
            case '.': ++pos_; return tok(TokenType::DOT,    ".");
            case '@': ++pos_; return tok(TokenType::AT,     "@");
            case '~': ++pos_; return tok(TokenType::NOT,    "~");
            case '!': ++pos_; return tok(TokenType::NOT,    "!");
            case '>': ++pos_; return tok(TokenType::GT,     ">");
            case '<': ++pos_; return tok(TokenType::LT,     "<");
            case '=': ++pos_; return tok(TokenType::EQ,     "=");
        }

        // --- Numeric literal (possibly negative) ----------------------------
        if (std::isdigit(c) ||
            (c == '-' && pos_ + 1 < input_.size() &&
             std::isdigit(input_[pos_ + 1]))) {
            return scanNumber(start);
        }

        // --- Identifiers and keywords ---------------------------------------
        if (std::isalpha(c) || c == '_') {
            return scanIdentifier(start);
        }

        throw std::runtime_error(
            std::string("fqltl::Lexer: unexpected character '") + c +
            "' at position " + std::to_string(start));
    }

    /// Tokenize the entire input into a vector (including the final END token).
    std::vector<Token> tokenize() {
        std::vector<Token> tokens;
        while (true) {
            Token t = nextToken();
            tokens.push_back(t);
            if (t.type == TokenType::END) break;
        }
        return tokens;
    }

private:
    std::string input_;   // stored by value to avoid dangling-reference UB
    size_t      pos_;

    static Token tok(TokenType t, std::string s, double v = 0.0) {
        return Token(t, std::move(s), v);
    }

    void skipWhitespaceAndComments() {
        while (pos_ < input_.size()) {
            if (std::isspace(static_cast<unsigned char>(input_[pos_]))) {
                ++pos_;
            } else if (pos_ + 1 < input_.size() &&
                       input_[pos_] == '/' && input_[pos_ + 1] == '/') {
                // single-line comment: skip until newline
                while (pos_ < input_.size() && input_[pos_] != '\n') ++pos_;
            } else {
                break;
            }
        }
    }

    Token scanNumber(size_t start) {
        if (input_[pos_] == '-') ++pos_; // optional leading minus

        while (pos_ < input_.size() && std::isdigit(input_[pos_])) ++pos_;

        if (pos_ < input_.size() && input_[pos_] == '.') {
            ++pos_; // decimal point
            while (pos_ < input_.size() && std::isdigit(input_[pos_])) ++pos_;
        }

        // Scientific notation: e/E followed by optional +/- and digits
        if (pos_ < input_.size() &&
            (input_[pos_] == 'e' || input_[pos_] == 'E')) {
            ++pos_;
            if (pos_ < input_.size() &&
                (input_[pos_] == '+' || input_[pos_] == '-')) {
                ++pos_;
            }
            while (pos_ < input_.size() && std::isdigit(input_[pos_])) ++pos_;
        }

        std::string text = input_.substr(start, pos_ - start);
        double val = std::stod(text);
        return Token(TokenType::NUMBER, text, val);
    }

    Token scanIdentifier(size_t start) {
        while (pos_ < input_.size() &&
               (std::isalnum(input_[pos_]) || input_[pos_] == '_')) {
            ++pos_;
        }
        std::string text = input_.substr(start, pos_ - start);

        // --- keywords -----------------------------------------------------
        if (text == "true")   return tok(TokenType::TRUE,       text);
        if (text == "false")  return tok(TokenType::FALSE,      text);
        if (text == "not")    return tok(TokenType::NOT,        text);
        if (text == "and")    return tok(TokenType::AND,        text);
        if (text == "or")     return tok(TokenType::OR,         text);
        if (text == "forall") return tok(TokenType::FORALL,     text);
        if (text == "exists") return tok(TokenType::EXISTS,     text);

        // Single-letter temporal operator keywords
        // Note: these identifiers cannot be used as variable names.
        if (text == "G")  return tok(TokenType::GLOBALLY,   text);
        if (text == "F")  return tok(TokenType::EVENTUALLY, text);
        if (text == "X")  return tok(TokenType::NEXT,       text);
        if (text == "U")  return tok(TokenType::UNTIL,      text);

        return tok(TokenType::IDENT, text);
    }
};

} // namespace fqltl
