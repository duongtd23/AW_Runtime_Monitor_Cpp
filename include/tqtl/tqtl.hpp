#ifndef TQTL_HPP
#define TQTL_HPP

/**
 * @file tqtl.hpp
 * @brief Main header file for the Timed Quality Temporal Logic (TQTL) library.
 * 
 * This header includes all core TQTL components.
 * 
 * TQTL is a formal logic for monitoring and evaluating the quality of
 * perception systems, particularly for autonomous vehicle applications.
 * 
 * Based on the paper:
 * "Evaluating Perception Systems for Autonomous Vehicles Using Quality Temporal Logic"
 * by Dokhanchi et al. (RV 2018)
 */

#include "QualityValue.hpp"
#include "BoundingBox.hpp"
#include "DataObject.hpp"
#include "Frame.hpp"
#include "DataStream.hpp"
#include "Environment.hpp"
#include "Formula.hpp"
#include "Predicate.hpp"
#include "Monitor.hpp"
#include "Parser.hpp"

namespace tqtl {

/**
 * @brief Library version information
 */
constexpr const char* VERSION = "0.1.0";
constexpr int VERSION_MAJOR = 0;
constexpr int VERSION_MINOR = 1;
constexpr int VERSION_PATCH = 0;

} // namespace tqtl

#endif // TQTL_HPP
