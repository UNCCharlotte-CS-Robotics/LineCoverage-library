#pragma once
#include <cmath>

namespace mcpm {
	constexpr double EPSILON = 0.000001;
	constexpr double INFINITO = 1000000000.0;

	template <typename T, typename U>
		inline bool GREATER(T a, U b) { return (a - b) > EPSILON; }

	template <typename T>
		inline bool LESS(T a, T b) { return (b - a) > EPSILON; }

	template <typename T>
		inline bool EQUAL(T a, T b) { return std::abs(b - a) < EPSILON; }

	template <typename T>
		inline bool GREATER_EQUAL(T a, T b) { return GREATER(a, b) || EQUAL(a, b); }

	template <typename T>
		inline bool LESS_EQUAL(T a, T b) { return LESS(a, b) || EQUAL(a, b); }

	template <typename T>
		inline T MIN(T a, T b) { return std::min(a, b); }

	template <typename T>
		inline T MAX(T a, T b) { return std::max(a, b); }
}
