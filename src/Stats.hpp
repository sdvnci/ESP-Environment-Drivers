#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <type_traits>
#include <vector>

/**
 * Statistical helpers for filtering sensor samples.
 * Work on any container with begin()/end()/size() - std::array, std::vector, etc.
 */
namespace stats {

    /**
     * @brief Arithmetic mean, accumulated in double so float samples are not truncated.
     * @return The mean, or NaN for an empty container.
     */
    template <typename Container>
    float mean(const Container& values) {
        if (values.size() == 0) return NAN;
        double sum = 0.0;
        for (const auto& v : values) sum += v;
        return static_cast<float>(sum / values.size());
    }

    /**
     * @brief Population standard deviation.
     * @param meanValue The mean if already known, otherwise NaN to compute it.
     * @return The standard deviation, or NaN for an empty container.
     */
    template <typename Container>
    float stddev(const Container& values, float meanValue = NAN) {
        if (values.size() == 0) return NAN;
        const double m = std::isnan(meanValue) ? mean(values) : meanValue;

        double sum = 0.0;
        for (const auto& v : values) sum += (v - m) * (v - m);
        return static_cast<float>(std::sqrt(sum / values.size()));
    }

    /**
     * @brief First quartile, median and third quartile (nearest-rank).
     * NOTE: sorts the container in place.
     * @return {q1, median, q3}, or all NaN for an empty container.
     */
    template <typename Container>
    std::array<float, 3> quartiles(Container& values) {
        const size_t n = values.size();
        if (n == 0) return {NAN, NAN, NAN};

        std::sort(std::begin(values), std::end(values));
        auto at = [&values](size_t i) { return static_cast<float>(*(std::begin(values) + i)); };
        return {at(n / 4), at(n / 2), at((3 * n) / 4)};
    }

    /**
     * @brief Keep only the values within 1.5 IQR of the quartiles (Tukey's fences).
     * @return The values that are not outliers, in their original order.
     */
    template <typename Container>
    auto removeOutliers(const Container& values, float q1, float q3)
        -> std::vector<typename std::decay<decltype(*std::begin(values))>::type> {
        using T = typename std::decay<decltype(*std::begin(values))>::type;

        const float iqr = q3 - q1;
        const float lower = q1 - 1.5f * iqr;
        const float upper = q3 + 1.5f * iqr;

        std::vector<T> filtered;
        filtered.reserve(values.size());
        std::copy_if(
            std::begin(values),
            std::end(values),
            std::back_inserter(filtered),
            [lower, upper](const T& v) { return v >= lower && v <= upper; }
        );
        return filtered;
    }

    /**
     * @brief Mean of the values after removing outliers. Copies, so the input is untouched.
     * @return The filtered mean, or NaN for an empty container.
     */
    template <typename Container>
    float robustMean(const Container& values) {
        using T = typename std::decay<decltype(*std::begin(values))>::type;
        std::vector<T> sorted(std::begin(values), std::end(values));
        const std::array<float, 3> q = quartiles(sorted);
        const std::vector<T> kept = removeOutliers(sorted, q[0], q[2]);
        return mean(kept);
    }
};
