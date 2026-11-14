#include <AUnit.h>
#include "../src/Stats.hpp"
#include "../src/EnvMath.hpp"
#include <array>
#include <vector>


test(Stats_MeanKeepsFractions) {
    // The old accumulate-into-long implementation returned 0 here.
    const std::array<float, 3> halves = {0.5f, 0.5f, 0.5f};
    assertNear(stats::mean(halves), 0.5f, 1e-6f);

    const std::vector<float> mixed = {-1.25f, 2.5f, 0.75f};
    assertNear(stats::mean(mixed), 0.6666667f, 1e-6f);
}


test(Stats_EmptyIsNaN) {
    const std::vector<float> empty;
    std::vector<float> emptyMutable;
    assertTrue(std::isnan(stats::mean(empty)));
    assertTrue(std::isnan(stats::stddev(empty)));
    assertTrue(std::isnan(stats::quartiles(emptyMutable)[1]));
    assertTrue(std::isnan(stats::robustMean(empty)));
}


test(Stats_StdDev) {
    const std::array<float, 8> values = {2, 4, 4, 4, 5, 5, 7, 9};
    assertNear(stats::stddev(values), 2.0f, 1e-6f);
    assertNear(stats::stddev(values, 5.0f), 2.0f, 1e-6f);
}


test(Stats_QuartilesAndOutliers) {
    std::vector<float> values = {10, 1, 9, 2, 8, 3, 7, 4, 6, 5, 1000};
    const std::array<float, 3> q = stats::quartiles(values);
    assertEqual(q[0], 3.0f);
    assertEqual(q[1], 6.0f);
    assertEqual(q[2], 9.0f);

    const std::vector<float> kept = stats::removeOutliers(values, q[0], q[2]);
    assertEqual(kept.size(), (size_t)10);
    assertNear(stats::mean(kept), 5.5f, 1e-6f);
}


test(Stats_RobustMeanLeavesInputUntouched) {
    const std::vector<float> values = {5, 5, 5, 5, 5, 5, 5, -900};
    assertNear(stats::robustMean(values), 5.0f, 1e-6f);
    assertEqual(values.back(), -900.0f);
}


test(Env_DewPoint) {
    // Reference values from the Magnus formula with Sonntag constants.
    assertNear(env::dewPoint(25.0f, 50.0f), 13.86f, 0.01f);
    assertNear(env::dewPoint(30.0f, 80.0f), 26.17f, 0.01f);
    assertNear(env::dewPoint(10.0f, 100.0f), 10.0f, 0.001f);
    assertTrue(std::isnan(env::dewPoint(20.0f, 0.0f)));
    assertTrue(std::isnan(env::dewPoint(20.0f, 101.0f)));
}


test(Env_AltitudeRoundTrip) {
    assertNear(env::altitude(101325.0f, 1013.25f), 0.0f, 0.001f);
    // ~ 8.4 m per hPa near sea level.
    assertNear(env::altitude(101225.0f, 1013.25f), 8.4f, 0.1f);

    const float qnh = env::seaLevelPressure(98000.0f, 300.0f);
    assertNear(env::altitude(98000.0f, qnh), 300.0f, 0.01f);
}
