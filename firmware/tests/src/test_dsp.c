#include <zephyr/ztest.h>
#include "dsp_filter.h"

ZTEST_SUITE(dsp_suite, NULL, NULL, NULL, NULL, NULL);

/**
 * Test 1: Sea-level pressure (101325.0 Pa) must yield 0.0 m altitude.
 */
ZTEST(dsp_suite, test_altitude_sea_level)
{
    float alt = dsp_calculate_altitude(101325.0f);
    zassert_within(alt, 0.0f, 0.05f, "Sea level altitude should be 0.0 m");
}

/**
 * Test 2: Lower pressure (90000.0 Pa) must yield ~988.5 m positive altitude,
 * and higher pressure (103000.0 Pa) must yield negative altitude (below sea level).
 */
ZTEST(dsp_suite, test_altitude_known_pressures)
{
    float alt_high = dsp_calculate_altitude(90000.0f);
    zassert_within(alt_high, 988.5f, 1.5f, "90000 Pa should be ~988.5 m");

    float alt_low = dsp_calculate_altitude(103000.0f);
    zassert_true(alt_low < 0.0f, "Pressure above 101325 Pa must be below 0 m");
}

/**
 * Test 3: First sample into IIR filter must snap directly to input without lag,
 * and subsequent samples must follow y[n] = (1 - alpha)*y[n-1] + alpha*x[n].
 */
ZTEST(dsp_suite, test_iir_filter_step_response)
{
    struct dsp_iir_filter filter;
    dsp_iir_init(&filter, 0.10f); /* alpha = 0.10 */

    /* First sample: snaps directly to 100000.0 Pa */
    float y0 = dsp_iir_update(&filter, 100000.0f);
    zassert_within(y0, 100000.0f, 0.01f, "First sample should snap to input");

    /* Second sample with step to 101000.0 Pa:
     * Expected: 0.90 * 100000 + 0.10 * 101000 = 100100.0 Pa */
    float y1 = dsp_iir_update(&filter, 101000.0f);
    zassert_within(y1, 100100.0f, 0.05f, "Second sample should follow IIR formula");

    /* Third sample at 101000.0 Pa:
     * Expected: 0.90 * 100100 + 0.10 * 101000 = 100190.0 Pa */
    float y2 = dsp_iir_update(&filter, 101000.0f);
    zassert_within(y2, 100190.0f, 0.05f, "Third sample should converge toward target");
}

/**
 * Test 4: Defensive handling of invalid inputs (negative pressure, out-of-range alpha).
 */
ZTEST(dsp_suite, test_edge_cases)
{
    zassert_within(dsp_calculate_altitude(-50.0f), 0.0f, 0.001f,
                   "Negative pressure should safely return 0.0 m");

    struct dsp_iir_filter filter;
    dsp_iir_init(&filter, -1.0f); /* Invalid alpha -> should fall back to 0.15 */
    zassert_within(filter.alpha, DSP_DEFAULT_IIR_ALPHA, 0.001f,
                   "Out-of-bounds alpha should default to DSP_DEFAULT_IIR_ALPHA");
}
