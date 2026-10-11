#include <zephyr/ztest.h>
#include "dsp_filter.h"

ZTEST_SUITE(dsp_suite, NULL, NULL, NULL, NULL, NULL);

ZTEST(dsp_suite, test_altitude_sea_level)
{
    float alt = dsp_calculate_altitude(101325.0f);
    zassert_within(alt, 0.0f, 0.05f, "Sea level altitude should be 0.0 m");
}

ZTEST(dsp_suite, test_altitude_known_pressures)
{
    float alt_high = dsp_calculate_altitude(90000.0f);
    zassert_within(alt_high, 988.5f, 1.5f, "90000 Pa should be ~988.5 m");

    float alt_low = dsp_calculate_altitude(103000.0f);
    zassert_true(alt_low < 0.0f, "Pressure above 101325 Pa should be below 0 m");
}

ZTEST(dsp_suite, test_iir_filter_step_response)
{
    struct dsp_iir_filter filter;
    dsp_iir_init(&filter, 0.10f);

    float y0 = dsp_iir_update(&filter, 100000.0f);
    zassert_within(y0, 100000.0f, 0.01f, "Initial sample should set filter state directly");

    /* 0.90 * 100000 + 0.10 * 101000 = 100100.0 */
    float y1 = dsp_iir_update(&filter, 101000.0f);
    zassert_within(y1, 100100.0f, 0.05f, "Step response mismatch on second sample");

    /* 0.90 * 100100 + 0.10 * 101000 = 100190.0 */
    float y2 = dsp_iir_update(&filter, 101000.0f);
    zassert_within(y2, 100190.0f, 0.05f, "Step response mismatch on third sample");
}

ZTEST(dsp_suite, test_edge_cases)
{
    zassert_within(dsp_calculate_altitude(-50.0f), 0.0f, 0.001f,
                   "Negative pressure should return 0.0 m");

    struct dsp_iir_filter filter;
    dsp_iir_init(&filter, -1.0f);
    zassert_within(filter.alpha, DSP_DEFAULT_IIR_ALPHA, 0.001f,
                   "Out-of-bounds alpha should fall back to default");
}
