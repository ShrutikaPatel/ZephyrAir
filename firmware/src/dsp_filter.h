#ifndef ZEPHYRAIR_DSP_FILTER_H_
#define ZEPHYRAIR_DSP_FILTER_H_

#include <stdbool.h>

/** Standard sea-level atmospheric pressure in Pascals (Pa) */
#define DSP_SEA_LEVEL_PRESSURE_PA 101325.0f

/** Default IIR filter smoothing factor alpha (0.15 = 15% new sample, 85% history) */
#define DSP_DEFAULT_IIR_ALPHA     0.15f

/**
 * @brief State structure for a 1st-order IIR low-pass filter:
 *        y[n] = (1 - alpha) * y[n-1] + alpha * x[n]
 */
struct dsp_iir_filter {
    float alpha;
    float state;
    bool  initialized;
};

/**
 * @brief Initialize (or reset) a 1st-order IIR low-pass filter.
 *
 * @param filter Pointer to the filter state struct.
 * @param alpha  Smoothing factor in (0.0f, 1.0f]. If outside bounds, defaults to 0.15f.
 */
void dsp_iir_init(struct dsp_iir_filter *filter, float alpha);

/**
 * @brief Feed a new sample into the 1st-order IIR filter and return the filtered value.
 *
 * On the very first sample after initialization, the filter state snaps directly
 * to the input sample so there is no artificial startup ramp.
 *
 * @param filter Pointer to the filter state struct.
 * @param sample New raw measurement (e.g., pressure in Pa).
 * @return Smoothed output value.
 */
float dsp_iir_update(struct dsp_iir_filter *filter, float sample);

/**
 * @brief Calculate barometric altitude (in meters above sea level) from pressure.
 *
 * Uses the international barometric formula:
 *     h = 44330 * (1 - (P / P0)^0.1903)
 *
 * @param pressure_pa Measured atmospheric pressure in Pascals.
 * @return Estimated altitude in meters.
 */
float dsp_calculate_altitude(float pressure_pa);

#endif /* ZEPHYRAIR_DSP_FILTER_H_ */
