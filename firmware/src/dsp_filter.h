#ifndef ZEPHYRAIR_DSP_FILTER_H_
#define ZEPHYRAIR_DSP_FILTER_H_

#include <stdbool.h>

#define DSP_SEA_LEVEL_PRESSURE_PA 101325.0f
#define DSP_DEFAULT_IIR_ALPHA     0.15f

/* First-order IIR low-pass filter state: y[n] = (1 - alpha) * y[n-1] + alpha * x[n] */
struct dsp_iir_filter {
    float alpha;
    float state;
    bool  initialized;
};

void dsp_iir_init(struct dsp_iir_filter *filter, float alpha);
float dsp_iir_update(struct dsp_iir_filter *filter, float sample);
float dsp_calculate_altitude(float pressure_pa);

#endif /* ZEPHYRAIR_DSP_FILTER_H_ */
