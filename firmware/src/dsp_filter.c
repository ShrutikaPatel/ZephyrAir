#include "dsp_filter.h"
#include <math.h>
#include <stddef.h>

void dsp_iir_init(struct dsp_iir_filter *filter, float alpha)
{
    if (filter == NULL) {
        return;
    }

    if (alpha <= 0.0f || alpha > 1.0f) {
        filter->alpha = DSP_DEFAULT_IIR_ALPHA;
    } else {
        filter->alpha = alpha;
    }

    filter->state = 0.0f;
    filter->initialized = false;
}

float dsp_iir_update(struct dsp_iir_filter *filter, float sample)
{
    if (filter == NULL) {
        return sample;
    }

    if (!filter->initialized) {
        filter->state = sample;
        filter->initialized = true;
    } else {
        /* y[n] = (1 - alpha) * y[n-1] + alpha * x[n] */
        filter->state = ((1.0f - filter->alpha) * filter->state) + (filter->alpha * sample);
    }

    return filter->state;
}

float dsp_calculate_altitude(float pressure_pa)
{
    if (pressure_pa <= 0.0f) {
        return 0.0f;
    }

    return 44330.0f * (1.0f - powf(pressure_pa / DSP_SEA_LEVEL_PRESSURE_PA, 0.1903f));
}
