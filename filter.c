#include <stdlib.h>
#include "filter.h"
#include <math.h>

/* sets up a biquad Filter */
void BiQuadNewLpf(float filterCutFreq, biquad_t *newState, float dt)
{
    float omega, sn, cs, alpha;
    float a0, a1, a2, b0, b1, b2;

    if (filterCutFreq < 0.005) {
        newState->a0 = 1.0f;
        newState->a1 = newState->a2 = newState->a3 = newState->a4 = 0.0f;
    }

    else {
        /* setup variables */
        omega = 2 * (float) M_PI * (float) filterCutFreq * dt;
        sn = sinf(omega);
        cs = cosf(omega);
        alpha = sn * sinf((float) M_LN2 / 2 * BIQUAD_BANDWIDTH * omega / sn);

        b0 = (1 - cs) / 2;
        b1 = 1 - cs;
        b2 = (1 - cs) / 2;
        a0 = 1 + alpha;
        a1 = -2 * cs;
        a2 = 1 - alpha;

        /* precompute the coefficients */
        newState->a0 = b0 / a0;
        newState->a1 = b1 / a0;
        newState->a2 = b2 / a0;
        newState->a3 = a1 / a0;
        newState->a4 = a2 / a0;
    }
}

void BiQuadReset(biquad_t *newState) {
    /* zero initial samples */
    newState->x1 = newState->x2 = 0;
    newState->y1 = newState->y2 = 0;
}

/* Computes a biquad_t filter on a sample */
float applyBiQuadFilter(float sample, biquad_t *state)
{
    float result;

    /* compute result */
    result = state->a0 * sample + state->a1 * state->x1 + state->a2 * state->x2 -
             state->a3 * state->y1 - state->a4 * state->y2;

    /* shift x1 to x2, sample to x1 */
    state->x2 = state->x1;
    state->x1 = sample;

    /* shift y1 to y2, result to y1 */
    state->y2 = state->y1;
    state->y1 = result;

    return result;
}


