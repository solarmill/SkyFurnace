#ifdef __cplusplus
extern "C" {
#endif

#ifndef FILTER_H
#define FILTER_H

#define BIQUAD_BANDWIDTH 1.9f     /* bandwidth in octaves */

typedef struct biquad_s {
    float a0, a1, a2, a3, a4;
    float x1, x2, y1, y2;
} biquad_t;

void BiQuadNewLpf(float filterCutFreq, biquad_t *newState, float dt);
void BiQuadReset(biquad_t *newState);
float applyBiQuadFilter(float sample, biquad_t *state);

#endif //FILTER_H

#ifdef __cplusplus
}
#endif
