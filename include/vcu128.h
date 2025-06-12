#ifndef VCU118_DEF_H
#define VCU118_DEF_H

#include "fpga.h"

// TODO update numbers
#define VCU128_CLK_REG 96 // X0Y0 to X7Y11
#define VCU128_WIDTH 278
#define VCU128_NUM_ROWS 12 // number of clock regions on the y-coordinate
#define VCU128_FORBIDDEN 3
#define VCU128_CLB_PER_TILE 60
#define VCU128_BRAM_PER_TILE 12 // BRAM36
#define VCU128_DSP_PER_TILE 18 // 18 for clock region Y0, 24 for all others
#define VCU128_CLB_TOT  180960
#define VCU128_BRAM_TOT  4368
#define VCU128_DSP_TOT  9792

class vcu128
{
public:
    int clb_per_tile  = VCU128_CLB_PER_TILE;
    int bram_per_tile = VCU128_BRAM_PER_TILE;
    int dsp_per_tile  = VCU128_DSP_PER_TILE;
    int num_clk_reg   = VCU128_CLK_REG; // H
    unsigned long width  = VCU128_WIDTH; // W
//    void initialize_clk_reg();
    vcu128();
};
#endif

