
#ifndef __ZYNQ_FINE_GRAINED_H__
#define __ZYNQ_FINE_GRAINED_H__

#include "fine_grained.h"
#include "zynq.h"

class zynq_fine_grained {
public:
    finegrained_res_description fg[ZYNQ_WIDTH+1] = {
        {CLB, 48, 48, CLBLL, ALIGNMENT_R},
        {DSP, 3, 3, CLASS_NONE, ALIGNMENT_L},
        {CLB, 51, 51, CLBLL, ALIGNMENT_R},
        {BRAM, 1, 1, CLASS_NONE, ALIGNMENT_L},
        {CLB, 52, 52, CLBLL, ALIGNMENT_R},
        {DSP, 4, 4, CLASS_NONE, ALIGNMENT_L},
        {CLB, 55, 55, CLBLL, ALIGNMENT_R},
        {DSP, 5, 5, CLASS_NONE, ALIGNMENT_L},
        {CLB, 58, 58, CLBLL, ALIGNMENT_R},
        {CLB, 59, 59, CLBLL, ALIGNMENT_L},
        {CLB, 60, 60, CLBLL, ALIGNMENT_R},
        {CLB, 62, 62, CLBLL, ALIGNMENT_R},
        {DSP, 6, 6, CLASS_NONE, ALIGNMENT_L},
        {CLB, 65, 65, CLBLL, ALIGNMENT_R},
        {CLB, 66, 66, CLBLL, ALIGNMENT_R},
        {CLB, 68, 68, CLBLL, ALIGNMENT_R},
        {BRAM, 2, 2, CLASS_NONE, ALIGNMENT_L},
        {CLB, 69, 69, CLBLL, ALIGNMENT_R},
        {DSP, 7, 7, CLASS_NONE, ALIGNMENT_L},
        {CLB, 72, 72, CLBLL, ALIGNMENT_R},
        {DSP, 8, 8, CLASS_NONE, ALIGNMENT_L},
        {CLB, 75, 75, CLBLL, ALIGNMENT_R},
        {CLB, 76, 76, CLBLL, ALIGNMENT_L},
        {CLB, 77, 77, CLBLL, ALIGNMENT_R},
        {DSP, 9, 9, CLASS_NONE, ALIGNMENT_L},
        {CLB, 80, 80, CLBLL, ALIGNMENT_R},
        {CLB, 82, 82, CLBLL, ALIGNMENT_R},
        {DSP, 10, 10, CLASS_NONE, ALIGNMENT_L},
        {CLB, 85, 85, CLBLL, ALIGNMENT_R},
        {CLB, 87, 87, CLBLL, ALIGNMENT_R},
        {DSP, 11, 11, CLASS_NONE, ALIGNMENT_L},
        {CLB, 90, 90, CLBLL, ALIGNMENT_R},
        {CLB, 91, 91, CLBLL, ALIGNMENT_L},
        {CLB, 92, 92, CLBLL, ALIGNMENT_R},
        {DSP, 12, 12, CLASS_NONE, ALIGNMENT_L},
        {CLB, 95, 95, CLBLL, ALIGNMENT_R},
        {CLB, 97, 97, CLBLL, ALIGNMENT_R},
        {BRAM, 3, 3, CLASS_NONE, ALIGNMENT_L},
        {CLB, 98, 98, CLBLL, ALIGNMENT_R},
        {CLB, 99, 99, CLBLL, ALIGNMENT_L},
        {CLB, 100, 100, CLBLL, ALIGNMENT_R},
        {CLB, 101, 101, CLBLL, ALIGNMENT_L},
        {CLB, 102, 102, CLBLL, ALIGNMENT_R},
        {BRAM, 4, 4, CLASS_NONE, ALIGNMENT_L},
        {CLB, 103, 103, CLBLL, ALIGNMENT_R},
        {CLB, 104, 104, CLBLM, ALIGNMENT_R},
        {DSP, 13, 13, CLASS_NONE, ALIGNMENT_L},
        {CLB, 105, 105, CLBLM, ALIGNMENT_R},
        {CLB, 106, 106, CLBLL, ALIGNMENT_R},
        {CLB, 107, 107, CLBLL, ALIGNMENT_L},
        {CLB, 108, 108, CLBLL, ALIGNMENT_R},
        {CLB, 109, 109, CLBLM, ALIGNMENT_R},
        {CLB, 110, 110, CLBLL, ALIGNMENT_R},
        {CLB, 111, 111, CLBLL, ALIGNMENT_L},
        {FBDN, 0, 0, CLASS_NONE, ALIGNMENT_L}
    };

    void init_fine_grained();
    zynq_fine_grained();

};

#endif // __ZYNQ_FINE_GRAINED_H__

