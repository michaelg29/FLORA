
#ifndef __PYNQ_FINE_GRAINED_H__
#define __PYNQ_FINE_GRAINED_H__

#include "fine_grained.h"
#include "pynq.h"

class pynq_fine_grained {
public:
    finegrained_res_description fg[PYNQ_WIDTH+1] = {
        {CLB, 0, 1, CLBLM, ALIGNMENT_L},
        {CLB, 2, 3, CLBLM, ALIGNMENT_R},
        {CLB, 4, 5, CLBLM, ALIGNMENT_L},
        {CLB, 6, 7, CLBLM, ALIGNMENT_R},
        {BRAM, 0, 0, CLASS_NONE, ALIGNMENT_L},
        {CLB, 8, 9, CLBLM, ALIGNMENT_R},
        {CLB, 10, 11, CLBLM, ALIGNMENT_L},
        {DSP, 0, 0, CLASS_NONE, ALIGNMENT_R},
        {CLB, 12, 13, CLBLM, ALIGNMENT_L},
        {CLB, 14, 15, CLBLM, ALIGNMENT_R},
        {CLB, 16, 17, CLBLM, ALIGNMENT_L},
        {CLB, 18, 19, CLBLM, ALIGNMENT_R},
        {DSP, 1, 1, CLASS_NONE, ALIGNMENT_L},
        {CLB, 20, 21, CLBLM, ALIGNMENT_R},
        {CLB, 22, 23, CLBLM, ALIGNMENT_L},
        {BRAM, 1, 1, CLASS_NONE, ALIGNMENT_R},
        {CLB, 24, 25, CLBLL, ALIGNMENT_L},
        {CLB, 26, 27, CLBLM, ALIGNMENT_R},
        {CLB, 28, 29, CLBLL, ALIGNMENT_L},
        {CLB, 30, 31, CLBLM, ALIGNMENT_R},
        {BRAM, 2, 2, CLASS_NONE, ALIGNMENT_L},
        {CLB, 32, 33, CLBLM, ALIGNMENT_R},
        {CLB, 34, 35, CLBLM, ALIGNMENT_L},
        {DSP, 2, 2, CLASS_NONE, ALIGNMENT_R},
        {CLB, 36, 37, CLBLM, ALIGNMENT_L},
        {CLB, 38, 39, CLBLM, ALIGNMENT_R},
        {CLB, 40, 41, CLBLL, ALIGNMENT_L},
        {CLB, 42, 43, CLBLM, ALIGNMENT_R},
        {CLB, 44, 45, CLBLL, ALIGNMENT_L},
        {CLB, 46, 47, CLBLM, ALIGNMENT_R},
        {CLB, 48, 49, CLBLL, ALIGNMENT_L},
        {CLB, 50, 51, CLBLM, ALIGNMENT_L},
        {CLB, 52, 53, CLBLL, ALIGNMENT_R},
        {BRAM, 3, 3, CLASS_NONE, ALIGNMENT_L},
        {CLB, 54, 55, CLBLM, ALIGNMENT_R},
        {CLB, 56, 57, CLBLL, ALIGNMENT_L},
        {CLB, 58, 59, CLBLM, ALIGNMENT_R},
        {CLB, 60, 61, CLBLL, ALIGNMENT_L},
        {CLB, 62, 63, CLBLM, ALIGNMENT_R},
        {CLB, 64, 65, CLBLL, ALIGNMENT_L},
        {CLB, 66, 67, CLBLM, ALIGNMENT_R},
        {CLB, 68, 69, CLBLL, ALIGNMENT_L},
        {CLB, 70, 71, CLBLL, ALIGNMENT_R},
        {CLB, 72, 73, CLBLL, ALIGNMENT_L},
        {CLB, 74, 75, CLBLL, ALIGNMENT_R},
        {CLB, 76, 77, CLBLL, ALIGNMENT_L},
        {CLB, 78, 79, CLBLL, ALIGNMENT_R},
        {CLB, 80, 81, CLBLL, ALIGNMENT_R},
        {CLB, 82, 83, CLBLM, ALIGNMENT_L},
        {CLB, 84, 85, CLBLL, ALIGNMENT_R},
        {CLB, 86, 87, CLBLM, ALIGNMENT_L},
        {CLB, 88, 89, CLBLL, ALIGNMENT_R},
        {BRAM, 4, 4, CLASS_NONE, ALIGNMENT_L},
        {CLB, 90, 91, CLBLM, ALIGNMENT_R},
        {CLB, 92, 93, CLBLM, ALIGNMENT_L},
        {DSP, 3, 3, CLASS_NONE, ALIGNMENT_R},
        {CLB, 94, 95, CLBLM, ALIGNMENT_L},
        {CLB, 96, 97, CLBLM, ALIGNMENT_R},
        {CLB, 98, 99, CLBLM, ALIGNMENT_L},
        {CLB, 100, 101, CLBLM, ALIGNMENT_R},
        {DSP, 4, 4, CLASS_NONE, ALIGNMENT_L},
        {CLB, 102, 103, CLBLM, ALIGNMENT_R},
        {CLB, 104, 105, CLBLM, ALIGNMENT_L},
        {BRAM, 5, 5, CLASS_NONE, ALIGNMENT_R},
        {CLB, 106, 107, CLBLL, ALIGNMENT_L},
        {CLB, 108, 109, CLBLM, ALIGNMENT_R},
        {CLB, 110, 111, CLBLL, ALIGNMENT_L},
        {CLB, 112, 113, CLBLM, ALIGNMENT_R},
        {FBDN, 0, 0, CLASS_NONE, ALIGNMENT_L}
    };

    void init_fine_grained();
    pynq_fine_grained();

};

#endif // __PYNQ_FINE_GRAINED_H__

