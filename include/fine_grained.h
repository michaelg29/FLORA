#ifndef FINE_GRAINED_H
#define FINE_GRAINED_H
#include "fpga.h"

#ifndef FBDN
#define FBDN         3
#define CENTRAL_CLK  4
#endif

#ifndef CLB
#define CLB          0
#define BRAM         1
#define DSP          2

#define CLASS_NONE   0
#define CLBLM        1
#define CLBLL        2

#define ALIGNMENT_L  0
#define ALIGNMENT_R  1
#endif

typedef struct {
    int type_of_res;
    int slice_1;
    int slice_2;
    int class_of_res;
    int alignment_of_res;
} finegrained_res_description;

typedef struct {
    unsigned long slice_x1;
    unsigned long slice_x2;
    unsigned long slice_y1;
    unsigned long slice_y2;
} slice_address;

typedef struct {
    slice_address addresses[4];
} slices;

#endif // FINE_GRAINED_H
