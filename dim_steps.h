
#include "types.h"

typedef struct {
    byte start;
    byte stop;
    } sDimChanStep;

typedef struct {
    byte ticks;
    sDimChanStep Chan[8];
    } sDimStep;

// sDimStep *ptrDimSequences[16];
// word DimSequenceLengths[16];
