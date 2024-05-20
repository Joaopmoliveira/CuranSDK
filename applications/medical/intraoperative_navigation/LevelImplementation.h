#ifndef CURAN_LEVEL_HEADER_FILE_
#define CURAN_LEVEL_HEADER_FILE_

#include "DicomLoading.h"

struct Level
{
    enum LevelOrientation
    {
        vertical,
        horizontal
    };

    void draw(SkCanvas *canvas, int bubble_position, LevelOrientation orientation);
};

#endif