#pragma once
#include "Line.h"
#include <fstream>
#include <iostream>
#include <vsgParticleSystem/dynamics/Dynamics.h>

std::vector<Line> getData();

// https://stackoverflow.com/a/14539953
inline void progressBar(float progress) {
    int barwidth = 70;
    std::cout << "[";
    int pos = barwidth * progress;
    for (int i = 0; i < barwidth; i++) {
        if (i < pos)
            std::cout << "=";
        else if (i == pos)
            std::cout << ">";
        else
            std::cout << " ";
    }
    std::cout << "]" << int(progress * 100) << "%\r";
    std::cout.flush();
}
