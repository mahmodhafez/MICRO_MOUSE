#include <iostream>
#include "FloodFill.h"

int main(int argc, char *argv[])
{
    std::cerr << "[Main] Starting Micromouse Flood Fill..." << std::endl;

    FloodFill mouse;
    mouse.init();

    while (!mouse.isAtGoal())
    {
        if (!mouse.step())
        {
            break;
        }
    }

    if (mouse.isAtGoal())
    {
        mouse.highlightShortestPath();
        std::cerr << "[Main] Successfully reached the center goal at ("
                  << mouse.getX() << ", " << mouse.getY() << ")!" << std::endl;
    }
    else
    {
        std::cerr << "[Main] Exploration stopped." << std::endl;
    }

    return 0;
}
