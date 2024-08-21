#include "include.hpp"
#include <functional>
#include <tuple>

void winPointRed();
void winPointBlue();

void ringSideRed();
void ringSideBlue();

void goalSideRed();
void goalSideBlue();

void ringElimRed();
void ringElimBlue();

void goalElimRed();
void goalElimBlue();

void skills();
void tune();

struct autonTextTuple{
    std::string autoName; 
    std::function<void()> autonomous;
};
extern autonTextTuple autos[AUTO_COUNT];