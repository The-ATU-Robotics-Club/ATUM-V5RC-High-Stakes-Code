/**
 * @file goalrush.hpp
 * @brief Includes the goalrush class.
 * @date 2025-01-21
 *
 * @copyright Copyright (c) 2025
 *
 */
 
#pragma once

#include "atum/atum.hpp"


namespace atum{

class goalrush{
public:


void armExtend();

void armRetract();

void grab();

void release();

void armToggle();

void rushClampToggle();

bool armExtended();

private:

 std::unique_ptr<Piston> goalRush;
 std::unique_ptr<Piston> goalRushClamp;
Logger logger;
};
}
