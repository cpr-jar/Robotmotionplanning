#pragma once
#include "motion/program/robot_program.hpp"
namespace motion::program_detail {
std::vector<ProgramInstruction> expand_dxf(const ProgramInstruction &, const DxfDrawing &, const Pose &);
}
