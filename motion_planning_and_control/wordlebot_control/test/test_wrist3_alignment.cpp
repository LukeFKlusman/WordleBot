#include <cmath>
#include <iostream>

#include "wordlebot_control/wordle_bot_controller.hpp"

namespace
{
bool near(double lhs, double rhs, double tolerance)
{
  return std::abs(lhs - rhs) <= tolerance;
}

bool expectNear(const char * label, double lhs, double rhs, double tolerance)
{
  if (near(lhs, rhs, tolerance)) {
    return true;
  }
  std::cerr << label << " failed: got " << lhs << ", expected " << rhs
            << " +/- " << tolerance << '\n';
  return false;
}
}  // namespace

int main()
{
  bool ok = true;

  const double startup_offset = WordleBotController::computeContinuousJointRevolutionOffset(
    -2.0 * M_PI, 0.0);
  ok &= expectNear("startup -2pi offset", startup_offset, -2.0 * M_PI, 1e-9);
  ok &= std::abs((0.0 + startup_offset) - (-2.0 * M_PI)) < 0.2;

  const double aligned_offset = WordleBotController::computeContinuousJointRevolutionOffset(
    0.01, 0.0);
  ok &= expectNear("already aligned offset", aligned_offset, 0.0, 1e-9);

  const double first_aligned = 0.0 + startup_offset;
  const double final_aligned = 4.0 * M_PI + startup_offset;
  ok &= expectNear("multi-revolution delta preserved",
    final_aligned - first_aligned, 4.0 * M_PI, 1e-9);

  return ok ? 0 : 1;
}
