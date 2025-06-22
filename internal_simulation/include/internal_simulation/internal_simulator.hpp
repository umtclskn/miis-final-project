#pragma once
#include "internal_simulation/potential_field.hpp"
#include <vector>
#include <array>

// GridCell ve RobotState struct'ların tanımı da burada olmalı!
struct GridCell { int i = 0; int j = 0; double cx = 0, cy = 0; };
struct RobotState { double x, y, theta; };
struct PredictedObstacle { std::string name; std::vector<std::array<double,2>> trajectory; };

class InternalSimulator {
public:
    InternalSimulator();
    double run(const RobotState& robot,
               const GridCell& target,
               const std::vector<PredictedObstacle>& obstacles,
               const std::array<double,2>& goal);

private:
    PotentialField pf_;
};
