#pragma once
#include <array>
#include <vector>
#include <string>

// Obstacle yapısı
struct ObstacleInfo {
    std::array<double, 2> pos;
    std::array<double, 2> vel;
};

class PotentialField {
public:
    PotentialField();

    void attractive_motion(const std::array<double,2>& robot_pos,
                           const std::array<double,2>& target_pos,
                           std::array<double,2>& v_att_hat, double& s_att) const;

    void repulsive_motion(const std::array<double,2>& robot_pos,
                          const std::array<double,2>& obstacle_pos,
                          std::array<double,2>& v_rep_hat) const;

    void tangent_motion(const std::array<double,2>& robot_pos,
                        const std::array<double,2>& target_pos,
                        const std::array<double,2>& obs_pos,
                        const std::array<double,2>& obs_vel,
                        std::array<double,2>& v_tan_hat) const;

    double calculate_s_max(double d_min) const;

    void calculate_velocity(const std::array<double,2>& robot_pos,
                            const std::array<double,2>& target_pos,
                            const std::vector<ObstacleInfo>& obstacles,
                            std::array<double,2>& velocity,
                            std::string& zone,
                            double& min_dist) const;

    double score_intermediate_target(
        const std::array<double,2>& sim_pos,
        const std::array<double,2>& goal_pos,
        const std::vector<double>& danger_history
    ) const;

private:
    double R_tan_, R_rep_, R_danger_, R_max_;
    double S_fast_, S_middle_, S_slow_;
    double S_att_max_, R_rob2tar_, V_ref_;
};
