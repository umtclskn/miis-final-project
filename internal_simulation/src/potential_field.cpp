#include "internal_simulation/potential_field.hpp"
#include <cmath>
#include <algorithm>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 (M_PI / 2.0)
#endif

PotentialField::PotentialField()
    : R_tan_(1.0), R_rep_(0.5), R_danger_(0.08), R_max_(1.5),
      S_fast_(1.0), S_middle_(0.5), S_slow_(0.25),
      S_att_max_(1.0), R_rob2tar_(0.5), V_ref_(0.3)
{}

void PotentialField::attractive_motion(const std::array<double,2>& robot_pos,
                                       const std::array<double,2>& target_pos,
                                       std::array<double,2>& v_att_hat, double& s_att) const
{
    double dx = target_pos[0] - robot_pos[0];
    double dy = target_pos[1] - robot_pos[1];
    double dist = std::hypot(dx, dy);
    if (dist < 1e-2) { v_att_hat = {0,0}; s_att = 0.0; return; }
    v_att_hat = {dx / dist, dy / dist};
    if (dist <= R_rob2tar_)
        s_att = S_att_max_ * (1.0 - std::cos(M_PI_2 * dist / R_rob2tar_));
    else
        s_att = S_att_max_;
}

void PotentialField::repulsive_motion(const std::array<double,2>& robot_pos,
                                      const std::array<double,2>& obstacle_pos,
                                      std::array<double,2>& v_rep_hat) const
{
    double dx = robot_pos[0] - obstacle_pos[0];
    double dy = robot_pos[1] - obstacle_pos[1];
    double dist = std::hypot(dx, dy);
    if (dist < 1e-2) { v_rep_hat = {0,0}; return; }
    v_rep_hat = {dx / dist, dy / dist};
}

void PotentialField::tangent_motion(const std::array<double,2>& robot_pos,
                                    const std::array<double,2>& target_pos,
                                    const std::array<double,2>& obs_pos,
                                    const std::array<double,2>& obs_vel,
                                    std::array<double,2>& v_tan_hat) const
{
    double dx = target_pos[0] - robot_pos[0];
    double dy = target_pos[1] - robot_pos[1];
    double d_tar = std::hypot(dx, dy);
    std::array<double,2> target_dir = {0, 0};
    if (d_tar > 1e-3)
        target_dir = {dx / d_tar, dy / d_tar};

    double obs_speed = std::hypot(obs_vel[0], obs_vel[1]);
    bool C_dynamic = obs_speed >= V_ref_;
    double rx = obs_pos[0] - robot_pos[0];
    double ry = obs_pos[1] - robot_pos[1];
    double cross = dx * ry - dy * rx;
    bool C_disturb = std::abs(cross) < 0.3 * d_tar;

    if (C_dynamic && C_disturb) {
        if (obs_speed > 1e-3)
            v_tan_hat = {-obs_vel[0] / obs_speed, -obs_vel[1] / obs_speed};
        else
            v_tan_hat = target_dir;
    } else {
        v_tan_hat = target_dir;
    }
}

double PotentialField::calculate_s_max(double d_min) const
{
    if (d_min > R_max_)
        return S_fast_;
    else if (d_min >= R_tan_ && d_min <= R_max_)
        return (S_fast_ - S_middle_) / (R_max_ - R_tan_) * (d_min - R_max_) + S_fast_;
    else if (d_min >= R_rep_ && d_min < R_tan_) {
        double cos_term = std::cos(M_PI_2 * (d_min - R_rep_) / (R_tan_ - R_rep_));
        return (S_middle_ - S_slow_) * (1 - cos_term) + S_slow_;
    }
    else if (d_min > R_danger_ && d_min < R_rep_)
        return S_slow_;
    else
        return 0.0;
}

void PotentialField::calculate_velocity(
    const std::array<double,2>& robot_pos,
    const std::array<double,2>& target_pos,
    const std::vector<ObstacleInfo>& obstacles,
    std::array<double,2>& velocity,
    std::string& zone,
    double& min_dist
) const
{
    // --- Attractive motion
    std::array<double,2> v_att_hat = {0,0};
    double s_att = 0.0;
    attractive_motion(robot_pos, target_pos, v_att_hat, s_att);

    // --- Closest obstacle
    min_dist = 1e9;
    ObstacleInfo closest_obs;
    bool has_obs = false;
    for (const auto& obs : obstacles) {
        double dist = std::hypot(robot_pos[0] - obs.pos[0], robot_pos[1] - obs.pos[1]);
        if (dist < min_dist) {
            min_dist = dist;
            closest_obs = obs;
            has_obs = true;
        }
    }

    double s_max = calculate_s_max(min_dist);

    if (!has_obs || min_dist > R_tan_) {
        // Free zone (Eq.8)
        zone = "Free";
        velocity[0] = v_att_hat[0] * std::min(s_att, s_max);
        velocity[1] = v_att_hat[1] * std::min(s_att, s_max);
    }
    else if (min_dist > R_rep_ && min_dist <= R_tan_) {
        // Precautionary zone (Eq.9)
        zone = "Precautionary";
        std::array<double,2> v_tan_hat;
        tangent_motion(robot_pos, target_pos, closest_obs.pos, closest_obs.vel, v_tan_hat);

        double dot = v_tan_hat[0]*v_att_hat[0] + v_tan_hat[1]*v_att_hat[1];
        if (std::abs(dot-1.0) < 0.1) {
            velocity[0] = v_tan_hat[0] * std::min(s_att, s_max);
            velocity[1] = v_tan_hat[1] * std::min(s_att, s_max);
        } else {
            velocity[0] = v_tan_hat[0] * s_max;
            velocity[1] = v_tan_hat[1] * s_max;
        }
    }
    else if (min_dist > R_danger_ && min_dist <= R_rep_) {
        // Repulsive zone (Eq.10)
        zone = "Repulsive";
        std::array<double,2> v_rep_hat, v_tan_hat;
        repulsive_motion(robot_pos, closest_obs.pos, v_rep_hat);
        tangent_motion(robot_pos, target_pos, closest_obs.pos, closest_obs.vel, v_tan_hat);

        double dot = v_tan_hat[0]*v_att_hat[0] + v_tan_hat[1]*v_att_hat[1];
        std::array<double,2> move;
        if (std::abs(dot-1.0) < 0.1) {
            move = v_tan_hat;
        } else {
            move = {v_tan_hat[0] + v_rep_hat[0], v_tan_hat[1] + v_rep_hat[1]};
            double norm = std::hypot(move[0], move[1]);
            if (norm > 1e-3) {
                move[0] /= norm;
                move[1] /= norm;
            } else {
                move = v_rep_hat;
            }
        }
        velocity[0] = move[0] * s_max;
        velocity[1] = move[1] * s_max;
    }
    else {
        // Danger zone (R_danger_)
        zone = "Danger";
        velocity[0] = 0.0;
        velocity[1] = 0.0;
    }
}

double PotentialField::score_intermediate_target(
    const std::array<double,2>& simulated_final_pos,
    const std::array<double,2>& goal_pos
) const
{
    double d = std::hypot(simulated_final_pos[0] - goal_pos[0], simulated_final_pos[1] - goal_pos[1]);
    return -d;
}
