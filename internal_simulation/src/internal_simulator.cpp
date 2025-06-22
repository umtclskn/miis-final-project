#include "internal_simulation/internal_simulator.hpp"
#include "rclcpp/rclcpp.hpp"

// Potansiyel alan nesnesi kur
InternalSimulator::InternalSimulator() : pf_() {}

double InternalSimulator::run(
    const RobotState& robot,
    const GridCell& target,
    const std::vector<PredictedObstacle>& obstacles,
    const std::array<double, 2>& goal)
{
    // Simülasyon parametreleri
    constexpr int steps = 100;
    constexpr double dt = 0.05;

    // 1. Başlangıç pozisyonu (sadece x, y) - heading kullanılmayacak!
    std::array<double,2> sim_pos = {robot.x, robot.y};

    // 2. Engellerin ilk pozisyon ve hızları ObstacleInfo'ya çevrilir
    std::vector<ObstacleInfo> obs_list;
    for (const auto& obs : obstacles) {
        if (obs.trajectory.empty()) continue;
        ObstacleInfo oinfo;
        oinfo.pos = obs.trajectory[0];
        // İlk iki nokta varsa, hız vektörü hesaplanır
        if (obs.trajectory.size() >= 2) {
            double vx = (obs.trajectory[1][0] - obs.trajectory[0][0]) / dt;
            double vy = (obs.trajectory[1][1] - obs.trajectory[0][1]) / dt;
            oinfo.vel = {vx, vy};
        } else {
            oinfo.vel = {0.0, 0.0};
        }
        obs_list.push_back(oinfo);
    }

    // 3. Ara hedef (intermediate grid)
    std::array<double,2> intermediate_target = {target.cx, target.cy};

    // 4. Her iterasyonda potansiyel alanın önerdiği velocity ile pozisyonu kaydır!
    std::string dummy_zone;
    double dummy_min_dist;

    for (int t = 0; t < steps; ++t) {
        std::array<double,2> velocity;
        pf_.calculate_velocity(sim_pos, intermediate_target, obs_list, velocity, dummy_zone, dummy_min_dist);

        // Sadece pozisyonu güncelle (heading yok!)
        sim_pos[0] += velocity[0] * dt;
        sim_pos[1] += velocity[1] * dt;

        // Not: istersen burada collision check veya ek log ekleyebilirsin.
    }

    // 5. Skor: Simülasyon sonunda hedefe olan uzaklık (negatif, daha yakın daha iyi)
    double score = pf_.score_intermediate_target(sim_pos, goal);

    // 6. Log (isteğe bağlı)
    RCLCPP_INFO(rclcpp::get_logger("InternalSimulator"),
        "Cell (%d,%d) → sim_final (%.2f, %.2f), goal (%.2f, %.2f), score=%.2f",
        target.i, target.j, sim_pos[0], sim_pos[1], goal[0], goal[1], score);

    return score;
}
