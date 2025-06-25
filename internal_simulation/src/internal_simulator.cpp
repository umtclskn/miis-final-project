#include "internal_simulation/internal_simulator.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

// Potansiyel alan nesnesi kur
InternalSimulator::InternalSimulator() : pf_() {}

double InternalSimulator::run(
    const RobotState& robot,
    const GridCell& target,
    const std::vector<PredictedObstacle>& obstacles,
    const std::array<double, 2>& goal)
{
    // Simülasyon parametreleri
    auto start = std::chrono::steady_clock::now();
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

    std::vector<double> danger_history;  // her adım için risk skoru

    for (int t = 0; t < steps; ++t) {
        std::array<double,2> velocity;
        pf_.calculate_velocity(sim_pos, intermediate_target, obs_list, velocity, dummy_zone, dummy_min_dist);

        // Sadece pozisyonu güncelle (heading yok!)
        sim_pos[0] += velocity[0] * dt;
        sim_pos[1] += velocity[1] * dt;

        // Simülasyon sırasında riskli bölgelerde bulunma cezası
        if (dummy_zone == "Danger")
            danger_history.push_back(-1.0);
        else if (dummy_zone == "Repulsive")
            danger_history.push_back(-0.3);
        else
            danger_history.push_back(0.0);

        // Not: istersen burada collision check veya ek log ekleyebilirsin.
    }

    // 5. Skor: Simülasyon sonunda hedefe olan uzaklık (negatif, daha yakın daha iyi)
    double score = pf_.score_intermediate_target(sim_pos, goal, danger_history);

    // ZAMAN ÖLÇÜMÜ BİTİR
    auto end = std::chrono::steady_clock::now();
    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    // LOG
    RCLCPP_INFO(rclcpp::get_logger("InternalSimulator"),
        "Internal simulation for grid (%d,%d): score=%.5f, time=%ld ms",
        target.i, target.j, score, duration_ms);        
    return score;
}
