/**
 * @file benchmark_timer.h
 * @brief C++ 高精度性能计时工具（简化版 - 无资源监控）
 */

#ifndef BENCHMARK_TIMER_H
#define BENCHMARK_TIMER_H

#include <chrono>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace benchmark {

struct TimingStats {
    std::vector<double> samples;
    
    void add(double value) { samples.push_back(value); }
    
    double mean() const {
        if (samples.empty()) return 0;
        double sum = 0;
        for (double v : samples) sum += v;
        return sum / samples.size();
    }
    
    double stddev() const {
        if (samples.size() < 2) return 0;
        double m = mean();
        double sum = 0;
        for (double v : samples) sum += (v - m) * (v - m);
        return std::sqrt(sum / (samples.size() - 1));
    }
    
    double min() const {
        return samples.empty() ? 0 : *std::min_element(samples.begin(), samples.end());
    }
    
    double max() const {
        return samples.empty() ? 0 : *std::max_element(samples.begin(), samples.end());
    }
    
    double percentile(double p) const {
        if (samples.empty()) return 0;
        std::vector<double> sorted = samples;
        std::sort(sorted.begin(), sorted.end());
        size_t idx = static_cast<size_t>(p / 100.0 * (sorted.size() - 1));
        return sorted[idx];
    }
    
    size_t count() const { return samples.size(); }
};

struct ControlQualityMetrics {
    std::vector<std::vector<float>> target_positions;
    std::vector<std::vector<float>> actual_positions;
    
    void addSample(const float* target, const float* actual, int size) {
        target_positions.push_back(std::vector<float>(target, target + size));
        actual_positions.push_back(std::vector<float>(actual, actual + size));
    }
    
    double getMeanError() const {
        if (target_positions.empty()) return 0;
        double total_error = 0;
        int count = 0;
        for (size_t i = 0; i < target_positions.size(); i++) {
            for (size_t j = 0; j < target_positions[i].size(); j++) {
                total_error += std::abs(target_positions[i][j] - actual_positions[i][j]);
                count++;
            }
        }
        return count > 0 ? total_error / count : 0;
    }
    
    double getMaxError() const {
        double max_err = 0;
        for (size_t i = 0; i < target_positions.size(); i++) {
            for (size_t j = 0; j < target_positions[i].size(); j++) {
                double err = std::abs(target_positions[i][j] - actual_positions[i][j]);
                max_err = std::max(max_err, err);
            }
        }
        return max_err;
    }
};

class PerformanceMonitor {
public:
    // ========== 计时方法 ==========
    void startLoop() { loop_start_ = std::chrono::high_resolution_clock::now(); }
    void endLoop() {
        auto end = std::chrono::high_resolution_clock::now();
        loop_time_.add(std::chrono::duration<double, std::milli>(end - loop_start_).count());
    }
    
    void startInference() { inference_start_ = std::chrono::high_resolution_clock::now(); }
    void endInference() {
        auto end = std::chrono::high_resolution_clock::now();
        inference_time_.add(std::chrono::duration<double, std::milli>(end - inference_start_).count());
    }
    
    void startEncoder() { encoder_start_ = std::chrono::high_resolution_clock::now(); }
    void endEncoder() {
        auto end = std::chrono::high_resolution_clock::now();
        encoder_time_.add(std::chrono::duration<double, std::milli>(end - encoder_start_).count());
    }
    
    void startActor() { actor_start_ = std::chrono::high_resolution_clock::now(); }
    void endActor() {
        auto end = std::chrono::high_resolution_clock::now();
        actor_time_.add(std::chrono::duration<double, std::milli>(end - actor_start_).count());
    }
    
    void startCommSend() { comm_start_ = std::chrono::high_resolution_clock::now(); }
    void endCommSend() {
        auto end = std::chrono::high_resolution_clock::now();
        comm_send_time_.add(std::chrono::duration<double, std::milli>(end - comm_start_).count());
    }
    
    void recordControlQuality(const float* target, const float* actual, int size = 12) {
        control_quality_.addSample(target, actual, size);
    }
    
    // 保留空方法以保持兼容性
    void recordResourceUsage() {}
    
    // ========== 结果输出 ==========
    void printSummary() const {
        std::cout << "\n" << std::string(50, '=') << std::endl;
        std::cout << "C++ Performance Benchmark Results" << std::endl;
        std::cout << std::string(50, '=') << std::endl;
        std::cout << std::fixed << std::setprecision(3);
        
        std::cout << "\n[Timing (ms)]" << std::endl;
        std::cout << "  Loop Time:      mean=" << loop_time_.mean() 
                  << ", max=" << loop_time_.max()
                  << ", 99th=" << loop_time_.percentile(99) << std::endl;
        std::cout << "  Inference:      mean=" << inference_time_.mean()
                  << ", max=" << inference_time_.max() << std::endl;
        std::cout << "    - Encoder:    mean=" << encoder_time_.mean()
                  << ", max=" << encoder_time_.max() << std::endl;
        std::cout << "    - Actor:      mean=" << actor_time_.mean()
                  << ", max=" << actor_time_.max() << std::endl;
        
        std::cout << std::setprecision(6);
        std::cout << "\n[Control Quality]" << std::endl;
        std::cout << "  Mean Tracking Error: " << control_quality_.getMeanError() << " rad" << std::endl;
        std::cout << "  Max Tracking Error:  " << control_quality_.getMaxError() << " rad" << std::endl;
        
        std::cout << "\nTotal samples: " << loop_time_.count() << std::endl;
        std::cout << std::string(50, '=') << std::endl;
    }
    
    void saveResults(const std::string& filepath) const {
        std::ofstream f(filepath);
        f << std::fixed;
        
        f << "{\n";
        f << "  \"timing\": {\n";
        f << "    \"loop_time_ms\": {\"mean\": " << loop_time_.mean() 
          << ", \"std\": " << loop_time_.stddev()
          << ", \"min\": " << loop_time_.min()
          << ", \"max\": " << loop_time_.max()
          << ", \"percentile_99\": " << loop_time_.percentile(99)
          << ", \"count\": " << loop_time_.count() << "},\n";
        f << "    \"inference_time_ms\": {\"mean\": " << inference_time_.mean()
          << ", \"std\": " << inference_time_.stddev()
          << ", \"max\": " << inference_time_.max() << "},\n";
        f << "    \"encoder_time_ms\": {\"mean\": " << encoder_time_.mean()
          << ", \"max\": " << encoder_time_.max() << "},\n";
        f << "    \"actor_time_ms\": {\"mean\": " << actor_time_.mean()
          << ", \"max\": " << actor_time_.max() << "}\n";
        f << "  },\n";
        
        f << "  \"control_quality\": {\n";
        f << std::setprecision(8);
        f << "    \"mean_error\": " << control_quality_.getMeanError() << ",\n";
        f << "    \"max_error\": " << control_quality_.getMaxError() << "\n";
        f << "  },\n";
        
        f << "  \"metadata\": {\n";
        f << "    \"language\": \"cpp\",\n";
        f << "    \"total_samples\": " << loop_time_.count() << "\n";
        f << "  }\n";
        f << "}\n";
        
        f.close();
        std::cout << "Results saved to " << filepath << std::endl;
    }

private:
    TimingStats loop_time_, inference_time_, encoder_time_, actor_time_, comm_send_time_;
    ControlQualityMetrics control_quality_;
    
    std::chrono::high_resolution_clock::time_point loop_start_;
    std::chrono::high_resolution_clock::time_point inference_start_;
    std::chrono::high_resolution_clock::time_point encoder_start_;
    std::chrono::high_resolution_clock::time_point actor_start_;
    std::chrono::high_resolution_clock::time_point comm_start_;
};

} // namespace benchmark

#endif // BENCHMARK_TIMER_H
