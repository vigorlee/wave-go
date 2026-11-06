/**
 * @file benchmark_main.cpp
 * @brief GO2W C++ Benchmark - 带细粒度计时
 * 
 * 使用本地修改的 Controller.cpp（含计时代码）
 * 测量项目与 Python 版本完全匹配
 */

#include <iostream>
#include <csignal>
#include <chrono>
#include <getopt.h>
#include "Controller.h"
#include "benchmark_timer.h"

// 全局性能监控器（Controller.cpp 中使用 extern 引用）
benchmark::PerformanceMonitor g_monitor;

Controller* g_controller = nullptr;
int g_duration_seconds = 60;
std::string g_output_path = "";

void signalHandler(int signum)
{
    std::cout << "\nInterrupt - saving results and entering damp mode" << std::endl;
    
    g_monitor.printSummary();
    std::string output = "/home/jackie/DreamWaQ_Go2W/deploy/deploy_real/performance_benchmark/results/cpp_benchmark.json";
    g_monitor.saveResults(output);
    
    if (g_controller) g_controller->damp();
    exit(signum);
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <network_interface> [duration_seconds]" << std::endl;
        return 1;
    }
    
    std::string net_interface = argv[1];
    if (argc >= 3) {
        g_duration_seconds = std::atoi(argv[2]);
    }

    std::cout << "GO2W C++ Benchmark (Fine-Grained Timing)\n";
    std::cout << "Network: " << net_interface << ", Duration: " << g_duration_seconds << "s\n";
    std::cout << "Make sure robot is hung up or on ground. Press Enter..." << std::endl;
    std::cin.ignore();

    signal(SIGINT, signalHandler);

    Controller controller(net_interface);
    g_controller = &controller;

    controller.zero_torque_state();
    controller.move_to_default_pos();
    controller.default_pos_state();

    std::cout << "RL Benchmark Started - Press Select to exit" << std::endl;
    
    auto start_time = std::chrono::steady_clock::now();
    int loop_count = 0;
    
    while (!controller.isSelectPressed())
    {
        // 计时在 Controller::run() 内部完成
        controller.run();
        loop_count++;
        
        // 每 500 次显示进度
        if (loop_count % 500 == 0) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
            std::cout << "[Benchmark] " << elapsed << "/" << g_duration_seconds 
                      << "s, Loops: " << loop_count << std::endl;
        }
        
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
        if (elapsed >= g_duration_seconds) {
            std::cout << "\nBenchmark completed (" << g_duration_seconds << "s)" << std::endl;
            break;
        }
    }
    
    g_monitor.printSummary();
    std::string output = "/home/jackie/DreamWaQ_Go2W/deploy/deploy_real/performance_benchmark/results/cpp_benchmark.json";
    g_monitor.saveResults(output);

    controller.damp();
    return 0;
}
