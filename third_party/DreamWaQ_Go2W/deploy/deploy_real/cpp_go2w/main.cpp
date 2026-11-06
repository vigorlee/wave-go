/**
 * @file main.cpp
 * @brief GO2W DreamWaQ RL Deployment
 * 
 * Usage: ./go2w_dwaq <network_interface>
 * Control flow: zero_torque -> Start -> default_pos -> A -> RL control -> Select to exit
 */

#include <iostream>
#include <csignal>
#include "Controller.h"

Controller* g_controller = nullptr;

// Signal handler for safe exit (Ctrl+C)
void signalHandler(int signum)
{
    std::cout << "\nInterrupt - entering damp mode" << std::endl;
    if (g_controller) g_controller->damp();
    exit(signum);
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cout << "Usage: " << argv[0] << " <network_interface>" << std::endl;
        return 1;
    }

    signal(SIGINT, signalHandler);

    Controller controller(argv[1]);
    g_controller = &controller;

    controller.zero_torque_state();
    controller.move_to_default_pos();
    controller.default_pos_state();

    std::cout << "RL Started - Press Select to exit" << std::endl;
    
    while (!controller.isSelectPressed())
    {
        controller.run();
    }
    
    controller.damp();
    return 0;
}
