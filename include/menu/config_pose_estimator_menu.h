#pragma once

#include <iostream>
#include <string>
#include "base_menu.h"
#include "../src/objectdetection.h"

class ConfigPoseEstimatorMenu : public BaseMenu
{
public:
    ConfigPoseEstimatorMenu(BaseMenu* menu, ParameterHandler* param_handler)
    {
        MenuName = std::string("Config Pose Estimator Menu");
        m_MenuText = std::string("E - Enable/disable Pose Estimation\n")
            + "T - Transformation matrices based estimation\n"
            + "I - ISS frame based estimation\n"
            + "R - Return";
        parent = menu;
        child = true;
        parameter_handler = param_handler;
    }

    BaseMenu* getNextMenu(char choice, bool& quit, bool& execute)
    {
        BaseMenu* aNewMenu = 0;

        switch (choice)
        {
        case 'E':
        {
            std::cout << "pose estimation state: " << std::to_string(parameter_handler->toggle_pose_estimation()) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        break;
        case 'T':
        {
            std::cout << "transformation matrices based pose estimation state: " << std::to_string(parameter_handler->toggle_transformation_matrices_based_pose_estimation()) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        break;
        case 'I':
        {
            std::cout << "iss frame based pose estimation state: " << std::to_string(parameter_handler->toggle_iss_frame_based_pose_estimation()) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        break;
        case 'R':
        {
            aNewMenu = parent;
        }
        break;
        default:
        {
            // Do nothing - we won't change the menu
        }

        }
        return aNewMenu; // Sending it back to the main function
    }
};