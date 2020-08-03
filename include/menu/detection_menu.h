#pragma once

#include <iostream>
#include <string>
#include "base_menu.h"
#include "config_kpd_menu.h"
#include "config_normal_estimator_menu.h"
#include "config_output_menu.h"
#include "config_matcher_menu.h"
#include "config_pose_estimator_menu.h"
#include "config_input_menu.h"
#include "../src/objectdetection.h"
#include "../include/parameter_handler.h"

class DetectionMenu : public BaseMenu
{
public:
    DetectionMenu(BaseMenu* menu, ParameterHandler* param_handler)
    {
        MenuName = std::string("Detection Menu");
        m_MenuText = std::string("T - set detection Threshold\n")
            + "I - configure Input\n"
            + "N - configure Normal estimator\n"
            + "K - configure KPD\n"
            + "M - configure Matcher\n"
            + "O - configure Output\n"
            + "P - configure Pose estimator\n"
            + "E - Execute\n"
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
        case 'T':
        {
            parameter_handler->set_detection_threshold(std::stoi(get_input()));
        }
        break;
        case 'I':
        {
            aNewMenu = new ConfigInputMenu(this, parameter_handler);
        }
        break;
        case 'N':
        {
            aNewMenu = new ConfigNEMenu(this, parameter_handler);
        }
        break;
        case 'K':
        {
            aNewMenu = new ConfigKPDMenu(this, parameter_handler);
        }
        break;
        case 'M':
        {
            aNewMenu = new ConfigMatcherMenu(this, parameter_handler);
        }
        break;
        case 'O':
        {
            aNewMenu = new ConfigOutputMenu(this, parameter_handler);
        }
        break;
        case 'P':
        {
            aNewMenu = new ConfigPoseEstimatorMenu(this, parameter_handler);
        }
        break;
        case 'E':
        {
            parameter_handler->set_execution_param(std::string("detection"));
            execute = true;
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