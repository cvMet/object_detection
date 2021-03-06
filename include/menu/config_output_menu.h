#pragma once

#include <iostream>
#include <string>
#include "base_menu.h"
#include "config_kpd_menu.h"
#include "../src/objectdetection.h"
#include "../include/parameter_handler.h"

class ConfigOutputMenu : public BaseMenu
{
public:
    ConfigOutputMenu(BaseMenu* menu, ParameterHandler* param_handler)
    {
        MenuName = std::string("Output Config Menu");
        m_MenuText = std::string("S - enable/disable runtime Statistics\n")
            + "V - enable/disable Visualization\n"
            + "L - enable/disable detection Logging (detection state for each query-target pair)\n"
            + "M - enable/disable Match retrieval (#matches for each query-target pair)\n"
            + "R - Return\n"
            + "E - Execute";
        parent = menu;
        child = true;
        parameter_handler = param_handler;
    }

    BaseMenu* getNextMenu(char choice, bool& quit, bool& execute)
    {
        BaseMenu* aNewMenu = 0;

        switch (choice)
        {
        case 'S':
        {
            std::cout << "statistics state: " << std::to_string(parameter_handler->toggle_detection_stats()) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        break;
        case 'V':
        {
            std::cout << "visualization state: " << std::to_string(parameter_handler->toggle_visualization()) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        break;
        case 'L':
        {
            std::cout << "logging state: " << std::to_string(parameter_handler->toggle_detection_logging()) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        break;
        case 'M':
        {
            std::cout << "match_retrieval state: " << std::to_string(parameter_handler->toggle_match_retrieval()) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        break;
        case 'R':
        {
            aNewMenu = parent;
        }
        break;
        case 'E':
        {
            parameter_handler->set_execution_param(std::string("detection"));
            execute = true;
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