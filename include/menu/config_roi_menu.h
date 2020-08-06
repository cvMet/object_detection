#pragma once

#include <iostream>
#include <string>
#include "base_menu.h"
#include "../src/objectdetection.h"

class ConfigROIMenu : public BaseMenu
{
    bool state = false;
public:
    ConfigROIMenu(BaseMenu* menu, ParameterHandler* param_handler)
    {
        MenuName = std::string("Config Region of Interest Filter Menu");
        m_MenuText = std::string("E - Enable/disable ROI filter\n")
            + "X - set x-limit size (default = 0.2 [m])\n"
            + "Y - set y-limit size (default = 0.2 [m])\n"
            + "Z - set z-limit size (default = 0.2 [m])\n"
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
            state = parameter_handler->toggle_filter(std::string("roi"));
            std::cout << "roi filter state: " << std::to_string(state) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        break;
        case 'X':
        {
            parameter_handler->set_roi_x_limit(std::stof(get_input().substr(0, 5)));
        }
        break;
        case 'Y':
        {
            parameter_handler->set_roi_y_limit(std::stof(get_input().substr(0, 5)));
        }
        break;
        case 'Z':
        {
            parameter_handler->set_roi_z_limit(std::stof(get_input().substr(0, 5)));
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