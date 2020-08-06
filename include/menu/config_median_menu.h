#pragma once

#include <iostream>
#include <string>
#include "base_menu.h"
#include "../src/objectdetection.h"

class ConfigMedianMenu : public BaseMenu
{
    bool state = false;
public:
    ConfigMedianMenu(BaseMenu* menu, ParameterHandler* param_handler)
    {
        MenuName = std::string("Config Median Filter Menu");
        m_MenuText = std::string("E - Enable/disable median filter\n")
            + "W - set Window size (default = 5)\n"
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
            state = parameter_handler->toggle_filter(std::string("median"));
            std::cout << "median filter state: " << std::to_string(state) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        break;
        case 'W':
        {
            parameter_handler->set_median_window_size(std::stoi(get_input()));
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