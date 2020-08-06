#pragma once

#include <iostream>
#include <string>
#include "base_menu.h"
#include "../src/objectdetection.h"

class ConfigSORMenu : public BaseMenu
{
    bool state = false;
public:
    ConfigSORMenu(BaseMenu* menu, ParameterHandler* param_handler)
    {
        MenuName = std::string("Config Statistical Outlier Removal Filter Menu");
        m_MenuText = std::string("E - Enable/disable SOR filter\n")
            + "K - set K neighbor count (default = 10)\n"
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
            state = parameter_handler->toggle_filter(std::string("sor"));
            std::cout << "statistical outlier removal filter state: " << std::to_string(state) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        break;
        case 'W':
        {
            parameter_handler->set_sor_neighbor_count(std::stoi(get_input()));
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