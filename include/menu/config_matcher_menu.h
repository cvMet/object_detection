#pragma once

#include <iostream>
#include <string>
#include "base_menu.h"
#include "../src/objectdetection.h"

class ConfigMatcherMenu : public BaseMenu
{
public:
    ConfigMatcherMenu(BaseMenu* menu, ParameterHandler* param_handler)
    {
        MenuName = std::string("Config Matcher Menu");
        m_MenuText = std::string("A - enable/disable RANSAC (default = true)\n")
            + "T - set distance Threshold (default = 0.950f)\n"
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
        case 'A':
        {
            std::cout << "RANSAC state: " << std::to_string(toggle_ransac()) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        break;
        case 'T':
        {
            set_matcher_distance_threshold(std::stof(get_input().substr(0, 5)));
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