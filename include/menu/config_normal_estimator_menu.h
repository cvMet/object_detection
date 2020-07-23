#pragma once

#include <iostream>
#include <string>
#include "base_menu.h"
#include "../src/objectdetection.h"

class ConfigNEMenu : public BaseMenu
{
public:
    ConfigNEMenu(BaseMenu* menu, ParameterHandler * param_handler)
    {
        MenuName = std::string("Config Normal Estimator Menu");
        m_MenuText = std::string("S - add normal estimator Scale factor (default = 5)\n")
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
        case 'S':
        {
            set_ne_scalefactor(std::stoi(get_input()));
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