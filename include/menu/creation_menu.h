#pragma once

#include <iostream>
#include <string>
#include "base_menu.h"
#include "creation_menu.h"
#include "filter_menu.h"
#include "../src/objectdetection.h"
#include "../include/parameter_handler.h"

class CloudCreationMenu : public BaseMenu
{

public:
    CloudCreationMenu(BaseMenu* menu, ParameterHandler* param_handler)
    {
        MenuName = std::string("Cloud Creation Menu");
        m_MenuText = std::string("D - set Dataset\n")
            + "O - set Object\n"
            + "F - Filter options\n"
            + "S - enable/disable Statistics\n"
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
        case 'D':
        {
            parameter_handler->set_dataset(get_input());
        }
        break;
        case 'O':
        {
            parameter_handler->set_object(get_input());
        }
        break;
        case 'F':
        {
            aNewMenu = new FilterMenu(this, parameter_handler);
        }
        break;
        case 'S':
        {
            std::cout << "statistics state: " << std::to_string(parameter_handler->toggle_cloudgen_stats()) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        break;
        case 'E':
        {
            parameter_handler->set_execution_param(std::string("cloudcreation"));
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

        return aNewMenu;
    }
};
