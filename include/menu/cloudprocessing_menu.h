#pragma once

#include <iostream>
#include <string>
#include "base_menu.h"
#include "../src/objectdetection.h"
#include "../include/parameter_handler.h"
#include "../include/menu/background_removal_menu.h"

class CloudProcessingMenu : public BaseMenu
{

public:
    CloudProcessingMenu(BaseMenu* menu, ParameterHandler* param_handler)
    {
        MenuName = std::string("Cloud Processing Menu");
        m_MenuText = std::string("F - Filter clouds\n")
            + "D - Downsample clouds\n"
            + "B - Background removal\n"
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
        case 'F':
        {
            aNewMenu = new FilterMenu(this, parameter_handler);
        }
        break;
        case 'D':
        {
            std::cout << "downsampling not implemented yet" << std::endl;
        }
        break;
        case 'B':
        {
            aNewMenu = new BackgroundRemovalMenu(this, parameter_handler);
        }
        break;
        case 'E':
        {
            parameter_handler->set_execution_param(std::string("processing"));
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
