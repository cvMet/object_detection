#pragma once

#include <iostream>
#include <string>
#include "base_menu.h"
#include "learning_menu.h"
#include "../src/objectdetection.h"

class LearningMenu : public BaseMenu
{

public:
    LearningMenu(BaseMenu* menu, ParameterHandler* param_handler)
    {
        MenuName = std::string("Learning Menu");
        m_MenuText = std::string("L - enable query Learning\n")
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
        case 'E':
        {
            execute = true;
        }
        break;

        case 'L':
        {
            //To be implemented - sry 
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
