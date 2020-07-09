#pragma once

#include <iostream>
#include <string>
#include "base_menu.h"
#include "learning_menu.h"
#include "../src/objectdetection.h"

class LearningMenu : public BaseMenu
{

public:
    LearningMenu(BaseMenu* menu)
    {
        MenuName = std::string("Learning Menu");
        m_MenuText = std::string("L - enable query Learning\n")
            + "E - Execute\n"
            + "R - Return\n"
            + "Q - Quit";
        parent = menu;
        child = true;
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
            set_query_learning();
        }
        break;

        case 'Q':
        {
            quit = true;
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