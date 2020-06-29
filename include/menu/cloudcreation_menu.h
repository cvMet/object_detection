#pragma once

#include <iostream>
#include <string>
#include "base_menu.h"
#include "../src/objectdetection.h"
#include "../include/menu/filter_menu.h"

class CloudCreationMenu : public BaseMenu
{
public:
    CloudCreationMenu()
    {
        m_MenuText = std::string("CloudCreationMenu\n")
            + "Please make your selection\n"
            + "D - Set dataset\n"
            + "O - Set object\n"
            + "F - Filter options\n"
            + "E - Execute\n"
            + "Q - Quit\n"
            + "Selection: ";
    }


    BaseMenu* getNextMenu(char choice, bool& quit, bool& execute)
    {
        BaseMenu* aNewMenu = 0;

        switch (choice)
        {
        case 'D':
        {
            set_dataset(get_input());
        }
        break;
        case 'O':
        {
            set_object(get_input());
        }
        break;
        case 'F':
        {
            aNewMenu = new FilterMenu;
        }
        break;
        case 'E':
        {
            set_execution_param(std::string("cloudcreation"));
            execute = true;
        }
        break;
        case 'Q':
        {
            quit = true;
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
