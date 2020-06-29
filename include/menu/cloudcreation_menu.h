#pragma once

#include <iostream>
#include <string>
#include "base_menu.h"
#include "../src/objectdetection.h"

class MainMenu;
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
            + "E - Execute";
    }


    BaseMenu* getNextMenu(char choice, bool& iIsQuitOptionSelected)
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
            std::cout << "not implemented yet" << std::endl;
        }
        break;
        case 'E':
        {
            std::cout << "not implemented yet" << std::endl;
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
