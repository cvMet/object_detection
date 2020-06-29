#pragma once

#include <iostream>
#include <string>
#include "base_menu.h"

class DetectionMenu : public BaseMenu
{
public:
    DetectionMenu()
    {
        m_MenuText = std::string("CloudCreationMenu\n")
            + "Please make your selection\n"
            + "a - Return to MainMenu\n"
            + "b - Return to MainMenu";
    }

    BaseMenu* getNextMenu(char choice, bool& iIsQuitOptionSelected)
    {
        BaseMenu* aNewMenu = 0;

        switch (choice)
        {
        case 'a':
        {
            std::cout << "pushed 1" << std::endl;
        }
        break;
        case 'b':
        {
            std::cout << "pushed 2" << std::endl;
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