#pragma once

#include <iostream>
#include <string>
#include "base_menu.h"
#include "../include/menu/cloudcreation_menu.h"

class FilterMenu : public BaseMenu
{
public:

    FilterMenu()
    {
        m_MenuText = std::string("Filter Menu\n")
            + "Choose the filters you want to apply during cloud creation\n"
            + "M - Median filter\n"
            + "R - ROI filter\n"
            + "S - SOR filter\n"
            + "Q Quit\n"
            + "Selection: ";
    }

    BaseMenu* getNextMenu(char choice, bool& quit, bool& execute)
    {
        BaseMenu* aNewMenu = 0;

        switch (choice)
        {
        case 'M':
        {
            std::cout << "median" << std::endl;
        }
        break;
        case 'R':
        {
            std::cout << "roi" << std::endl;
        }
        break;
        case 'S':
        {
            std::cout << "sor" << std::endl;
        }
        break;
        case 'Q':
        {
            std::cout << "sor" << std::endl;
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