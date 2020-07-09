#pragma once

#include <iostream>
#include <string>
#include "base_menu.h"
#include "../src/objectdetection.h"

class MergeMenu : public BaseMenu
{
public:
    MergeMenu(BaseMenu* menu)
    {
        MenuName = std::string("Merge Menu");
        m_MenuText = std::string("V - enable/disable Visualization\n")
            + "E - Execute\n"
            + "R - Return";
        parent = menu;
        child = true;
    }

    BaseMenu* getNextMenu(char choice, bool& quit, bool& execute)
    {
        BaseMenu* aNewMenu = 0;

        switch (choice)
        {
        case 'V':
        {
            std::cout << "visualization state: " << std::to_string(toggle_visualization()) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        break;

        case 'E':
        {
            set_execution_param(std::string("merging"));
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

        return aNewMenu; // Sending it back to the main function
    }
};