#pragma once

#include <iostream>
#include <string>
#include "base_menu.h"
#include "../src/objectdetection.h"

class CloudProcessingMenu : public BaseMenu
{

public:
    CloudProcessingMenu(BaseMenu* menu)
    {
        MenuName = std::string("Cloud Processing Menu");
        m_MenuText = std::string("T - set background removal Threshold (default = 0.005 [m])\n")
            + "B - remove Background\n"
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
        case 'T':
        {
            set_background_removal_threshold(std::stof(get_input().substr(0, 5)));
        }
        break;
        case 'B':
        {
            std::cout << "background_removal state: " << std::to_string(toggle_background_removal()) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        break;
        case 'E':
        {
            set_execution_param(std::string("processing"));
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
