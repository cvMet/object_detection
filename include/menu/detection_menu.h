#pragma once

#include <iostream>
#include <string>
#include "base_menu.h"
#include "config_kpd_menu.h"
#include "config_normal_estimator_menu.h"
#include "config_output_menu.h"
#include "config_matcher_menu.h"
#include "config_input_menu.h"
#include "../src/objectdetection.h"

class DetectionMenu : public BaseMenu
{
public:
    DetectionMenu(BaseMenu* menu)
    {
        MenuName = std::string("Detection Menu");
        m_MenuText = std::string("T - set detection Threshold\n")
            + "I - configure Input\n"
            + "N - configure Normal estimator\n"
            + "K - configure KPD\n"
            + "M - configure Matcher\n"
            + "O - configure Output\n"
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
            set_detection_threshold(std::stoi(get_input()));
        }
        break;
        case 'I':
        {
            aNewMenu = new ConfigInputMenu(this);
        }
        break;
        case 'N':
        {
            aNewMenu = new ConfigNEMenu(this);
        }
        break;
        case 'K':
        {
            aNewMenu = new ConfigKPDMenu(this);
        }
        break;
        case 'M':
        {
            aNewMenu = new ConfigMatcherMenu(this);
        }
        break;
        case 'O':
        {
            aNewMenu = new ConfigOutputMenu(this);
        }
        break;
        case 'E':
        {
            set_execution_param(std::string("detection"));
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