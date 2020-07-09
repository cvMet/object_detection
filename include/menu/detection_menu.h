#pragma once

#include <iostream>
#include <string>
#include "base_menu.h"
#include "config_kpd_menu.h"
#include "../src/objectdetection.h"

class DetectionMenu : public BaseMenu
{
public:
    DetectionMenu()
    {
        MenuName = std::string("Detection Menu");
        m_MenuText = std::string("D - set Dataset\n")
            + "O - set Object\n"
            + "P - set Preprocessor mode\n"
            + "C - set deteCtion threshold\n"
            + "T - Configure KPD\n"
            + "S - enable/disable runtime Statistics\n"
            + "V - enable/disable Visualization\n"
            + "L - enable/disable detection Logging\n"
            + "M - enable/disable Match retrieval\n"
            + "E - Execute\n"
            + "Q - Quit";
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
        case 'C':
        {
            set_detection_threshold(std::stoi(get_input()));
        }
        break;
        case 'P':
        {
            set_preprocessor(get_input());
        }
        break;
        case 'T':
        {
            aNewMenu = new ConfigKPDMenu(this);
        }
        break;
        case 'S':
        {
            std::cout << "statistics state: " << std::to_string(toggle_detection_stats()) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        break;
        case 'V':
        {
            std::cout << "visualization state: " << std::to_string(toggle_visualization()) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        break;
        case 'L':
        {
            std::cout << "logging state: " << std::to_string(toggle_detection_logging()) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        break;
        case 'M':
        {
            std::cout << "match_retrieval state: " << std::to_string(toggle_match_retrieval()) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        break;
        case 'E':
        {
            set_execution_param(std::string("detection"));
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

        return aNewMenu; // Sending it back to the main function
    }
};