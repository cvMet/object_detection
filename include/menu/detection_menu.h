#pragma once

#include <iostream>
#include <string>
#include "base_menu.h"
#include "detection_menu.h"

class DetectionMenu : public BaseMenu
{
public:
    DetectionMenu()
    {
        m_MenuText = std::string("Detection Menu\n")
            + "D - set Dataset\n"
            + "O - set Object\n"
            + "P - set Preprocessor mode\n"
            + "T - add KPD Threshold (default = 0.7)\n"
            + "N - add KPD #Neighbors (default = 5)\n"
            + "S - enable/disable Statistics\n"
            + "V - enable/disable Visualization\n"
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
        case 'P':
        {
            set_preprocessor(get_input());
        }
        break;
        case 'T':
        {
            add_detector_threshold(std::stof(get_input().substr(0, 3)));
        }
        break;
        case 'N':
        {
            add_detector_threshold(std::stoi(get_input()));
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