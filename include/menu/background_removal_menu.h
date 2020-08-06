#pragma once

#include <iostream>
#include <string>
#include <chrono>
#include "../include/menu/base_menu.h"
#include "../src/objectdetection.h"

class BackgroundRemovalMenu : public BaseMenu
{
	bool state = false;
public:

	BackgroundRemovalMenu::BackgroundRemovalMenu(BaseMenu* menu, ParameterHandler* param_handler)
	{
		MenuName = std::string("Filter Menu");
		m_MenuText = std::string("B - enable/disable Background removal\n")
			+ "T - background removal Threshold\n"
			+ "R - Return";
		parent = menu;
		child = true;
		parameter_handler = param_handler;
	}

	BaseMenu* getNextMenu(char choice, bool& quit, bool& execute)
	{
		BaseMenu* aNewMenu = 0;

		switch (choice)
		{
		case 'B':
		{
			std::cout << "background_removal state: " << std::to_string(parameter_handler->toggle_background_removal()) << std::endl;
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
		break;
		case 'T':
		{
			parameter_handler->set_background_removal_threshold(std::stof(get_input().substr(0, 5)));
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