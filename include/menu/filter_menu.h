#pragma once

#include <iostream>
#include <string>
#include <chrono>
#include "../include/menu/base_menu.h"
#include "../src/objectdetection.h"
#include "../include/menu/config_median_menu.h"
#include "../include/menu/config_sor_menu.h"
#include "../include/menu/config_roi_menu.h"

class FilterMenu : public BaseMenu
{
	bool state = false;
public:

	FilterMenu::FilterMenu(BaseMenu* menu, ParameterHandler* param_handler)
	{
		MenuName = std::string("Filter Menu");
		m_MenuText = std::string("E - enable/disable Filtering\n")
			+ "M - Median filter\n"
			+ "I - ROI filter\n"
			+ "S - SOR filter\n"
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
		case 'E':
		{
			state = parameter_handler->toggle_filtering_state();
			std::cout << "Filtering state: " << std::to_string(state) << std::endl;
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
		break;
		case 'M':
		{
			aNewMenu = new ConfigMedianMenu(this, parameter_handler);
		}
		break;
		case 'I':
		{
			aNewMenu = new ConfigROIMenu(this, parameter_handler);
		}
		break;
		case 'S':
		{
			aNewMenu = new ConfigSORMenu(this, parameter_handler);
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