#pragma once

#include <iostream>
#include <string>
#include <chrono>
#include "../include/menu/base_menu.h"
#include "../src/objectdetection.h"

class FilterMenu : public BaseMenu
{
	bool state = false;
public:

	FilterMenu::FilterMenu(BaseMenu* menu, ParameterHandler* param_handler)
	{
		MenuName = std::string("Filter Menu");
		m_MenuText = std::string("Choose the filters you want to apply during cloud creation\n")
			+ "M - Median filter\n"
			+ "R - ROI filter\n"
			+ "S - SOR filter\n"
			+ "Q Quit";
		parent = menu;
		child = true;
		parameter_handler = param_handler;
	}

	BaseMenu* getNextMenu(char choice, bool& quit, bool& execute)
	{
		BaseMenu* aNewMenu = 0;

		switch (choice)
		{
		case 'M':
		{
			state = parameter_handler->toggle_filter(std::string("median"));
			std::cout << "median filter state: " << std::to_string(state) << std::endl;
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
		break;
		case 'R':
		{
			state = parameter_handler->toggle_filter(std::string("roi"));
			std::cout << "roi filter state: " << std::to_string(state) << std::endl;
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
		break;
		case 'S':
		{
			state = parameter_handler->toggle_filter(std::string("sor"));
			std::cout << "statistical outlier removal filter state: " << std::to_string(state) << std::endl;
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
		break;
		case 'Q':
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