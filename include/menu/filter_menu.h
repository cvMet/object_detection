#pragma once

#include <iostream>
#include <string>
#include <chrono>
#include "../include/menu/base_menu.h"

class FilterMenu : public BaseMenu
{
	bool state = false;
public:

	FilterMenu::FilterMenu()
	{
		m_MenuText = std::string("Filter Menu\n")
			+ "Choose the filters you want to apply during cloud creation\n"
			+ "M - Median filter\n"
			+ "R - ROI filter\n"
			+ "S - SOR filter\n"
			+ "Q Quit";
	}

	FilterMenu::FilterMenu(BaseMenu* menu)
	{
		m_MenuText = std::string("Filter Menu\n")
			+ "Choose the filters you want to apply during cloud creation\n"
			+ "M - Median filter\n"
			+ "R - ROI filter\n"
			+ "S - SOR filter\n"
			+ "Q Quit";
		parent = menu;
		child = true;
	}

	BaseMenu* getNextMenu(char choice, bool& quit, bool& execute)
	{
		BaseMenu* aNewMenu = 0;

		switch (choice)
		{
		case 'M':
		{
			state = toggle_filter(std::string("median"));
			std::cout << "median filter state: " << std::to_string(state) << std::endl;
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
		break;
		case 'R':
		{
			state = toggle_filter(std::string("roi"));
			std::cout << "roi filter state: " << std::to_string(state) << std::endl;
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
		break;
		case 'S':
		{
			state = toggle_filter(std::string("sor"));
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