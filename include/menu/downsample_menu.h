#pragma once

#include <iostream>
#include <string>
#include <chrono>
#include "../include/menu/base_menu.h"
#include "../src/objectdetection.h"

class DownsampleMenu : public BaseMenu
{
	bool state = false;
public:

	DownsampleMenu::DownsampleMenu(BaseMenu* menu, ParameterHandler* param_handler)
	{
		MenuName = std::string("Downsample Menu");
		m_MenuText = std::string("E - Enable/disable downsampling\n")
			+ "L - set Leaf size (default = 0.005[m])\n"
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
			state = parameter_handler->toggle_downsample();
			std::cout << "Downsample state: " << std::to_string(state) << std::endl;
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
		break;
		case 'L':
		{
			parameter_handler->set_leaf_size(std::stof(get_input().substr(0, 5)));
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