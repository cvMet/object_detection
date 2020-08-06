#pragma once

#include <iostream>
#include <string>
#include "base_menu.h"
#include "merge_menu.h"
#include "creation_menu.h"
#include "learning_menu.h"
#include "processing_menu.h"
#include "detection_menu.h"
#include "../src/objectdetection.h"

class MainMenu : public BaseMenu
{
public:
	MainMenu(ParameterHandler* param_handler)
	{
		MenuName = std::string("Main Menu");
		m_MenuText = std::string("C - Create clouds\n")
			+ "P - Process clouds\n"
			+ "M - Merge clouds\n"
			+ "D - Detect objects\n"
			+ "L - Learn descriptors\n"
			+ "Q - Quit";
		parameter_handler = param_handler;
	}

	BaseMenu* getNextMenu(char choice, bool& quit, bool& execute)
	{
		BaseMenu* aNewMenu = 0;

		switch (choice)
		{
			case 'C':
			{
				aNewMenu = new CloudCreationMenu(this, parameter_handler);
			}
			break;
			case 'P':
			{
				aNewMenu = new CloudProcessingMenu(this, parameter_handler);
			}
			break;
			case 'M':
			{
				aNewMenu = new MergeMenu(this, parameter_handler);
			}
			break;
			case 'D':
			{
				aNewMenu = new DetectionMenu(this, parameter_handler);
			}
			break;
			case 'L':
			{
				aNewMenu = new LearningMenu(this, parameter_handler);
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

		return aNewMenu;
	}

};