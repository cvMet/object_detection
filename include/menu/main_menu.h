#pragma once

#include <iostream>
#include <string>
#include "base_menu.h"
#include "merge_menu.h"
#include "cloudcreation_menu.h"
#include "learning_menu.h"
#include "detection_menu.h"
#include "../src/objectdetection.h"

class MainMenu : public BaseMenu
{
public:
	MainMenu()
	{
		MenuName = std::string("Main Menu");
		m_MenuText = std::string("C - Create clouds\n")
			+ "M - Merge clouds\n"
			+ "D - Detect objects\n"
			+ "L - Learn descriptors\n"
			+ "Q - Quit";
	}

	BaseMenu* getNextMenu(char choice, bool& quit, bool& execute)
	{
		BaseMenu* aNewMenu = 0;

		switch (choice)
		{
			case 'C':
			{
				aNewMenu = new CloudCreationMenu(this);
			}
			break;
			case 'M':
			{
				aNewMenu = new MergeMenu(this);
			}
			break;
			case 'D':
			{
				aNewMenu = new DetectionMenu(this);
			}
			break;
			case 'L':
			{
				aNewMenu = new LearningMenu(this);
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