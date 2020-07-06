#pragma once

#include <iostream>
#include <string>
#include "base_menu.h"
#include "merge_menu.h"
#include "cloudcreation_menu.h"
#include "detection_menu.h"
#include "../src/objectdetection.h"

class MainMenu : public BaseMenu
{
public:
	MainMenu()
	{
		MenuName = std::string("Main Menu");
		m_MenuText = std::string("C - Create Clouds\n")
			+ "M - Merge Clouds\n"
			+ "D - Detect Objects\n"
			+ "Q - Quit";
	}

	BaseMenu* getNextMenu(char choice, bool& quit, bool& execute)
	{
		BaseMenu* aNewMenu = 0;

		switch (choice)
		{
			case 'C':
			{
				aNewMenu = new CloudCreationMenu;
			}
			break;
			case 'M':
			{
				aNewMenu = new MergeMenu;
			}
			break;
			case 'D':
			{
				aNewMenu = new DetectionMenu;
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