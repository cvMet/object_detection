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

	BaseMenu* getNextMenu(char choice, bool& quit, bool& execute) // This is us actually defining the pure virtual method above
	{
		BaseMenu* aNewMenu = 0; // We're setting up the pointer here, but makin sure it's null (0)

		switch (choice) // Notice - I have only done "options". You would obviously need to do this for all of your menus
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
				// Ah, they selected quit! Update the bool we got as input
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