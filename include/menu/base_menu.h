#pragma once

#include <iostream>
#include <string>
#include <windows.h>
#include "../include/menu/base_menu.h"

class BaseMenu
{
public:
    BaseMenu() { MenuName = "BaseMenu", m_MenuText = "This shouldn't ever be shown!", parent = NULL; }
    bool child = false;
    virtual ~BaseMenu() { }
    virtual BaseMenu *getNextMenu(char iChoice, bool& quit, bool& execute) = 0;
    virtual void printText()
    {
        std::cout << m_MenuText << std::endl;
    }
    virtual void printFlavorText()
    {
        int width = longest_menu_item();
        int margin = floor((width - MenuName.length()) / 2);
        for (int i = 0; i < width; ++i) {
            std::cout << "-";
        }
        std::cout << std::endl;
        for (int i = 0; i < margin; ++i) {
            std::cout << " ";
        }
        std::cout << MenuName << std::endl;
        for (int i = 0; i < width; ++i) {
            std::cout << "-";
        }
        std::cout << std::endl;
    }
    virtual int longest_menu_item(void) {
        std::string delimiter = "\n";
        std::string s = m_MenuText;
        size_t pos = 0;
        std::string token;
        int max = 0;
        while ((pos = s.find(delimiter)) != std::string::npos) {
            token = s.substr(0, pos);
            if (token.length() > max) {
                max = token.length();
            }
            s.erase(0, pos + delimiter.length());
        }
        return max;
    }

    void clearScreen()
    {
        HANDLE                     hStdOut;
        CONSOLE_SCREEN_BUFFER_INFO csbi;
        DWORD                      count;
        DWORD                      cellCount;
        COORD                      homeCoords = { 0, 0 };

        hStdOut = GetStdHandle(STD_OUTPUT_HANDLE);
        if (hStdOut == INVALID_HANDLE_VALUE) return;

        /* Get the number of cells in the current buffer */
        if (!GetConsoleScreenBufferInfo(hStdOut, &csbi)) return;
        cellCount = csbi.dwSize.X * csbi.dwSize.Y;

        /* Fill the entire buffer with spaces */
        if (!FillConsoleOutputCharacter(
            hStdOut,
            (TCHAR)' ',
            cellCount,
            homeCoords,
            &count
        )) return;

        /* Fill the entire buffer with the current colors and attributes */
        if (!FillConsoleOutputAttribute(
            hStdOut,
            csbi.wAttributes,
            cellCount,
            homeCoords,
            &count
        )) return;

        /* Move the cursor home */
        SetConsoleCursorPosition(hStdOut, homeCoords);
    }

protected:
    std::string MenuName;
    std::string m_MenuText;// This string will be shared by all children (i.e. derived) classes
    BaseMenu* parent;
};