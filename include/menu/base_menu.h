#pragma once

#include <iostream>
#include <string>
#include <windows.h>

class BaseMenu
{
public:
    BaseMenu() { m_MenuText = "This shouldn't ever be shown!"; }
    virtual ~BaseMenu() { }
    virtual BaseMenu *getNextMenu(char iChoice, bool& iIsQuitOptionSelected) = 0;
    virtual void printText()
    {
        std::cout << m_MenuText << std::endl;
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
    std::string m_MenuText; // This string will be shared by all children (i.e. derived) classes
};