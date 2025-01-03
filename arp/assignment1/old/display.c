#include <ncurses.h>
#include <stdlib.h>
#include <unistd.h>

void setup_background()
{
    clear();
    bkgd(COLOR_PAIR(1)); // Set background color pair
    mvprintw(0, (COLS - 7) / 2, "DISPLAY");
    mvhline(1, 0, '=', COLS);
    mvhline(LINES - 2, 0, '=', COLS);
    mvprintw(LINES - 1, 2, "Press Ctrl+C to exit");
    refresh();
}

int main()
{
    // Initialize ncurses
    initscr();
    start_color();
    cbreak();
    noecho();
    curs_set(0); // Hide cursor

    // Define colors
    if (has_colors())
    {
        init_pair(1, COLOR_GREEN, COLOR_BLACK); // Green text, black background
        init_pair(2, COLOR_WHITE, COLOR_BLUE);  // Alternate pair
    }
    else
    {
        endwin();
        fprintf(stderr, "Your terminal does not support color.\n");
        exit(1);
    }

    // Set up the BIOS-like background
    setup_background();

    // Placeholder for waiting for child processes and sending PIDs (to be implemented)
    // ...

    // Main loop
    while (1)
    {
        // Refresh periodically
        usleep(500000);
    }

    // End ncurses mode
    endwin();
    return 0;
}
