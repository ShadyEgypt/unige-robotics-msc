#include <ncurses.h>

int main()
{
    // Initialize ncurses
    initscr();
    noecho();
    cbreak();

    // Get the current terminal size
    int rows, cols;
    getmaxyx(stdscr, rows, cols); // Get number of rows and columns of the terminal

    // Display the current terminal size
    mvprintw(0, 0, "Current terminal size:");
    mvprintw(1, 0, "Rows: %d", rows);
    mvprintw(2, 0, "Columns: %d", cols);

    refresh(); // Refresh to show the output

    // Wait for user input to exit
    getch();

    // End ncurses
    endwin();

    return 0;
}
