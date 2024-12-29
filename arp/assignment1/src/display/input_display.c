
#include "input_display.h"
#include <ncurses.h>
#include "globals.h"
// Function to set the pressed input green
void begin_format_input(int input, WindowLayout layout)
{
    switch (input)
    {
    case 'q':
        wattron(layout.tl_win, COLOR_PAIR(1));
        break;
    case 'w':
        wattron(layout.tc_win, COLOR_PAIR(1));
        break;
    case 'e':
        wattron(layout.tr_win, COLOR_PAIR(1));
        break;
    case 'a':
        wattron(layout.cl_win, COLOR_PAIR(1));
        break;
    case 's':
        wattron(layout.cc_win, COLOR_PAIR(1));
        break;
    case 'd':
        wattron(layout.cr_win, COLOR_PAIR(1));
        break;
    case 'z':
        wattron(layout.bl_win, COLOR_PAIR(1));
        break;
    case 'x':
        wattron(layout.bc_win, COLOR_PAIR(1));
        break;
    case 'c':
        wattron(layout.br_win, COLOR_PAIR(1));
        break;
    case ' ':
        wattron(layout.cc_win, COLOR_PAIR(1));
        break;
    }
}

// Function to reset the color of the pressed input
void end_format_input(int input, WindowLayout layout)
{
    switch (input)
    {
    case 'q':
        wattroff(layout.tl_win, COLOR_PAIR(1));
        break;
    case 'w':
        wattroff(layout.tc_win, COLOR_PAIR(1));
        break;
    case 'e':
        wattroff(layout.tr_win, COLOR_PAIR(1));
        break;
    case 'a':
        wattroff(layout.cl_win, COLOR_PAIR(1));
        break;
    case 's':
        wattroff(layout.cc_win, COLOR_PAIR(2));
        break;
    case 'd':
        wattroff(layout.cr_win, COLOR_PAIR(1));
        break;
    case 'z':
        wattroff(layout.bl_win, COLOR_PAIR(1));
        break;
    case 'x':
        wattroff(layout.bc_win, COLOR_PAIR(1));
        break;
    case 'c':
        wattroff(layout.br_win, COLOR_PAIR(1));
        break;
    case ' ':
        wattroff(layout.cc_win, COLOR_PAIR(1));
        break;
    }
}

void format_left_win(int input, WindowLayout layout)
{
    // Begin the coloring of the pressed key if any key is pressed
    begin_format_input(input, layout);

    /// Creating the ascii arts
    // Up-left arrow
    mvwprintw(layout.tl_win, 1, 3, "_");
    mvwprintw(layout.tl_win, 2, 2, "'\\");

    // Up arrow
    mvwprintw(layout.tc_win, 1, 3, "A");
    mvwprintw(layout.tc_win, 2, 3, "|");

    // Up-right arrow
    mvwprintw(layout.tr_win, 1, 3, "_");
    mvwprintw(layout.tr_win, 2, 3, "/'");

    // Left arrow
    mvwprintw(layout.cl_win, 2, 2, "<");
    mvwprintw(layout.cl_win, 2, 3, "-");

    // Right arrow
    mvwprintw(layout.cr_win, 2, 3, "-");
    mvwprintw(layout.cr_win, 2, 4, ">");

    // Down-left arrow
    mvwprintw(layout.bl_win, 2, 2, "|/");
    mvwprintw(layout.bl_win, 3, 2, "'-");

    // Down arrow
    mvwprintw(layout.bc_win, 2, 3, "|");
    mvwprintw(layout.bc_win, 3, 3, "V");

    // Down-right arrow
    mvwprintw(layout.br_win, 2, 3, "\\|");
    mvwprintw(layout.br_win, 3, 3, "-'");

    // Break symbol
    mvwprintw(layout.cc_win, 2, 3, "X");

    // Disable the color for the next iteration
    end_format_input(input, layout);

    // Setting the symbols for the corners of the cells
    mvwprintw(layout.tc_win, 0, 0, ".");
    mvwprintw(layout.tr_win, 0, 0, ".");
    mvwprintw(layout.cl_win, 0, 0, "+");
    mvwprintw(layout.cc_win, 0, 0, "+");
    mvwprintw(layout.cr_win, 0, 0, "+");
    mvwprintw(layout.cr_win, 0, 6, "+");
    mvwprintw(layout.bl_win, 0, 0, "+");
    mvwprintw(layout.bc_win, 0, 0, "+");
    mvwprintw(layout.br_win, 0, 0, "+");
    mvwprintw(layout.br_win, 0, 6, "+");
    mvwprintw(layout.bc_win, 4, 0, "'");
    mvwprintw(layout.br_win, 4, 0, "'");

    // Signaling what's the button to close everything
    mvwprintw(layout.left_split, LINES - 3, 3, "Press p to close everything");
}

void refresh_left_win(WindowLayout layout)
{
    wrefresh(layout.right_split);
    wrefresh(layout.left_split);
    wrefresh(layout.tl_win);
    wrefresh(layout.tc_win);
    wrefresh(layout.tr_win);
    wrefresh(layout.cl_win);
    wrefresh(layout.cc_win);
    wrefresh(layout.cr_win);
    wrefresh(layout.bl_win);
    wrefresh(layout.bc_win);
    wrefresh(layout.br_win);
}