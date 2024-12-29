#ifndef INPUT_DISPLAY_H
#define INPUT_DISPLAY_H

#include <ncurses.h>
#include "globals.h"

void begin_format_input(int input, WindowLayout layout);

void end_format_input(int input, WindowLayout layout);

void format_left_win(int input, WindowLayout layout);

void refresh_left_win(WindowLayout layout);

#endif // INPUT_DISPLAY_H
