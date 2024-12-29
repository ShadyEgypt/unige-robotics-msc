#ifndef DYNAMIC_DISPLAY_H
#define DYNAMIC_DISPLAY_H

#include <ncurses.h>
#include "globals.h"

void format_right_win(Drone *drone_current_data, WindowLayout layout, int start_y_r, int start_x_r);

#endif // DYNAMIC_DISPLAY_H
