#include "dynamic_display.h"
#include <ncurses.h>
#include <math.h>
#include "globals.h"

void format_right_win(Drone *drone_current_data, WindowLayout layout, int start_y_r, int start_x_r)
{
    // Displaying the current position of the drone with nice formatting
    mvwprintw(layout.right_split, start_y_r, start_x_r, "position {");
    mvwprintw(layout.right_split, start_y_r + 1, start_x_r, "\tx: %f",
              drone_current_data->drone_pos.x);
    mvwprintw(layout.right_split, start_y_r + 2, start_x_r, "\ty: %f",
              drone_current_data->drone_pos.y);
    mvwprintw(layout.right_split, start_y_r + 3, start_x_r, "}");

    // Displaying the current vel of the drone with nice formatting
    mvwprintw(layout.right_split, start_y_r + 4, start_x_r, "vel {");
    mvwprintw(layout.right_split, start_y_r + 5, start_x_r, "\tx: %f",
              drone_current_data->drone_vel.x);
    mvwprintw(layout.right_split, start_y_r + 6, start_x_r, "\ty: %f",
              drone_current_data->drone_vel.y);
    mvwprintw(layout.right_split, start_y_r + 7, start_x_r, "}");

    // Displaying the current force beeing applied on the drone only by the
    // user. So no border effects are taken into consideration while
    // displaying these values.
    mvwprintw(layout.right_split, start_y_r + 8, start_x_r, "force {");
    mvwprintw(layout.right_split, start_y_r + 9, start_x_r, "\tx: %f",
              drone_current_data->drone_force.x);
    mvwprintw(layout.right_split, start_y_r + 10, start_x_r, "\ty: %f",
              drone_current_data->drone_force.y);
    mvwprintw(layout.right_split, start_y_r + 11, start_x_r, "}");
}