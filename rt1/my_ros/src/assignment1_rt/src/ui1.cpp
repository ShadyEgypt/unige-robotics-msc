#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <ncurses.h>
#include <vector>
#include <string>

// Function to kill an existing turtle
void killExistingTurtle(ros::NodeHandle &nh, const std::string &turtle_name)
{
    ros::ServiceClient kill_client = nh.serviceClient<turtlesim::Kill>("/kill");
    turtlesim::Kill srv;
    srv.request.name = turtle_name;

    if (kill_client.call(srv))
    {
        ROS_INFO("Killed existing turtle: %s", turtle_name.c_str());
    }
    else
    {
        ROS_WARN("Failed to kill turtle: %s. It might not exist.", turtle_name.c_str());
    }
}

// Function to spawn a new turtle
void spawnTurtle(ros::NodeHandle &nh, const std::string &turtle_name, float x, float y, float theta)
{
    ros::ServiceClient spawn_client = nh.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn srv;

    srv.request.x = x;
    srv.request.y = y;
    srv.request.theta = theta;
    srv.request.name = turtle_name;

    if (spawn_client.call(srv))
    {
        ROS_INFO("Spawned turtle: %s", turtle_name.c_str());
    }
    else
    {
        ROS_ERROR("Failed to spawn turtle: %s", turtle_name.c_str());
    }
}

void drawBox(WINDOW *win, int start_y, int start_x, const std::string &title, bool isActive)
{
    int width = 40, height = 10;
    wattron(win, isActive ? COLOR_PAIR(2) : COLOR_PAIR(1));
    box(win, 0, 0);
    wattroff(win, isActive ? COLOR_PAIR(2) : COLOR_PAIR(1));
    mvwprintw(win, 0, (width - title.length()) / 2, title.c_str());
}

void updateVelocities(WINDOW *win, const std::vector<float> &linear, const std::vector<float> &angular, bool isActive)
{
    wattron(win, isActive ? A_BOLD : A_DIM);
    mvwprintw(win, 2, 2, "Linear Velocities: ");
    mvwprintw(win, 3, 4, "vx: %.2f", linear[0]);
    mvwprintw(win, 4, 4, "vy: %.2f", linear[1]);
    mvwprintw(win, 6, 2, "Angular Velocities:");
    mvwprintw(win, 7, 4, "wz: %.2f", angular[0]);
    wattroff(win, isActive ? A_BOLD : A_DIM);
    wrefresh(win);
}

void editVelocities(WINDOW *win, std::vector<float> &linear, std::vector<float> &angular)
{
    echo();
    curs_set(1);
    mvwprintw(win, 3, 20, "Enter vx: ");
    wrefresh(win);
    wscanw(win, "%f", &linear[0]);
    mvwprintw(win, 4, 20, "Enter vy: ");
    wrefresh(win);
    wscanw(win, "%f", &linear[1]);
    mvwprintw(win, 7, 20, "Enter wz: ");
    wrefresh(win);
    wscanw(win, "%f", &angular[0]);
    curs_set(0);
    noecho();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ui_node");
    ros::NodeHandle nh;

    // Kill existing turtles if they exist
    killExistingTurtle(nh, "turtle1");
    killExistingTurtle(nh, "turtle2");

    // Spawn two new turtles
    spawnTurtle(nh, "turtle1", 2.0, 2.0, 0.0);
    spawnTurtle(nh, "turtle2", 8.0, 8.0, 0.0);

    ros::Publisher proposedVelTurtle1Pub = nh.advertise<geometry_msgs::Twist>("/proposed_vel_turtle1", 10);
    ros::Publisher proposedVelTurtle2Pub = nh.advertise<geometry_msgs::Twist>("/proposed_vel_turtle2", 10);

    initscr();
    start_color();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);

    init_pair(1, COLOR_WHITE, COLOR_BLUE);
    init_pair(2, COLOR_WHITE, COLOR_GREEN);

    WINDOW *turtle1Win = newwin(10, 40, 2, 5);
    WINDOW *turtle2Win = newwin(10, 40, 2, 50);

    std::vector<float> turtle1Linear(2, 0.0f), turtle1Angular(1, 0.0f);
    std::vector<float> turtle2Linear(2, 0.0f), turtle2Angular(1, 0.0f);

    bool isTurtle1Active = true;
    int key;

    while (true)
    {
        clear();   // Clear the entire screen
        refresh(); // Update the main screen
        drawBox(turtle1Win, 2, 5, "Turtle1", isTurtle1Active);
        drawBox(turtle2Win, 2, 50, "Turtle2", !isTurtle1Active);

        updateVelocities(turtle1Win, turtle1Linear, turtle1Angular, isTurtle1Active);
        updateVelocities(turtle2Win, turtle2Linear, turtle2Angular, !isTurtle1Active);

        key = getch();

        if (key == 'q' || key == 'Q')
        {
            break;
        }
        else if (key == KEY_LEFT || key == KEY_RIGHT)
        {
            isTurtle1Active = !isTurtle1Active;
        }
        else if (key == '\n')
        {
            geometry_msgs::Twist proposed_vel;
            if (isTurtle1Active)
            {
                editVelocities(turtle1Win, turtle1Linear, turtle1Angular);
                proposed_vel.linear.x = turtle1Linear[0];
                proposed_vel.linear.y = turtle1Linear[1];
                proposed_vel.angular.z = turtle1Angular[0];
                proposedVelTurtle1Pub.publish(proposed_vel);
                // ROS_INFO("Publishing proposed velocity for turtle1: vx=%.2f, vy=%.2f, wz=%.2f",
                //          proposed_vel.linear.x, proposed_vel.linear.y, proposed_vel.angular.z);
            }
            else
            {
                editVelocities(turtle2Win, turtle2Linear, turtle2Angular);
                proposed_vel.linear.x = turtle2Linear[0];
                proposed_vel.linear.y = turtle2Linear[1];
                proposed_vel.angular.z = turtle2Angular[0];
                proposedVelTurtle2Pub.publish(proposed_vel);
                // ROS_INFO("Publishing proposed velocity for turtle2: vx=%.2f, vy=%.2f, wz=%.2f",
                //          proposed_vel.linear.x, proposed_vel.linear.y, proposed_vel.angular.z);
            }
        }
    }

    delwin(turtle1Win);
    delwin(turtle2Win);
    endwin();

    return 0;
}
