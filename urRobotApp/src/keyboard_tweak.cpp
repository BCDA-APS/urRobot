#include "pva/client.h"
#include <ncurses.h>
#include <string>
#include <iostream>
#include <cassert>
#include <sstream>

int main(int argc, char *argv[]) {
    
    if (argc <= 1) {
        std::cout << "Please provide IOC prefix" << std::endl;
        return 1;
    }

    std::string prefix(argv[1]);

    pvac::ClientProvider provider("pva");
    
    // // PVs for positive and negative X, Y, and Z
    // std::stringstream ss;
    // ss << prefix << "Control:PoseXTweakFwd";

    pvac::ClientChannel channel_up(provider.connect("nam:UP"));
    pvac::ClientChannel channel_down(provider.connect("nam:DOWN"));
    pvac::ClientChannel channel_left(provider.connect("nam:LEFT"));
    pvac::ClientChannel channel_right(provider.connect("nam:RIGHT"));

    initscr();
    keypad(stdscr, TRUE);
    noecho();

    int value_up = 0;
    int value_down = 0;
    int value_left = 0;
    int value_right = 0;

    bool running = true;
    while(running) {
        int ch = getch();
        switch (ch) {
            case KEY_UP:
                printw("Up\n");
                value_up = channel_up.get()->getSubField<epics::pvData::PVInt>("value")->getAs<int>();
                channel_up.put().set("value", value_up+1).exec();
                break;
            case KEY_LEFT:
                printw("Left\n");
                value_left = channel_left.get()->getSubField<epics::pvData::PVInt>("value")->getAs<int>();
                channel_left.put().set("value", value_left+1).exec();
                break;
            case KEY_RIGHT:
                printw("Right\n");
                value_right = channel_right.get()->getSubField<epics::pvData::PVInt>("value")->getAs<int>();
                channel_right.put().set("value", value_right+1).exec();
                break;
            case KEY_DOWN:
                printw("Down\n");
                value_down = channel_down.get()->getSubField<epics::pvData::PVInt>("value")->getAs<int>();
                channel_down.put().set("value", value_down+1).exec();
                break;
            case 'q':
                printw("Quit by user\n");
                running = false;
                break;
            default:
                break;
        }
        refresh();
    }
    endwin();

    return 0; 
}
