#include <memory>
#include <ncurses.h>
#include <string>
#include <iostream>
#include <cassert>

#include <pva/client.h>
#include <pv/caProvider.h>
#include <pv/pvData.h>


int main(int argc, char *argv[]) {
    
    if (argc <= 1) {
        std::cout << "Please provide IOC prefix" << std::endl;
        return 1;
    }
    std::string prefix(argv[1]);
    
    // using channel access for provider
    epics::pvAccess::ca::CAClientFactory::start();
    pvac::ClientProvider provider("ca");

    // Enable auto move and reset commanded values
    // TODO: fix this
    // pvac::ClientChannel channel_AutoMoveL(provider.connect(prefix + "Control:AutoMoveL"));
    // pvac::ClientChannel channel_RstPoseCmd(provider.connect(prefix + "Control:sync_pose_cmd.PROC"));
    // channel_RstPoseCmd.put().set("value", 1).exec();
    // channel_AutoMoveL.put().set("value", 1).exec();

    // Connect PVs for positive and negative X, Y, and Z
    pvac::ClientChannel channel_XFwd(provider.connect(prefix + "Control:PoseXTweakFwd.PROC"));
    pvac::ClientChannel channel_XRev(provider.connect(prefix + "Control:PoseXTweakRev.PROC"));
    pvac::ClientChannel channel_YFwd(provider.connect(prefix + "Control:PoseYTweakFwd.PROC"));
    pvac::ClientChannel channel_YRev(provider.connect(prefix + "Control:PoseYTweakRev.PROC"));
    pvac::ClientChannel channel_ZFwd(provider.connect(prefix + "Control:PoseZTweakFwd.PROC"));
    pvac::ClientChannel channel_ZRev(provider.connect(prefix + "Control:PoseZTweakRev.PROC"));

    // initialize ncurses
    initscr();
    keypad(stdscr, TRUE);
    noecho();

    bool running = true;
    while(running) {
        int ch = getch();
        switch (ch) {
            case 'w':
            case KEY_UP:
                channel_YFwd.put().set("value", 1).exec();
                break;
            case 'a':
            case KEY_LEFT:
                channel_XRev.put().set("value", 1).exec();
                break;
            case 's':
            case KEY_DOWN:
                channel_YRev.put().set("value", 1).exec();
                break;
            case 'd':
            case KEY_RIGHT:
                channel_XFwd.put().set("value", 1).exec();
                break;
            case 'W':
                channel_ZFwd.put().set("value", 1).exec();
                break;
            case 'S':
                channel_ZRev.put().set("value", 1).exec();
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
