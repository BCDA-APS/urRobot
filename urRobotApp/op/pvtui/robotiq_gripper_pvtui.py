#!/usr/bin/env python3

from textual.app import App
from textual.widgets import Header, Label
from textual.containers import Horizontal, Vertical, Container
from pvtui import PVLed, PVButton, PVTextMonitor, PVInput
import argparse


class RobotiqGripperTUI(App):

    CSS_PATH = "robotiq_gripper_pvtui.css"

    def __init__(self, macros=None):
        super().__init__()
        self.macros = macros

    def compose(self):
        yield Header(show_clock=True)
        with Vertical():
            with Horizontal(classes="hrow"):
                yield Label("Connected: ")
                yield PVLed("$(P)RobotiqGripper:Connected", self.macros)
            with Horizontal(classes="hrow"):
                yield Label("Active:    ")
                yield PVLed("$(P)RobotiqGripper:IsActive", self.macros)
            with Horizontal(classes="hrow"):
                yield Label("Open:      ")
                yield PVLed("$(P)RobotiqGripper:IsOpen", self.macros)
            with Horizontal(classes="hrow"):
                yield Label("Closed:    ")
                yield PVLed("$(P)RobotiqGripper:IsClosed", self.macros)
            with Horizontal(classes="hrow"):
                yield Label("Current Position:    ")
                yield PVTextMonitor("$(P)RobotiqGripper:CurrentPosition", self.macros)
            with Horizontal(classes="hrow"):
                yield Label("Closed Position:    ")
                yield PVTextMonitor("$(P)RobotiqGripper:ClosedPosition", self.macros)
            with Horizontal(classes="hrow"):
                yield Label("Open Position:    ")
                yield PVTextMonitor("$(P)RobotiqGripper:OpenPosition", self.macros)
            with Horizontal(classes="hrow"):
                yield Label("Move status (raw):    ")
                yield PVTextMonitor("$(P)RobotiqGripper:MoveStatusRaw", self.macros)
            with Horizontal(classes="hrow"):
                yield Label("Move status:    ")
                yield PVTextMonitor("$(P)RobotiqGripper:MoveStatus.VAL", self.macros)
            yield PVButton("$(P)RobotiqGripper:Connect", self.macros, label="Connect")
            yield PVButton("$(P)RobotiqGripper:Activate", self.macros, label="Activate")
            yield PVButton("$(P)RobotiqGripper:Open", self.macros, label="Open")
            yield PVButton("$(P)RobotiqGripper:Close", self.macros, label="Close")
            yield PVButton("$(P)RobotiqGripper:AutoCalibrate", self.macros, label="Calibrate")

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser()
    parser.add_argument('-macro')
    args = parser.parse_args()
    
    # macros are given in the same form as
    # caQtDM and MEDM, e.g.
    # -macro P=xxx:,R=a_macro:,M=another:
    macros_dict = dict()
    if args.macro is not None:
        for m in args.macro.split(","):
            kv = m.replace(" ", "").split("=")
            macros_dict.update({kv[0]:kv[1]})
    
    macros_dict = {"P" : "urExample:"}

    app = RobotiqGripperTUI(macros=macros_dict)
    app.title = "Robotiq Gripper"
    app.run()



