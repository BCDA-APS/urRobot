epics = require("epics")

local Actions = {
    Custom = 0, -- empty calc record
    OpenRobotiqGripper = 1,
    CloseRobotiqGripper = 2,
}

-- Sets the ActionDoneCalc for the waypoint for one of the preset actions above
function select(args)

    local action = string.format("%s%sWaypoint%s:%d:Action.FLNK", args.P, args.R, args.T, args.N)
    local inpa = string.format("%s%sWaypoint%s:%d:ActionDoneCalc.INPA", args.P, args.R, args.T, args.N)
    local inpb = string.format("%s%sWaypoint%s:%d:ActionDoneCalc.INPB", args.P, args.R, args.T, args.N)
    local inpc = string.format("%s%sWaypoint%s:%d:ActionDoneCalc.INPC", args.P, args.R, args.T, args.N)
    local inpd = string.format("%s%sWaypoint%s:%d:ActionDoneCalc.INPD", args.P, args.R, args.T, args.N)
    local vala = string.format("%s%sWaypoint%s:%d:ActionDoneCalc.A", args.P, args.R, args.T, args.N)
    local valb = string.format("%s%sWaypoint%s:%d:ActionDoneCalc.B", args.P, args.R, args.T, args.N)
    local valc = string.format("%s%sWaypoint%s:%d:ActionDoneCalc.C", args.P, args.R, args.T, args.N)
    local vald = string.format("%s%sWaypoint%s:%d:ActionDoneCalc.D", args.P, args.R, args.T, args.N)
    local calc = string.format("%s%sWaypoint%s:%d:ActionDoneCalc.CALC", args.P, args.R, args.T, args.N)

    local move_status = string.format("%s%sRobotiqGripper:MoveStatusRaw CP", args.P, args.R)

    if (A == Actions.OpenRobotiqGripper) then
        local open = string.format("%s%sRobotiqGripper:Open.PROC", args.P, args.R)
        epics.put(action, open)

        epics.put(inpa, move_status)
        epics.put(calc, "(A!=0)")

    elseif (A == Actions.CloseRobotiqGripper) then
        local close = string.format("%s%sRobotiqGripper:Close.PROC", args.P, args.R)
        epics.put(action, close)

        epics.put(inpa, move_status)
        epics.put(calc, "(A!=0)")

    elseif (A == Actions.Custom) then
        epics.put(action, "")

        epics.put(inpa, "")
        epics.put(inpb, "")
        epics.put(inpc, "")
        epics.put(inpd, "")
        epics.put(vala, "")
        epics.put(valb, "")
        epics.put(valc, "")
        epics.put(vald, "")
        epics.put(calc, "1")
    else
        print(string.format("No preset action for selection: %s", A))
    end
end
