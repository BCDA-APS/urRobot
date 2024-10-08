epics = require("epics")

local Actions = {
    Custom = 0,
    OpenRobotiqGripper = 1,
    CloseRobotiqGripper = 2,
}

local calcout_fields = {
    "A",
    "B",
    "C",
    "D",
    "E",
    "F",
    "G",
    "H",
    "I",
    "J",
    "K",
    "L",
    "INPA",
    "INPB",
    "INPC",
    "INPD",
    "INPE",
    "INPF",
    "INPG",
    "INPH",
    "INPI",
    "INPJ",
    "INPK",
    "INPL",
    "CALC",
    "OCAL",
    "SCAN",
    "PREC",
    "ODLY",
    "OEVT",
    "OOPT",
    "DOPT",
    "IVOA",
    "IVOV",
    "OUT",
    "FLNK",
    "DESC",
    "EVNT",
    "EGU",
}

-- stores all calcout fields from src in dest
-- FIX: make sure this zeros everything
local function zero_calcout(pv)
    local to_zero_num = {"A","B","C","D","E","F","G","H","I","J","K","L","SCAN",""}
    local to_zero_str = {"INPA","INPB","INPC","INPD","INPE","INPF","INPG","INPH","INPI","INPJ","INPK","INPL"}
    for _, v in ipairs(to_zero_num) do
        epics.put(string.format("%s.%s", pv, v), 0)
    end
    for _, v in ipairs(to_zero_str) do
        epics.put(string.format("%s.%s", pv, v),"")
    end
end

-- stores all calcout fields from src in dest
local function copy_calcout(src, dest)
    for _,v in ipairs(calcout_fields) do
        -- print(string.format("copying %s.%s to %s.%s", src, v, dest, v))
        src_val = epics.get(string.format("%s.%s", src, v))
        epics.put(string.format("%s.%s", dest, v), src_val)
    end
end

-- Copies the FLNK from src to dest
local function copy_action_FLNK(src, dest)
    epics.put(string.format("%s.FLNK", dest), string.format("%s.FLNK", src))
end


function select(args)

    local action_done_calc = string.format("%sWaypoint%s:%d:ActionDoneCalc", args.P, args.T, args.N)
    local action_done_calc_save = string.format("%sWaypoint%s:%d:action_done_calc_save", args.P, args.T, args.N)
    local action = string.format("%sWaypoint%s:%d:Action.FLNK", args.P, args.T, args.N)
    local action_save = string.format("%sWaypoint%s:%d:action_save.FLNK", args.P, args.T, args.N)
    local action_opt_last = string.format("%sWaypoint%s:%d:action_opt_last", args.P, args.T, args.N)

    if epics.get(action_opt_last) == Actions.Custom then
        copy_calcout(action_done_calc, action_done_calc_save)
        -- copy_action_FLNK(action, action_save)
    end
    
    epics.put(action_opt_last, A)

    local inpa = string.format("%s%sWaypoint%s:%d:ActionDoneCalc.INPA", args.P, args.R, args.T, args.N)
    local inpb = string.format("%s%sWaypoint%s:%d:ActionDoneCalc.INPB", args.P, args.R, args.T, args.N)
    local inpc = string.format("%s%sWaypoint%s:%d:ActionDoneCalc.INPC", args.P, args.R, args.T, args.N)
    local calc = string.format("%s%sWaypoint%s:%d:ActionDoneCalc.CALC", args.P, args.R, args.T, args.N)

    local move_status = string.format("%s%sRobotiqGripper:MoveStatusRaw CP", args.P, args.R)

    if (A == Actions.OpenRobotiqGripper) then
        zero_calcout(action_done_calc)
        local open = string.format("%s%sRobotiqGripper:Open.PROC", args.P, args.R)
        epics.put(action, open)

        local is_open = string.format("%s%sRobotiqGripper:IsOpen CP", args.P, args.R)
        local is_stopped_outer = string.format("%s%sRobotiqGripper:IsStoppedOuter CP", args.P, args.R)
        epics.put(inpa, move_status)
        epics.put(inpb, is_open)
        epics.put(inpc, is_stopped_outer)
        epics.put(calc, "(A!=0) && ((B==1) || (C==1))")

    elseif (A == Actions.CloseRobotiqGripper) then
        zero_calcout(action_done_calc)
        local close = string.format("%s%sRobotiqGripper:Close.PROC", args.P, args.R)
        epics.put(action, close)

        local is_closed = string.format("%s%sRobotiqGripper:IsClosed CP", args.P, args.R)
        local is_stopped_inner = string.format("%s%sRobotiqGripper:IsStoppedInner CP", args.P, args.R)
        epics.put(inpa, move_status)
        epics.put(inpb, is_closed)
        epics.put(inpc, is_stopped_inner)
        epics.put(calc, "(A!=0) && ((B==1) || (C==1))")

    elseif (A == Actions.Custom) then
        print("Custom action selected")
        copy_calcout(action_done_calc_save, action_done_calc)
        -- copy_action_FLNK(action_save, action)
    else
        print(string.format("No preset action for selection: %s", A))
    end

end
