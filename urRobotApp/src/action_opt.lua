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

-- sets all fields of calcout record to zero or empty string
local function zero_calcout(pv)
    local to_zero_num = {"A","B","C","D","E","F","G","H","I","J","K","L","CALC","OCAL","SCAN","PREC","ODLY","OOPT","DOPT","IVOA","IVOV"}
    local to_zero_str = {"INPA","INPB","INPC","INPD","INPE","INPF","INPG","INPH","INPI","INPJ","INPK","INPL","OEVT","FLNK","DESC","EGU"}
    for _, v in ipairs(to_zero_num) do
        epics.put(string.format("%s.%s", pv, v), 0)
    end
    for _, v in ipairs(to_zero_str) do
        epics.put(string.format("%s.%s", pv, v),"")
    end
end

-- Sets the "dest" PV to the value of the "src" PV
local function copy_pv(src, dest)
    epics.put(string.format("%s", dest), epics.get(string.format("%s", src)))
end

-- copies all fields from "src" calcout record to "dest" calcout record
local function copy_calcout(src, dest)
    for _,v in ipairs(calcout_fields) do
        copy_pv(string.format("%s.%s", src, v), string.format("%s.%s", dest, v))
    end
end

function select(args)
    local action_opt_val = A

    local action_opt = string.format("%sWaypoint%s:%d:ActionOpt", args.P, args.T, args.N)
    local action_done_calc = string.format("%sWaypoint%s:%d:ActionDoneCalc", args.P, args.T, args.N)
    local action_done_calc_inpa = string.format("%sWaypoint%s:%d:ActionDoneCalc.INPA", args.P, args.T, args.N)
    local action_done_calc_N_val = string.format("%sActionDoneCalc%d.VAL CP", args.P, action_opt_val)
    local action_link = string.format("%sWaypoint%s:%d:Action.FLNK", args.P, args.T, args.N)
    local action_link_N = string.format("%sActionLink%d.FLNK", args.P, action_opt_val)
    local action_link_N_desc = string.format("%sActionLink%d.DESC", args.P, action_opt_val)
    --
    -- 0 means no waypoint action
    if action_opt_val == 0 then
        epics.put(action_done_calc_inpa, "")
        return
    end

    -- print(action_link_N, " -> ", action_link)
    copy_pv(action_link_N, action_link)
    -- print(action_done_calc_inpa, " = ", action_done_calc_N_val)
    epics.put(action_done_calc_inpa, action_done_calc_N_val)
end

-- function update_option_names(args)
    -- local enum_strs = {"ONST","TWST","THST","FRST","FVST","SXST","SVST","EIST","NIST","TEST"}
    -- local action_opt = string.format("%sWaypoint%s:%d:ActionOpt", args.P, args.T, args.N)
    -- local updating_names_flag = string.format("%sWaypoint%s:%d:updating_names_flag", args.P, args.T, args.N)
    -- epics.put(updating_names_flag, 1)
    -- epics.put(string.format("%s.ONST", action_opt), AA)
    -- epics.put(string.format("%s.TWST", action_opt), BB)
    -- epics.put(string.format("%s.THST", action_opt), CC)
    -- epics.put(string.format("%s.FRST", action_opt), DD)
    -- epics.put(string.format("%s.FVST", action_opt), EE)
    -- epics.put(string.format("%s.SXST", action_opt), FF)
    -- epics.put(string.format("%s.SVST", action_opt), GG)
    -- epics.put(string.format("%s.EIST", action_opt), HH)
    -- epics.put(string.format("%s.NIST", action_opt), II)
    -- epics.put(string.format("%s.TEST", action_opt), JJ)
    -- epics.put(updating_names_flag, 0)
-- end

-- function select(args)
--
    -- local action_done_calc = string.format("%sWaypoint%s:%d:ActionDoneCalc", args.P, args.T, args.N)
    -- local action_done_calc_save = string.format("%sWaypoint%s:%d:action_done_calc_save", args.P, args.T, args.N)
    -- local action_flnk= string.format("%sWaypoint%s:%d:Action.FLNK", args.P, args.T, args.N)
    -- local action_save_flnk = string.format("%sWaypoint%s:%d:action_save.FLNK", args.P, args.T, args.N)
    -- local action_opt_last = string.format("%sWaypoint%s:%d:action_opt_last", args.P, args.T, args.N)
--
    -- if epics.get(action_opt_last) == Actions.Custom then
        -- copy_calcout(action_done_calc, action_done_calc_save)
        -- copy_pv(action_flnk, action_save_flnk)
    -- end
--
    -- epics.put(action_opt_last, A)
--
    -- local inpa = string.format("%s%sWaypoint%s:%d:ActionDoneCalc.INPA", args.P, args.R, args.T, args.N)
    -- local inpb = string.format("%s%sWaypoint%s:%d:ActionDoneCalc.INPB", args.P, args.R, args.T, args.N)
    -- local inpc = string.format("%s%sWaypoint%s:%d:ActionDoneCalc.INPC", args.P, args.R, args.T, args.N)
    -- local scan = string.format("%s%sWaypoint%s:%d:ActionDoneCalc.SCAN", args.P, args.R, args.T, args.N)
    -- local calc = string.format("%s%sWaypoint%s:%d:ActionDoneCalc.CALC", args.P, args.R, args.T, args.N)
--
    -- local move_status = string.format("%s%sRobotiqGripper:MoveStatusRaw CP", args.P, args.R)
--
    -- if (A == Actions.OpenRobotiqGripper) then
        -- zero_calcout(action_done_calc)
        -- local open = string.format("%s%sRobotiqGripper:Open.PROC", args.P, args.R)
        -- epics.put(action_flnk, open)
--
        -- local is_open = string.format("%s%sRobotiqGripper:IsOpen CP", args.P, args.R)
        -- local is_stopped_outer = string.format("%s%sRobotiqGripper:IsStoppedOuter CP", args.P, args.R)
        -- epics.put(inpa, move_status)
        -- epics.put(inpb, is_open)
        -- epics.put(inpc, is_stopped_outer)
        -- epics.put(scan, ".1 second")
        -- epics.put(calc, "(A!=0) && ((B==1) || (C==1))")
--
    -- elseif (A == Actions.CloseRobotiqGripper) then
        -- zero_calcout(action_done_calc)
        -- local close = string.format("%s%sRobotiqGripper:Close.PROC", args.P, args.R)
        -- epics.put(action_flnk, close)
--
        -- local is_closed = string.format("%s%sRobotiqGripper:IsClosed CP", args.P, args.R)
        -- local is_stopped_inner = string.format("%s%sRobotiqGripper:IsStoppedInner CP", args.P, args.R)
        -- epics.put(inpa, move_status)
        -- epics.put(inpb, is_closed)
        -- epics.put(inpc, is_stopped_inner)
        -- epics.put(scan, ".1 second")
        -- epics.put(calc, "(A!=0) && ((B==1) || (C==1))")
--
    -- elseif (A == Actions.Custom) then
        -- copy_calcout(action_done_calc_save, action_done_calc)
        -- copy_pv(action_save_flnk, action_flnk)
    -- else
        -- print(string.format("No preset action for selection: %s", A))
    -- end
--
-- end
