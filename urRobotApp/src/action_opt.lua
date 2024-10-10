epics = require("epics")

-- Sets the "dest" PV to the value of the "src" PV
local function copy_pv(src, dest)
    epics.put(string.format("%s", dest), epics.get(string.format("%s", src)))
end

function select(args)
    local action_opt_val = A

    local action_done_calc_INPA = string.format("%sWaypoint%s:%d:ActionDoneCalc.INPA", args.P, args.T, args.N)
    local action_done_calc_CALC = string.format("%sWaypoint%s:%d:ActionDoneCalc.CALC", args.P, args.T, args.N)
    local action_done_calc_N_VAL = string.format("%sActionDoneCalc%d.VAL CP", args.P, action_opt_val)
    local action_link_FLNK = string.format("%sWaypoint%s:%d:Action.FLNK", args.P, args.T, args.N)
    local action_link_N_FLNK = string.format("%sActionLink%d.FLNK", args.P, action_opt_val)

    -- 0 means no waypoint action
    if action_opt_val == 0 then
        epics.put(action_done_calc_CALC, "1")
    else
        epics.put(action_done_calc_CALC, "A")
        copy_pv(action_link_N_FLNK, action_link_FLNK)
        epics.put(action_done_calc_INPA, action_done_calc_N_VAL)
    end

end
