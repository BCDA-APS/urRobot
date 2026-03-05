epics = require("epics")

function select(args)
    local action_opt_val = A

    local action_done_calc_INPA = string.format("%sWaypoint%s:%d:ActionDoneCalc.INPA", args.P, args.T, args.N)
    local action_done_calc_CALC = string.format("%sWaypoint%s:%d:ActionDoneCalc.CALC", args.P, args.T, args.N)
    local action_done_calc_N_VAL = string.format("%sActionDoneCalc%d.VAL CP", args.P, action_opt_val)
    local action_link_FLNK = string.format("%sWaypoint%s:%d:Action.FLNK", args.P, args.T, args.N)
    local action_link_N_FLNK = string.format("%sActionSseq%d.PROC", args.P, action_opt_val)

    if action_opt_val == 0 then
	-- 0 means no waypoint action
        epics.put(action_link_FLNK, " ")
        epics.put(action_done_calc_CALC, "1")
    else
        epics.put(action_done_calc_CALC, "A")
	epics.put(action_link_FLNK, action_link_N_FLNK)
        epics.put(action_done_calc_INPA, action_done_calc_N_VAL)
    end

end
