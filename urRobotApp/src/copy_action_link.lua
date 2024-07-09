epics = require("epics")

function copy_action_link(args)
    local global_action_link = string.format("%s%sControl:WaypointAction.FLNK", args.P, args.R)
    local local_action_link = string.format("%s%sWaypoint%s:%d:Action.PROC", args.P, args.R, args.T, args.N)
    epics.put(global_action_link, local_action_link)

    local global_done_link = string.format("%s%sControl:WaypointActionDoneCalc.INPA", args.P, args.R)
    local local_done_link = string.format("%s%sWaypoint%s:%d:ActionDone CP", args.P, args.R, args.T, args.N)
    epics.put(global_done_link, local_done_link)
end
