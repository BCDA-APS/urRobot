-- NOTE: IOC must add $(URROBOT)/urRobotApp/src to LUA_SCRIPT_PATH
epics = require("epics")
PI = 3.141592653589793

-- TODO: Maybe rewrite this in C and call it with sub record?
function save_waypoints(args)
    local MAX_WAYPOINTS = A
    local file = io.open(args.filename, "w")
    if file then
        for i = 1, MAX_WAYPOINTS do
            local enabled = epics.get(string.format("%s%sWaypoint%d:Enabled", args.P, args.R, i))
            if (enabled == 1) then
                for j = 1, 6 do
                    local jx = epics.get(string.format("%s%sWaypoint%d:J%d.VAL", args.P, args.R, i, j))
                    jx = jx * PI / 180.0 -- convert to radians
                    file:write(string.format("%f,", jx))
                end
                local speed = epics.get(string.format("%s%sWaypoint%d:Speed.VAL", args.P, args.R, i))
                local accel = epics.get(string.format("%s%sWaypoint%d:Acceleration.VAL", args.P, args.R, i))
                local blend = epics.get(string.format("%s%sWaypoint%d:Blend.VAL", args.P, args.R, i))
                local grip = epics.get(string.format("%s%sWaypoint%d:Gripper.VAL", args.P, args.R, i))
                file:write(string.format("%f,%f,%f,%f\n", speed, accel, blend, grip))
            end
        end
        file:close()
    else
        print(string.format("Error: Unable to open file '%s' for writing", args.filename))
    end
end
