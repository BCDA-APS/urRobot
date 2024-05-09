epics = require("epics")

-- Save each waypoint and gripper action to the specified file
-- with the format "j1,j2,j3,j4,j5,j6,gripper,speed,accel"
function save_waypoints(args)
    local MAX_WAYPOINTS = A
    local file = io.open(args.filename, "w")
    if file then
        for i = 1, MAX_WAYPOINTS do -- for each waypoint
            local enabled = epics.get(string.format("%s%sWaypoint%d:Enabled", args.P, args.R, i))
            if (enabled == 1) then
                for j = 1, 6 do -- for each joint angle
                    local jx = epics.get(string.format("%s%sWaypoint%d:J%d.VAL", args.P, args.R, i, j))
                    file:write(string.format("%f,", jx))
                end
                local grip = epics.get(string.format("%s%sWaypoint%d:Gripper.VAL", args.P, args.R, i))
                local speed = epics.get(string.format("%s%sWaypoint%d:Speed.VAL", args.P, args.R, i))
                local accel = epics.get(string.format("%s%sWaypoint%d:Acceleration.VAL", args.P, args.R, i))
                file:write(string.format("%f,%f,%f\n", grip, speed, accel))
            end
        end
        file:close()
    else
        print(string.format("Error: Unable to open file '%s' for writing", args.filename))
    end
end
