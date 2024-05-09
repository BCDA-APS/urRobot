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


-- TODO:
-- - Read file to get joint angles, gripper, speed, accel
-- - Set auto_move off (0)
-- - Set J1-J6Cmd values
-- - Set JointSpeed, JointAcceleration
-- - call MoveJ (wait for it to finish)
-- - call open/close gripper 
--

function play_waypoints(args)
    local MAX_WAYPOINTS = A
    local file = io.open(args.filename, "r")
    if file then
        local auto_move_pv = string.format("%s%sControl:auto_moveJ", args.P, args.R)
        epics.put(auto_move_pv, 0)
        for line in file:lines() do
            local fields = {}
            for field in line:gmatch("[^,]+") do
                table.insert(fields, field)
            end
            local grip = 0 -- move gripper after joints
            for i, value in ipairs(fields) do
                if (i >= 1 and i <= 6) then
                    -- set JxCmd to angle values on this line
                    local Jx_pv = string.format("%s%sControl:J%dCmd", args.P, args.R, i)
                    epics.put(Jx_pv, value)
                elseif (i == 7) then
                    grip = value
                elseif (i == 8) then
                    local speed_pv = string.format("%s%sControl:JointSpeed", args.P, args.R)
                    epics.put(speed_pv, value)
                elseif (i == 9) then
                    local accel_pv = string.format("%s%sControl:JointAcceleration", args.P, args.R)
                    epics.put(accel_pv, value)
                end
            end

            -- moveJ and wait for it to finish
            local moveJ_pv = string.format("%s%sControl:moveJ.PROC", args.P, args.R)
            local steady_pv = string.format("%s%sControl:Steady", args.P, args.R)
            epics.put(moveJ_pv, 1)

            -- wait for move to begin
            while true do
                local steady = epics.get(steady_pv)
                if (steady == 0) then
                    break
                end
            end

            -- wait for current move to finish
            print("Moving to waypoint")
            while true do
                local steady = epics.get(steady_pv)
                if (steady == 1) then
                    break
                end
            end
            print("Waypoint reached!")
            local gripper_status_pv = string.format("%s%sRobotiqGripper:MoveStatus", args.P, args.R)
            local gripper_status = epics.get(gripper_status_pv)
            print(gripper_status)
        end

        file:close()
    else
        print(string.format("Error: Unable to open file '%s' for writing", args.filename))
    end
end
