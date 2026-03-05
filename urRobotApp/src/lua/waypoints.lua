-- NOTE: IOC must add $(URROBOT)/urRobotApp/src/lua to LUA_SCRIPT_PATH
epics = require("epics")

-- Returns true if two numbers are within the specified tolerance of each other
function almost_equal(a, b, tol)
    tol = tol or 1e-6
    if (math.abs(a - b) <= tol) then
        return true
    else
        return false
    end
end


-- Returns true if two tables have equal elements within the tolerance
function table.almost_equal(t1, t2, tol)
    tol = tol or 1e-6
    assert(#t1 == #t2, "Tables must be the same length")

    for i, _ in ipairs(t1) do
        if not almost_equal(t1[i], t2[i], tol) then
            return false
        end
    end
    return true
end

-- TODO: Make PVs for tolerance
function waypointJ_reached(args)
    local actual_pose = {A, B, C, D, E, F}
    local waypoint_base = string.format("%sWaypointJ:%s", args.prefix, args.N)
    local waypoint_pose = {
        epics.get(string.format("%s:J1",waypoint_base)),
        epics.get(string.format("%s:J2",waypoint_base)),
        epics.get(string.format("%s:J3",waypoint_base)),
        epics.get(string.format("%s:J4",waypoint_base)),
        epics.get(string.format("%s:J5",waypoint_base)),
        epics.get(string.format("%s:J6",waypoint_base))
    }
    local reached = table.almost_equal(waypoint_pose, actual_pose, args.tol)
    return reached and 1 or 0
end

function waypointL_reached(args)
    local actual_pose_xyz = {A, B, C}
    local actual_pose_rpy = {D, E, F}
    local waypoint_base = string.format("%sWaypointL:%s", args.prefix, args.N)
    local waypoint_pose_xyz = {
        epics.get(string.format("%s:X",waypoint_base)),
        epics.get(string.format("%s:Y",waypoint_base)),
        epics.get(string.format("%s:Z",waypoint_base)),
    }
    local waypoint_pose_rpy = {
        epics.get(string.format("%s:Roll",waypoint_base)),
        epics.get(string.format("%s:Pitch",waypoint_base)),
        epics.get(string.format("%s:Yaw",waypoint_base))
    }

    local reached1 = table.almost_equal(waypoint_pose_xyz, actual_pose_xyz, 1.0)
    local reached2 = table.almost_equal(waypoint_pose_rpy, actual_pose_rpy, 0.1)
    return (reached1 and reached2) and 1 or 0
end
