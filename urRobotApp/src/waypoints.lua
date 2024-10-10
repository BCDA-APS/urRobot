-- NOTE: IOC must add $(URROBOT)/urRobotApp/src to LUA_SCRIPT_PATH
epics = require("epics")

-- Returns a new string with all occurances of 
-- a substring 'old' in 'str' replaced with 'new'
function string.replace(str, old, new)
    local result = ""
    local start_index = 1
    while true do
        local found_start, found_end = string.find(str, old, start_index, true)
        if not found_start then
            break
        end
        result = result .. string.sub(str, start_index, found_start - 1) .. new
        start_index = found_end + 1
    end
    result = result .. string.sub(str, start_index)
    return result
end


function string.split(input_str, delimiter)
    local words = {}
    delimiter = delimiter or "%s"  -- Default delimiter is whitespace
    local pattern = string.format("([^%s]+)", delimiter)
    for word in string.gmatch(input_str, pattern) do
        table.insert(words, word)
    end
    return words
end


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


-- Waits for a process to complete
-- The process status is specified by the value of 'done_pv'
-- which is 1 when the process is done, and 0 when the process is not done
-- The timeout to wait for is given in seconds
function wait_process(done_pv, timeout)
    if timeout == nil then
        timeout = 300.0 -- seconds
    end

    -- wait to start
    local t0 = os.time()
    while true do
        local elap = (os.time() - t0)
        if (elap >= timeout) then
            print("Error: Timeout exceeded waiting to start process")
            break
        end
        done = epics.get(done_pv)
        if done == 0 then
            break
        end
    end

    -- Wait to finish
    t0 = os.time()
    while true do
        local elap = (os.time() - t0)
        if (elap >= timeout) then
            print("Error: Timeout exceeded waiting for process to complete")
            break
        end
        done = epics.get(done_pv)
        if done == 1 then
            break
        end
    end
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
