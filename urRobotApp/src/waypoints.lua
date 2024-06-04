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

-- FIX: isn't working may just need to adjust tolerance
function waypointL_reached(args)
    local actual_pose = {A, B, C, D, E, F}
    local waypoint_base = string.format("%sWaypointL:%s", args.prefix, args.N)
    local waypoint_pose = {
        epics.get(string.format("%s:X",waypoint_base)),
        epics.get(string.format("%s:Y",waypoint_base)),
        epics.get(string.format("%s:Z",waypoint_base)),
        epics.get(string.format("%s:Roll",waypoint_base)),
        epics.get(string.format("%s:Pitch",waypoint_base)),
        epics.get(string.format("%s:Yaw",waypoint_base))
    }
    local reached = table.almost_equal(waypoint_pose, actual_pose, args.tol)
    return reached and 1 or 0
end

function play_path_pv_file(args)

    local done_pv = string.format("%sControl:AsyncMoveDone.RVAL", args.prefix)
    local filename = AA
    local file = io.open(filename, "r")

    -- Split each line by whitespace and store in table of tables, e.g.
    -- {{"WaypointJ:1:moveJ", "OPEN"},{"WaypointJ:2"}}
    local lines = {}
    if file then
        for line in file:lines() do
            local _line = string.replace(line, "$(P)$(R)", args.prefix)
            if string.len(_line) > 0 then
                local words = string.split(_line, " ")
                table.insert(lines, words)
            end
        end
    else
        error(string.format("Unable to read file '%s'", filename))
    end
    file:close()

    -- Apply gripper override, saving initial state before
    local gripper0 = {}
    for _, line in ipairs(lines) do
        local gripper_pv = string.replace(line[1], "moveJ", "Gripper")
        gripper_pv = string.replace(gripper_pv, "moveL", "Gripper")
        local gripper_pv_rval = string.format("%s.RVAL", gripper_pv)
        table.insert(gripper0,{gripper_pv,epics.get(gripper_pv_rval)})
        local override_str = line[2]
        if override_str ~= nil then
            if override_str == "close" or override_str == "closed" then
                epics.put(gripper_pv, 1)
            elseif override_str == "OPEN" or override_str == "OPENED" then
                epics.put(gripper_pv, 0)
            else
                error(string.format("Invalid gripper override '%s' in path file", override_str))
            end
        end

        -- Move to the waypoint
        local move_pv = line[1]
        epics.put(move_pv, 1)
        wait_process(done_pv)

    end

    -- Reset gripper action PVs to their original values
    for _,v in ipairs(gripper0) do
        epics.put(v[1], v[2])
    end

    -- reset joint and pose command values
    local reset_JCmd_string = string.format("%sControl:ResetJCmd.PROC",args.prefix)
    local reset_PoseCmd_string = string.format("%sControl:ResetPoseCmd.PROC",args.prefix)
    epics.put(reset_JCmd_string, 1)
    epics.put(reset_PoseCmd_string, 1)
end


function _lookup_waypoint_pvs(args, wp_type, waypoint_pv_table)
    local fmt_string
    local nmax
    if wp_type == "J" then
        fmt_string = "%sWaypointJ:%d"
        nmax = args.maxj
    elseif wp_type == "L" then
        fmt_string = "%sWaypointL:%d"
        nmax = args.maxl
    else
        error("Invalid waypoint type")
    end
    for n = 1,nmax do
        local pv_name = string.format(fmt_string,args.prefix, n)
        local pv_value = epics.get(pv_name)
        if not waypoint_pv_table[pv_value] then
            waypoint_pv_table[pv_value] = pv_name
        else
            if #pv_value > 0 then
                local msg = string.format(
                    "\nRequested path is ambiguous. Multiple waypoints with the same name '%s'",
                    pv_value
                )
                error(msg)
            end
        end
    end
end

function play_path_pv_desc_file(args)

    local done_pv = string.format("%sControl:AsyncMoveDone.RVAL", args.prefix)

    -- Lookup waypoint PVs and their string names e.g. $(P)$(R)WaypointL:$(N).VAL
    local waypoint_pvs = {} -- map {string name : pv name}
    _lookup_waypoint_pvs(args, "J", waypoint_pvs)
    _lookup_waypoint_pvs(args, "L", waypoint_pvs)
    print("\nAll loaded waypoint PVs:")
    for k,v in pairs(waypoint_pvs) do
        print(k,v)
    end

    -- Read file to get requested waypoints
    local filename = AA
    local wp_req = {}
    local gripper_overrides = {}
    local file = io.open(filename, "r")
    if file then
        for line in file:lines() do
            local line_split = string.split(line,'"')
            table.insert(wp_req, line_split[1])
            if line_split[2] then
                table.insert(gripper_overrides, string.replace(line_split[2], " ",""))
            else
                table.insert(gripper_overrides, "NONE")
            end
        end
    else
        error(string.format("Unable to read file '%s'", filename))
    end
    file:close()
    print("\nRequested waypoints/overrides:")
    for i,_ in ipairs(wp_req) do
        print(wp_req[i], gripper_overrides[i])
    end

    -- Save original gripper states
    local gripper0 = {} -- {pv_name : gripper action}
    for _,wp in ipairs(wp_req) do
        local grip_pv_name = string.format("%s:Gripper",waypoint_pvs[wp])
        gripper0[grip_pv_name] = epics.get(grip_pv_name)
    end

    -- move to each requested waypoint and gripper override
    print("")
    for i,wp in ipairs(wp_req) do
        print("Move to ", waypoint_pvs[wp])
        if gripper_overrides[i] ~= "NONE" then
            if gripper_overrides[i] == "OPEN" or gripper_overrides[i] == "OPENED" then
                epics.put(string.format("%s:Gripper",waypoint_pvs[wp]), 0)
                print("Overriding gripper action to OPEN")
            elseif gripper_overrides[i] == "CLOSE" or gripper_overrides[i] == "CLOSED" then
                epics.put(string.format("%s:Gripper",waypoint_pvs[wp]), 1)
                print("Overriding gripper action to CLOSE")
            end
        end

        local move_pv
        if string.find(waypoint_pvs[wp], "WaypointJ") then
            move_pv = string.format("%s:moveJ.PROC",waypoint_pvs[wp])
        elseif string.find(waypoint_pvs[wp], "WaypointL") then
            move_pv = string.format("%s:moveL.PROC",waypoint_pvs[wp])
        end
        epics.put(move_pv, 1)
        wait_process(done_pv)
    end

    -- reset gripper actions to original values
    for k,v in pairs(gripper0) do
        epics.put(k, v)
    end

    -- reset joint and pose command values
    local reset_JCmd_string = string.format("%sControl:ResetJCmd.PROC",args.prefix)
    local reset_PoseCmd_string = string.format("%sControl:ResetPoseCmd.PROC",args.prefix)
    epics.put(reset_JCmd_string, 1)
    epics.put(reset_PoseCmd_string, 1)

end
