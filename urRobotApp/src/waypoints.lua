-- NOTE: IOC must add $(URROBOT)/urRobotApp/src to LUA_SCRIPT_PATH
epics = require("epics")


-- Returns a new string with all occurances of 
-- a substring 'old' in 'str' replaced with 'new'
function string_replace(str, old, new)
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


function split_string(input_str, delimiter)
    local words = {}
    delimiter = delimiter or "%s"  -- Default delimiter is whitespace
    local pattern = string.format("([^%s]+)", delimiter)
    for word in string.gmatch(input_str, pattern) do
        table.insert(words, word)
    end
    return words
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


function play_path_pv_file(args)

    local done_pv = string.format("%sControl:AsyncMoveDone.RVAL", args.prefix)
    local filename = AA
    local file = io.open(filename, "r")

    -- Split each line by whitespace and store in table of tables, e.g.
    -- {{"WaypointJ:1:moveJ", "OPEN"},{"WaypointJ:2"}}
    local lines = {}
    if file then
        for line in file:lines() do
            local _line = string_replace(line, "$(P)$(R)", args.prefix)
            if string.len(_line) > 0 then
                local words = split_string(_line, " ")
                table.insert(lines, words)
            end
        end
    else
        error(string.format("Unable to read file '%s'", filename))
    end
    file:close()

    -- Apply gripper override, saving initial state before
    -- FIX: gripper0 shouldn't have duplicates
    local gripper0 = {}
    for _, line in ipairs(lines) do
        local gripper_pv = string_replace(line[1], "moveJ", "Gripper")
        gripper_pv = string_replace(gripper_pv, "moveL", "Gripper")
        local gripper_pv_rval = string.format("%s.RVAL", gripper_pv)
        table.insert(gripper0,{gripper_pv,epics.get(gripper_pv_rval)})
        local override_str = line[2]
        if override_str ~= nil then
            if override_str == "CLOSE" or override_str == "CLOSED" then
                -- io.write("Setting ",gripper_pv, " ", line[2],"\n")
                epics.put(gripper_pv, 1)
            elseif override_str == "OPEN" or override_str == "OPENED" then
                -- io.write("Setting ", gripper_pv, " ", line[2],"\n")
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
