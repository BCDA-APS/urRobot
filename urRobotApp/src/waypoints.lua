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

    -- Read waypoint PVs from file
    -- Gripper action can be overridden here
    -- TODO: set gripper action back to what it was before running this
    local waypoint_move_pvs = {}
    if file then
        for line in file:lines() do
            local _line = string_replace(line, "$(P)$(R)", args.prefix)
            if string.len(_line) > 0 then
                local move_pv = ""
                words = split_string(_line, " ")
                if #words == 1 then
                    move_pv = string.format("%s.PROC",_line)
                elseif #words == 2 then
                    local gripper_pv = string_replace(words[1],"moveJ", "Gripper")
                    gripper_pv = string_replace(gripper_pv,"moveL", "Gripper")
                    if words[2] == "OPEN" then
                        print(string.format("setting gripper action to OPEN for %s", words[1]))
                        epics.put(gripper_pv, 0)
                    elseif words[2] == "CLOSE" then
                        print(string.format("setting gripper action to CLOSE for %s", words[1]))
                        epics.put(gripper_pv, 1)
                    else
                        error("Error reading path file. Invalid gripper action")
                    end
                    move_pv = string.format("%s.PROC",words[1])
                else
                    error("Error reading path file")
                end
                table.insert(waypoint_move_pvs, move_pv)
            end
        end
        file:close()
    else
        print(string.format("Error: unable to open file %s", filename))
    end

    -- Move to each waypoint, waiting to reach it before continuing
    -- TODO: ignore empty lines, handle hash comments
    for _, line in ipairs(waypoint_move_pvs) do
        epics.put(line, 1)
        wait_process(done_pv)
    end

    -- reset joint and pose command values
    local reset_JCmd_string = string.format("%sControl:ResetJCmd.PROC",args.prefix)
    local reset_PoseCmd_string = string.format("%sControl:ResetPoseCmd.PROC",args.prefix)
    epics.put(reset_JCmd_string, 1)
    epics.put(reset_PoseCmd_string, 1)

end
