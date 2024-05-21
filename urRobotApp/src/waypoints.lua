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
    local waypoint_move_pvs = {}
    if file then
        for line in file:lines() do
            line_sub = string_replace(line, "$(P)$(R)", args.prefix)
            move_pv = string.format("%s.PROC",line_sub)
            table.insert(waypoint_move_pvs, move_pv)
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
