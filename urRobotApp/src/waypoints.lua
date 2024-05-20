-- NOTE: IOC must add $(URROBOT)/urRobotApp/src to LUA_SCRIPT_PATH
epics = require("epics")

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

-- TODO: allow $(P) and $(R) in PV path file and do text substitution here
function play_path_pv_file(args)
    local done_pv = string.format("%sControl:AsyncMoveDone.RVAL", args.prefix)
    local filename = AA
    local file = io.open(filename, "r")

    local waypoint_move_pvs = {}

    if file then
        for line in file:lines() do
            move_pv = string.format("%s.PROC",line)
            table.insert(waypoint_move_pvs, move_pv)
        end
        file:close()
    else
        print(string.format("Error: unable to open file %s", filename))
    end

    for _, line in ipairs(waypoint_move_pvs) do
        epics.put(line, 1)
        wait_process(done_pv)
    end
end
