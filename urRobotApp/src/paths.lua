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


function path_go(args)

    local done_pv = string.format("%sControl:AsyncMoveDone.RVAL", args.prefix)

    for i = 1,args.kmax do
        local wp_type = epics.get(string.format("%sPath%d:%d:Type", args.prefix, args.N, i))
        wp_type = (wp_type == 0.0) and "L" or "J"
        local wp_num = epics.get(string.format("%sPath%d:%d:Number", args.prefix, args.N, i))
        local wp = string.format("%sWaypoint%s:%d", args.prefix, wp_type, wp_num)
        local wp_action = math.floor(epics.get(string.format("%sPath%d:%d:ActionOverride", args.prefix, args.N, i)))
        local wp_enabled = epics.get(string.format("%sPath%d:%d:Enabled", args.prefix, args.N, i))
        if (wp_num <= 0) then
            wp_enabled = 0
        end

        if wp_enabled ~= 0 then
            print(string.format("Moving to waypoint %s...", wp))

            -- Apply action override
            local action_pv
            local wp_action0
            local action_opt_val_pv
            local set_action_opt_lnk0_pv
            local set_action_opt_proc_pv
            if wp_action ~= 3 then
                print(string.format("Overriding action to %d", wp_action))
                action_pv = string.format("%s:ActionOpt", wp)
                wp_action0 = math.floor(epics.get(action_pv))
                action_opt_val_pv = string.format("%sPath%d:%d:action_opt_val", args.prefix, args.N, i)
                set_action_opt_lnk0_pv = string.format("%sPath%d:%d:set_action_opt.LNK0", args.prefix, args.N, i)
                set_action_opt_proc_pv = string.format("%sPath%d:%d:set_action_opt.PROC", args.prefix, args.N, i)
                epics.put(action_opt_val_pv, wp_action)
                epics.put(set_action_opt_lnk0_pv, string.format("%s PP",action_pv))
                epics.put(set_action_opt_proc_pv, 1) -- manually force processing
            end

            -- Move to waypoint and wait for motion and action to finish
            -- move_pv = string.format("%s:move%s.PROC", wp, wp_type)
            -- epics.put(move_pv, 1)

            -- wait_process(done_pv)
            -- wait for 3 seconds to simulate waiting for motion
            t0 = os.time()
            while true do
                local elap = os.time() - t0
                if (elap >= 3.0) then
                    break
                end
            end
            print("Done waiting")

            -- Restore original action
            if wp_action ~= 3 then
                print(string.format("Restoring original action to %d", wp_action0))
                epics.put(action_opt_val_pv, wp_action0)
                epics.put(set_action_opt_lnk0_pv, string.format("%s PP",action_pv))
                epics.put(set_action_opt_proc_pv, 1) -- manually force processing
            end
            print("")
        end
    end
    print(string.format("Path %d completed!", args.N))
end


function get_desc(args)
    wp_type = epics.get(string.format("%sPath%d:%d:Type", args.prefix, args.N, args.k))
    wp_type = (wp_type == 0.0) and "L" or "J"
    wp_num = epics.get(string.format("%sPath%d:%d:Number", args.prefix, args.N, args.k))
    if wp_num > 0 then
        wp = string.format("%sWaypoint%s:%d", args.prefix, wp_type, wp_num)
        return epics.get(wp)
    else
        return ""
    end
end
