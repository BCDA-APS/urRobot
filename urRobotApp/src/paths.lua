-- NOTE: IOC must add $(URROBOT)/urRobotApp/src to LUA_SCRIPT_PATH
epics = require("epics")

function wait_busy(busy_pv)
    if timeout == nil then
        timeout = 300.0 -- seconds
    end

    t0 = os.time()
    while true do
        local elap = (os.time() - t0)
        if (elap >= timeout) then
            print("Error: Timeout exceeded waiting for process to complete")
            break
        end
        busy = epics.get(busy_pv)
        if busy == 0 then
            break
        end
    end
end

-- Moves robot through all enabled points along path
function path_go(args)

    local done_pv = string.format("%sControl:AsyncMoveDone.RVAL", args.prefix)
    local path_stop_pv = string.format("%sPath%d:Stop", args.prefix, args.N)
    local sync_joint_disa_pv = string.format("%sControl:sync_joint_cmd.DISA", args.prefix)
    local sync_pose_disa_pv = string.format("%sControl:sync_pose_cmd.DISA", args.prefix)

    -- disable automatic syncing of joint and pose command values
    epics.put(sync_joint_disa_pv, 1)
    epics.put(sync_pose_disa_pv, 1)

    epics.put(path_stop_pv, 0)
    local stopped = 0

    for i = 1,args.kmax do

        -- check if Path$(N):Stop is called
        local path_stop_val = epics.get(path_stop_pv)
        if path_stop_val == 1 then
            stopped = 1
            break
        end

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

            -- Apply action override
            local action_pv
            local wp_action0
            local action_opt_val_pv
            local set_action_opt_lnk0_pv
            local set_action_opt_proc_pv
            if wp_action ~= 0 then
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
            print(string.format("Moving to waypoint %s...", wp))
            move_pv = string.format("%s:move%s.PROC", wp, wp_type)
            wp_busy_pv = string.format("%s:Busy", wp)
            epics.put(move_pv, 1)
            epics.put(wp_busy_pv, 1)
            wait_busy(wp_busy_pv)

            -- Restore original action
            if wp_action ~= 0 then -- TODO: check this works, was 3?
                epics.put(action_opt_val_pv, wp_action0)
                epics.put(set_action_opt_lnk0_pv, string.format("%s PP",action_pv))
                epics.put(set_action_opt_proc_pv, 1) -- manually force processing
            end
        end
    end
    if stopped then
        print(string.format("Path %d stopped", args.N))
    else
        print(string.format("Path %d completed!", args.N))
    end

    -- re-enable automatic syncing of joint and pose command values
    epics.put(sync_joint_disa_pv, 0)
    epics.put(sync_pose_disa_pv, 0)
end


function get_wp_info(args)
    local wp_type = epics.get(string.format("%sPath%d:%d:Type", args.prefix, args.N, args.k))
    wp_type = (wp_type == 0.0) and "L" or "J"
    local wp_num = epics.get(string.format("%sPath%d:%d:Number", args.prefix, args.N, args.k))
    if wp_num > 0 then
        local default_action_num = epics.get(string.format("%sWaypoint%s:%d:ActionOpt", args.prefix, wp_type, wp_num))
        local override_action_num = epics.get(string.format("%sPath%d:%d:ActionOverride", args.prefix, args.N, args.k))
        local action_desc
        if override_action_num == 0 then
            action_desc = epics.get(string.format("%sWaypoint%s:%d:ActionDesc", args.prefix, wp_type, wp_num))
        else
            action_desc = epics.get(string.format("%sActionLink%d.DESC", args.prefix, override_action_num))
        end
        epics.put(string.format("%sPath%d:%d:ActionDesc", args.prefix, args.N, args.k), action_desc)

        local wp_pv = string.format("%sWaypoint%s:%d", args.prefix, wp_type, wp_num)
        local reached_inp_pv = string.format("%sPath%d:%d:Reached.INP",args.prefix,args.N,args.k)
        local wp_reached_pv = string.format("%s:Reached CP",wp_pv)
        epics.put(reached_inp_pv, wp_reached_pv) -- set link to Waypoint:Reached PV
        return epics.get(wp_pv)
    else
        epics.put(string.format("%sPath%d:%d:ActionDesc", args.prefix, args.N, args.k), "")
        return ""
    end
end
