return {
    -- loads specified number of waypoints of both types
    load_waypoints = function(prefix, num)
	for n = 1, num do
	    local macros = string.format("P=%s,N=%d", prefix, n)
	    dbLoadRecords("$(URROBOT)/urRobotApp/Db/waypointJ.db", macros)
	    dbLoadRecords("$(URROBOT)/urRobotApp/Db/waypointL.db", macros)
	end
    end,

    load_waypoint_actions = function(prefix, num)
	for n = 1, num do
	    local macros = string.format("P=%s,N=%d", prefix, n)
	    dbLoadRecords("$(URROBOT)/urRobotApp/Db/waypoint_action.db", macros)
	end
    end,

    -- load npaths with kmax waypoints per path
    load_paths = function(prefix, npaths, kmax)
	for n = 1, npaths do
	    for k = 1, kmax do
		local macros = string.format("P=%s,N=%d,K=%d", prefix, n, k)
		dbLoadRecords("$(URROBOT)/urRobotApp/Db/path_waypoint.db", macros)
	    end
	end
	for n = 1, npaths do
	    local macros = string.format("P=%s,N=%d,KMAX=%d,DESC='Path %d'", prefix, n, kmax, n)
	    dbLoadRecords("$(URROBOT)/urRobotApp/Db/path.db", macros)
	end
    end
}

