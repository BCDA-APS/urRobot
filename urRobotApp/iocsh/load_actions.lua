for n = 1, tonumber(N) do
    local macros = string.format("P=%s,N=%d", P, n)
    dbLoadRecords("$(URROBOT)/urRobotApp/Db/waypoint_action.db", macros)
end
