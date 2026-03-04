for n = 1, tonumber(N) do
    local macros = string.format("P=%s,N=%d", P, n)
    dbLoadRecords("$(URROBOT)/urRobotApp/Db/waypointJ.db", macros)
    dbLoadRecords("$(URROBOT)/urRobotApp/Db/waypointL.db", macros)
end
