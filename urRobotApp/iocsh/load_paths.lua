NPATHS = tonumber(N)
KMAX = tonumber(K)
for n = 1, NPATHS do
    for k = 1, KMAX do
	local macros = string.format("P=%s,N=%d,K=%d", P, n, k)
	dbLoadRecords("$(URROBOT)/urRobotApp/Db/path_waypoint.db", macros)
    end
end
for n = 1, NPATHS do
    local macros = string.format("P=%s,N=%d,KMAX=%d,DESC='Path %d'", P, n, KMAX, n)
    dbLoadRecords("$(URROBOT)/urRobotApp/Db/path.db", macros)
end
