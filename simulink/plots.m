time = out.ScopeData{1}.Values.Time;
command_position = out.ScopeData{1}.Values.Data(:,[1 3]);
actual_position = out.ScopeData{2}.Values.Data(:,[1 3]);
plot(time, command_position(:,1), "b:");
hold on
plot(time, command_position(:,2), "r:");
plot(time, actual_position(:,1), "b-");
plot(time, actual_position(:,2), "r-");
xlabel("Time (s)");
ylabel("\theta (rad)")
legend(["Commanded position - first joint","Commanded position - third joint","Actual position - first joint", "Actual position - third joint"])
title("Tracking accuracy with joint speed 0.25 rad/s")