close all

good = sum(success == 1);
bad = sum(success == 0);

average = sum(time) / length(time);

plot([1:length(time)], time)

plot([1:length(time_collision_check)], time_collision_check, 'b')
hold on
plot([1:length(time_centroidal_statics_check)], time_centroidal_statics_check, 'r')
plot([1:length(time_solve)], time_solve, 'g')
legend('collision check', 'centroidal statics check', 'IK solve')

disp(["success rate is: ", good/length(success)*100])
disp(["average: ", average])