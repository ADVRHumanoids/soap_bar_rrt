close all
clc

% resize the value vectors
l_sole_value_fx = l_sole_value_fx([length(l_sole_value_fx)-length(l_sole_reference_fx)+1:end]);
l_sole_value_fy = l_sole_value_fy([length(l_sole_value_fy)-length(l_sole_reference_fy)+1:end]);
l_sole_value_fz = l_sole_value_fz([length(l_sole_value_fz)-length(l_sole_reference_fz)+1:end]);
l_sole_value_tx = l_sole_value_tx([length(l_sole_value_tx)-length(l_sole_reference_tx)+1:end]);
l_sole_value_ty = l_sole_value_ty([length(l_sole_value_ty)-length(l_sole_reference_ty)+1:end]);
l_sole_value_tz = l_sole_value_tz([length(l_sole_value_tz)-length(l_sole_reference_tz)+1:end]);

r_sole_value_fx = r_sole_value_fx([length(r_sole_value_fx)-length(r_sole_reference_fx)+1:end]);
r_sole_value_fy = r_sole_value_fy([length(r_sole_value_fy)-length(r_sole_reference_fy)+1:end]);
r_sole_value_fz = r_sole_value_fz([length(r_sole_value_fz)-length(r_sole_reference_fz)+1:end]);
r_sole_value_tx = r_sole_value_tx([length(r_sole_value_tx)-length(r_sole_reference_tx)+1:end]);
r_sole_value_ty = r_sole_value_ty([length(r_sole_value_ty)-length(r_sole_reference_ty)+1:end]);
r_sole_value_tz = r_sole_value_tz([length(r_sole_value_tz)-length(r_sole_reference_tz)+1:end]);

l_ball_tip_value_fx = l_ball_tip_value_fx([length(l_ball_tip_value_fx)-length(l_ball_tip_reference_fx)+1:end]);
l_ball_tip_value_fy = l_ball_tip_value_fy([length(l_ball_tip_value_fy)-length(l_ball_tip_reference_fy)+1:end]);
l_ball_tip_value_fz = l_ball_tip_value_fz([length(l_ball_tip_value_fz)-length(l_ball_tip_reference_fz)+1:end]);
l_ball_tip_value_tx = l_ball_tip_value_tx([length(l_ball_tip_value_tx)-length(l_ball_tip_reference_tx)+1:end]);
l_ball_tip_value_ty = l_ball_tip_value_ty([length(l_ball_tip_value_ty)-length(l_ball_tip_reference_ty)+1:end]);
l_ball_tip_value_tz = l_ball_tip_value_tz([length(l_ball_tip_value_tz)-length(l_ball_tip_reference_tz)+1:end]);

r_ball_tip_value_fx = r_ball_tip_value_fx([length(r_ball_tip_value_fx)-length(r_ball_tip_reference_fx)+1:end]);
r_ball_tip_value_fy = r_ball_tip_value_fy([length(r_ball_tip_value_fy)-length(r_ball_tip_reference_fy)+1:end]);
r_ball_tip_value_fz = r_ball_tip_value_fz([length(r_ball_tip_value_fz)-length(r_ball_tip_reference_fz)+1:end]);
r_ball_tip_value_tx = r_ball_tip_value_tx([length(r_ball_tip_value_tx)-length(r_ball_tip_reference_tx)+1:end]);
r_ball_tip_value_ty = r_ball_tip_value_ty([length(r_ball_tip_value_ty)-length(r_ball_tip_reference_ty)+1:end]);
r_ball_tip_value_tz = r_ball_tip_value_tz([length(r_ball_tip_value_tz)-length(r_ball_tip_reference_tz)+1:end]);

% plot
figure(1)
plot(linspace(0, length(l_sole_value_fx)-1, length(l_sole_value_fx)), l_sole_value_fx, 'r', 'LineWidth', 2)
hold on
plot(linspace(0, length(l_sole_value_fy)-1, length(l_sole_value_fy)), l_sole_value_fy, 'b', 'LineWidth', 2)
plot(linspace(0, length(l_sole_value_fz)-1, length(l_sole_value_fz)), l_sole_value_fz, 'g', 'LineWidth', 2)
plot(linspace(0, length(l_sole_reference_fx)-1, length(l_sole_value_tx)), l_sole_reference_fx, 'r--', 'LineWidth', 2)
plot(linspace(0, length(l_sole_reference_fy)-1, length(l_sole_value_ty)), l_sole_reference_fy, 'b--', 'LineWidth', 2)
plot(linspace(0, length(l_sole_reference_fz)-1, length(l_sole_value_tz)), l_sole_reference_fz, 'g--', 'LineWidth', 2)
title('LSole')
legend('fx', 'fy', 'fz')

figure(2)
plot(linspace(0, length(r_sole_value_fx)-1, length(r_sole_value_fx)), r_sole_value_fx, 'r', 'LineWidth', 2)
hold on
plot(linspace(0, length(r_sole_value_fy)-1, length(r_sole_value_fy)), r_sole_value_fy, 'b', 'LineWidth', 2)
plot(linspace(0, length(r_sole_value_fz)-1, length(r_sole_value_fz)), r_sole_value_fz, 'g', 'LineWidth', 2)
plot(linspace(0, length(r_sole_reference_fx)-1, length(r_sole_value_tx)), r_sole_reference_fx, 'r--', 'LineWidth', 2)
plot(linspace(0, length(r_sole_reference_fy)-1, length(r_sole_value_ty)), r_sole_reference_fy, 'b--', 'LineWidth', 2)
plot(linspace(0, length(r_sole_reference_fz)-1, length(r_sole_value_tz)), r_sole_reference_fz, 'g--', 'LineWidth', 2)
title('RSole')
legend('fx', 'fy', 'fz')

figure(3)
plot(linspace(0, length(l_ball_tip_value_fx)-1, length(l_ball_tip_value_fx)), l_ball_tip_value_fx, 'r', 'LineWidth', 2)
hold on
plot(linspace(0, length(l_ball_tip_value_fy)-1, length(l_ball_tip_value_fy)), l_ball_tip_value_fy, 'b', 'LineWidth', 2)
plot(linspace(0, length(l_ball_tip_value_fz)-1, length(l_ball_tip_value_fz)), l_ball_tip_value_fz, 'g', 'LineWidth', 2)
plot(linspace(0, length(l_ball_tip_reference_fx)-1, length(l_ball_tip_value_tx)), l_ball_tip_reference_fx, 'r--', 'LineWidth', 2)
plot(linspace(0, length(l_ball_tip_reference_fy)-1, length(l_ball_tip_value_ty)), l_ball_tip_reference_fy, 'b--', 'LineWidth', 2)
plot(linspace(0, length(l_ball_tip_reference_fz)-1, length(l_ball_tip_value_tz)), l_ball_tip_reference_fz, 'g--', 'LineWidth', 2)
title('LBallTip')
legend('fx', 'fy', 'fz')

figure(4)
plot(linspace(0, length(r_ball_tip_value_fx)-1, length(r_ball_tip_value_fx)), r_ball_tip_value_fx, 'r', 'LineWidth', 2)
hold on
plot(linspace(0, length(r_ball_tip_value_fy)-1, length(r_ball_tip_value_fy)), r_ball_tip_value_fy, 'b', 'LineWidth', 2)
plot(linspace(0, length(r_ball_tip_value_fz)-1, length(r_ball_tip_value_fz)), r_ball_tip_value_fz, 'g', 'LineWidth', 2)
plot(linspace(0, length(r_ball_tip_reference_fx)-1, length(r_ball_tip_value_tx)), r_ball_tip_reference_fx, 'r--', 'LineWidth', 2)
plot(linspace(0, length(r_ball_tip_reference_fy)-1, length(r_ball_tip_value_ty)), r_ball_tip_reference_fy, 'b--', 'LineWidth', 2)
plot(linspace(0, length(r_ball_tip_reference_fz)-1, length(r_ball_tip_value_tz)), r_ball_tip_reference_fz, 'g--', 'LineWidth', 2)
title('RBallTip')
legend('fx', 'fy', 'fz')
