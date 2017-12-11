% clear all;
% close all;
figure(1)
clf

addpath('./Rotations');

traj_beforeOptim = load('./imu_dock_beforeOptim.dat');
traj_afterOptim = load('./imu_dock_afterOptim.dat');
checking_figures = load('./KF_pose_stdev.dat');

% traj_beforeOptim = load('./imu_beforeOptim_mocap.dat');
% traj_afterOptim = load('./imu_afterOptim_mocap.dat');
% checking_figures = load('./KF_pose_stdev_mocap.dat');

t_before = traj_beforeOptim(:,1);
p_before = traj_beforeOptim(:,2:4);
q_before = traj_beforeOptim(:,5:8);
v_before = traj_beforeOptim(:,9:11);
ab_before = traj_beforeOptim(:,12:14);
wb_before = traj_beforeOptim(:,15:17);

t_after = traj_afterOptim(:,1);
p_after = traj_afterOptim(:,2:4);
q_after = traj_afterOptim(:,5:8);
v_after = traj_afterOptim(:,9:11);
ab_after = traj_afterOptim(:,12:14);
wb_after = traj_afterOptim(:,15:17);

o_before = [];
q_before_mat = q_before;
q_before_mat(:,1) = q_before(:,4);
q_before_mat(:,2) = q_before(:,1);
q_before_mat(:,3) = q_before(:,2);
q_before_mat(:,4) = q_before(:,3);

o_after = [];
q_after_mat = q_after;
q_after_mat(:,1) = q_after(:,4);
q_after_mat(:,2) = q_after(:,1);
q_after_mat(:,3) = q_after(:,2);
q_after_mat(:,4) = q_after(:,3);

for i=1:1:size(q_before_mat,1)
    o_before = [o_before; q2v(q_before_mat(i,:)')'];
    o_after = [o_after; q2v(q_after_mat(i,:)')'];
end

KF_ts    = checking_figures(:,1);
est_KF   = checking_figures(:,2:17);
KF_stdev = checking_figures(:,18:33);

exp_KF1 = [0 0 0 0 0 0 1];
exp_final = [0 -0.06 0 0 0 0 1];
quat = traj_afterOptim(size(traj_afterOptim,1),[8 5 6 7]);

exp_final(1:3) = qRot(exp_final(1:3)',quat);


fh=figure(1);
set(fh,'Name','Estimated position and velocity','NumberTitle','off');
subplot(2,2,1);
hold on;
plot(t_before, p_before(:,1), 'r');
plot(t_before, p_before(:,2), 'g');
plot(t_before, p_before(:,3), 'b');
plot(t_before(size(t_before,1)), exp_final(1), 'r*');
plot(t_before(size(t_before,1)), exp_final(2), 'g*');
plot(t_before(size(t_before,1)), exp_final(3), 'b*');
xlabel('time (ms)');
ylabel('Estimated P');
% legend('Px before optim', 'Py  before optim', 'Pz  before optim', 'expected PX_{KF2}', 'expected PY_{KF2}', 'expected PZ_{KF2}');
title('position estimation before optimization wrt time');
grid

subplot(2,2,2);
hold on;
plot(t_after, p_after(:,1), 'r');
plot(t_after, p_after(:,2), 'g');
plot(t_after, p_after(:,3), 'b');
plot(t_after(size(t_after,1)), exp_final(1), 'r*');
plot(t_after(size(t_after,1)), exp_final(2), 'g*');
plot(t_after(size(t_after,1)), exp_final(3), 'b*');
plot(KF_ts, est_KF(:,1), 'rd');
plot(KF_ts, est_KF(:,2), 'gd');
plot(KF_ts, est_KF(:,3), 'bd');
errorbar(KF_ts, est_KF(:,1), - 2*KF_stdev(:,1), 2*KF_stdev(:,1),'rx');
errorbar(KF_ts, est_KF(:,2), - 2*KF_stdev(:,2), 2*KF_stdev(:,2),'gx');
errorbar(KF_ts, est_KF(:,3), - 2*KF_stdev(:,3), 2*KF_stdev(:,3),'bx');
xlim([0,KF_ts(end)])
xlabel('time (ms)');
ylabel('Estimated P');
% legend('Px after optim', 'Py after optim', 'Pz after optim', 'expected PX_{KF2}', 'expected PY_{KF2}', 'expected PZ_{KF2}', 'estimated PX_{KF2}', 'estimated PY_{KF2}', 'estimated PZ_{KF2}');
title('position estimation after optimization wrt time');
grid

subplot(2,2,3);
hold on;
plot(t_before, v_before(:,1), 'r');
plot(t_before, v_before(:,2), 'g');
plot(t_before, v_before(:,3), 'b');
xlabel('time (ms)');
ylabel('Estimated V');
% legend('Vx before optim', 'Vy  before optim', 'Vz  before optim');
title('Velocity estimation before optimization wrt time');
grid

subplot(2,2,4);
hold on;
plot(t_after, v_after(:,1), 'r');
plot(t_after, v_after(:,2), 'g');
plot(t_after, v_after(:,3), 'b');
xlabel('time (ms)');
ylabel('Estimated V');
% title('Velocity estimation after optimization wrt time');
grid

% fh=figure(2);
% set(fh,'Name','Estimated quaternion','NumberTitle','off');
% subplot(2,1,1);
% hold on;
% plot(t_before, 2*q_after(:,1), 'r');
% plot(t_before, 2*q_after(:,2), 'g');
% plot(t_before, 2*q_after(:,3), 'b');
% xlabel('time (ms)');
% ylabel('Estimated Q');
% % legend('Qx before optim', 'Qy  before optim', 'Qz  before optim', 'expected PX_{KF1}', 'expected PY_{KF1}', 'expected PZ_{KF1}');
% title('quaternion estimation after optimization wrt time');
% grid

%% plot biases

figure();
subplot(3,1,1);
hold on;
plot(t_after, ab_after(:,1), 'r');
plot(t_after, ab_after(:,2), 'g');
plot(t_after, ab_after(:,3), 'b');
xlabel('time (ms)');
ylabel('Acc bias (m/s^2)');
legend('Abx', 'Aby', 'Abz');
title('Acc Bias estimation wrt time');

subplot(3,1,2);
hold on;
plot(t_after, wb_after(:,1), 'r');
plot(t_after, wb_after(:,2), 'g');
plot(t_after, wb_after(:,3), 'b');
xlabel('time (ms)');
ylabel('Gyro bias (rad/s)');
legend('Wbx', 'Wby', 'Wbz');
title('Gyro Bias estimation wrt time');

subplot(3,1,3);
hold on;
plot(t_after, o_after(:,1), 'r');
plot(t_after, o_after(:,2), 'g');
plot(t_after, o_after(:,3), 'b');
plot(t_after, o_before(:,1), 'r--');
plot(t_after, o_before(:,2), 'g--');
plot(t_after, o_before(:,3), 'b--');
xlabel('time (ms)');
ylabel('orientation (rad)');
legend('Ox', 'Oy', 'Oz', 'refOx', 'refOy', 'refOz');
title('Orientation wrt time');