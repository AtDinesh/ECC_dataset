% clear all;
% close all;

addpath('./Rotations');

traj_beforeOptim = load('./straight_walk/withoutOdom/traj_beforeOptim.dat');
traj_afterOptim = load('./straight_walk/withoutOdom/traj_afterOptim.dat');
checking_figures = load('./straight_walk/withoutOdom/KF_pose_stdev.dat');
kalman_pose_data = load('./straight_walk/kalman_pose.txt');
Mocap = load('./straight_walk/Mocap.txt');

% traj_beforeOptim = load('./8shape/withoutOdom/traj_beforeOptim.dat');
% traj_afterOptim = load('./8shape/withoutOdom/traj_afterOptim.dat');
% checking_figures = load('./8shape/withoutOdom/KF_pose_stdev.dat');
% kalman_pose_data = load('./8shape/kalman_pose.txt');
% Mocap = load('./8shape/Mocap.txt');

% traj_beforeOptim = load('./speedWalk/withoutOdom/traj_beforeOptim.dat');
% traj_afterOptim = load('./speedWalk/withoutOdom/traj_afterOptim.dat');
% checking_figures = load('./speedWalk/withoutOdom/KF_pose_stdev.dat');
% kalman_pose_data = load('./speedWalk/kalman_pose.txt');
% Mocap = load('./speedWalk/Mocap.txt');

% traj_beforeOptim = load('./curve/withoutOdom/traj_beforeOptim.dat');
% traj_afterOptim = load('./curve/withoutOdom/traj_afterOptim.dat');
% checking_figures = load('./curve/withoutOdom/KF_pose_stdev.dat');
% kalman_pose_data = load('./curve/kalman_pose.txt');
% Mocap = load('./curve/Mocap.txt');

t_before = traj_beforeOptim(:,1);
p_before = traj_beforeOptim(:,2:4);
q_before = traj_beforeOptim(:,5:8);
v_before = traj_beforeOptim(:,9:11);

t_after = traj_afterOptim(:,1);
p_after = traj_afterOptim(:,2:4);
q_after = traj_afterOptim(:,5:8);
v_after = traj_afterOptim(:,9:11);

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
KF_stdev = checking_figures(:,18:27);
ab_after = checking_figures(:,12:14);
wb_after = checking_figures(:,15:17);

kalman_ts = kalman_pose_data(:,1)*0.001;
kalman_p = kalman_pose_data(:,3:5);
kalman_o = kalman_pose_data(:,6:8);

Mocap_ts = Mocap(:,1);
Mocap_p = Mocap(:,2:4);
Mocap_o = Mocap(:,5:7);

fh=figure();
set(fh,'Name','Estimated position and velocity','NumberTitle','off');
subplot(2,2,1);
hold on;
plot(t_before, p_before(:,1), 'r');
plot(t_before, p_before(:,2), 'g');
plot(t_before, p_before(:,3), 'b');
xlabel('time (ms)');
ylabel('Estimated P');
legend('Px', 'Py', 'Pz');
title('position estimation before optimization wrt time');
grid

subplot(2,2,2);
hold on;
plot(t_after, p_after(:,1), 'r');
plot(t_after, p_after(:,2), 'g');
plot(t_after, p_after(:,3), 'b');
plot(KF_ts, est_KF(:,1), 'rd');
plot(KF_ts, est_KF(:,2), 'gd');
plot(KF_ts, est_KF(:,3), 'bd');
errorbar(KF_ts, est_KF(:,1), - 2*KF_stdev(:,1), 2*KF_stdev(:,1),'rx');
errorbar(KF_ts, est_KF(:,2), - 2*KF_stdev(:,2), 2*KF_stdev(:,2),'gx');
errorbar(KF_ts, est_KF(:,3), - 2*KF_stdev(:,3), 2*KF_stdev(:,3),'bx');
xlim([0,KF_ts(end)])
xlabel('time (ms)');
ylabel('Estimated P');
legend('Px', 'Py', 'Pz');
title('position estimation after optimization wrt time');
grid

subplot(2,2,3);
hold on;
plot(t_before, v_before(:,1), 'r');
plot(t_before, v_before(:,2), 'g');
plot(t_before, v_before(:,3), 'b');
xlabel('time (ms)');
ylabel('Estimated V');
legend('Vx', 'Vy', 'Vz');
title('Velocity estimation before optimization wrt time');
grid

subplot(2,2,4);
hold on;
plot(t_after, v_after(:,1), 'r');
plot(t_after, v_after(:,2), 'g');
plot(t_after, v_after(:,3), 'b');
xlabel('time (ms)');
ylabel('Estimated V');
legend('Vx', 'Vy', 'Vz');
title('Velocity estimation after optimization wrt time');
grid


%% plot position in 3D

% figure();
% subplot(1,2,1);
% plot3(p_before(:,1), p_before(:,2),p_before(:,3));
% subplot(1,2,2);
% plot3(p_after(:,1), p_after(:,2),p_after(:,3));
% xlabel('x (m)');
% ylabel('y (m)');
% zlabel('z (m)');

%% plot estimation results

fbias = figure(10);
subplot(2,2,1);
hold on;
plot(KF_ts, ab_after(:,1), 'r*');
plot(KF_ts, ab_after(:,2), 'g*');
plot(KF_ts, ab_after(:,3), 'b*');
plot(KF_ts, ab_after(:,1), 'r');
plot(KF_ts, ab_after(:,2), 'g');
plot(KF_ts, ab_after(:,3), 'b');
xlb1 = xlabel('time (s)','FontSize',17);
ylb1 = ylabel('Acc bias (m/s^2)','FontSize',17);
lgd1 = legend('Abx', 'Aby', 'Abz');
ttl1 = title('Acc Bias estimation wrt time','FontSize',25);
lgd1.FontSize = 15;
grid on;

subplot(2,2,2);
hold on;
plot(KF_ts, wb_after(:,1), 'r*');
plot(KF_ts, wb_after(:,2), 'g*');
plot(KF_ts, wb_after(:,3), 'b*');
plot(KF_ts, wb_after(:,1), 'r');
plot(KF_ts, wb_after(:,2), 'g');
plot(KF_ts, wb_after(:,3), 'b');
xlb2 = xlabel('time (s)','FontSize',17);
ylb2 = ylabel('Gyro bias (rad/s)','FontSize',17);
lgd2 = legend('Wbx', 'Wby', 'Wbz');
ttl2 = title('Gyro Bias estimation wrt time','FontSize',25);
lgd2.FontSize = 15;
grid on;

subplot(2,2,4);
hold on;
plot(t_after, o_after(:,1), 'r');
plot(t_after, o_after(:,2), 'g');
plot(t_after, o_after(:,3), 'b');
xlb3 = xlabel('time (s)','FontSize',17);
ylb3 = ylabel('orientation (rad)','FontSize',17);
lgd3 = legend('Ox after opt.', 'Oy after opt.', 'Oz after opt.');
ttl3 = title('Estimated orientation wrt time','FontSize',25);
lgd3.FontSize = 15;
grid on;

%% plot compare to ground truth and kalman
figure();
subplot(2,1,1);
hold on;
plot(kalman_ts,kalman_p(:,1), 'r');
plot(kalman_ts,kalman_p(:,2), 'g');
plot(kalman_ts,kalman_p(:,3), 'b');
legend('px', 'py', 'pz');
xlabel('timestamp');
ylabel('position (m)');
title('kalman position estimation (Holodeck)');

subplot(2,1,2);
hold on;
plot(kalman_ts,kalman_o(:,1), 'r');
plot(kalman_ts,kalman_o(:,2), 'g');
plot(kalman_ts,kalman_o(:,3), 'b');
legend('ox', 'oy', 'oz');
xlabel('timestamp');
ylabel('orientation (rad)');
title('kalman orientation estimation (Holodeck)');

figure();
subplot(2,1,1);
hold on;
plot(Mocap_ts,Mocap_p(:,1), 'r');
plot(Mocap_ts,Mocap_p(:,2), 'g');
plot(Mocap_ts,Mocap_p(:,3), 'b');
legend('px', 'py', 'pz');
xlabel('timestamp');
ylabel('position (m)');
title('Mocap Ground Truth Position (Holodeck)');

subplot(2,1,2);
hold on;
plot(Mocap_ts,Mocap_o(:,1), 'r');
plot(Mocap_ts,Mocap_o(:,2), 'g');
plot(Mocap_ts,Mocap_o(:,3), 'b');
legend('ox', 'oy', 'oz');
xlabel('timestamp');
ylabel('orientation (rad)');
title('Mocap Ground Truth Orientation (Holodeck)');

%% plot compare position to ground truth and kalman specifically

err_px = p_after(:,1) - Mocap_p(1:size(p_after,1),1);
err_py = p_after(:,2) - Mocap_p(1:size(p_after,1),2);
err_pz = p_after(:,3) - Mocap_p(1:size(p_after,1),3);
err_p = [err_px err_py err_pz];

kalman_err_p = kalman_p;
correspondency_mask = [];
for i=1:size(kalman_p,1)
    k_ts = kalman_ts(i);
    %n = find(Mocap_ts == kalman_ts(i));
    n = find(abs(Mocap_ts-kalman_ts(i))<=0.05,1,'first');
    if(isempty(n))
        kalman_err_p(i,1) = 0;
        kalman_err_p(i,2) = 0;
        kalman_err_p(i,3) = 0;
        correspondency_mask(i) = 0;
    else
        kalman_err_p(i,1) = kalman_p(i,1) - Mocap_p(n,1);
        kalman_err_p(i,2) = kalman_p(i,2) - Mocap_p(n,2);
        kalman_err_p(i,3) = kalman_p(i,3) - Mocap_p(n,3);
        correspondency_mask(i) = n;
    end
end    

correspondency_mask_est = [];
est_KF = [];
for i=1:size(KF_ts,1)
    n = find(abs(t_after-KF_ts(i))<=0.01,1,'last');
    if(isempty(n))
        est_KF(i,1) = 0;
        est_KF(i,2) = 0;
        est_KF(i,3) = 0;
        correspondency_mask_est(i) = 0;
    else
        est_KF(i,1) = p_after(n,1);
        est_KF(i,2) = p_after(n,2);
        est_KF(i,3) = p_after(n,3);
        correspondency_mask_est(i) = n;
    end
end
 

start_GT = find(Mocap_ts==t_after(1));
end_GT = find(abs(Mocap_ts-t_after(end))<=0.5,1,'first');
%end_GT = find(abs(Mocap_ts-t_after(end)) <= 0.01,1,'first');

start_kf = find(correspondency_mask~=0, 1, 'first');
%start_kf = find(kalman_ts==Mocap_ts(idx));
%end_kf = find(kalman_ts-Mocap_ts(end_GT), 1,'first');
end_kf = find(abs(kalman_ts-Mocap_ts(end_GT)) <= 0.5,1,'last');
%end_kf = find(kalman_ts==Mocap_ts(idx));


figure();
subplot(3,2,1);
hold on;
plot(KF_ts, est_KF(:,1), 'r*');
plot(t_after, p_after(:,1), 'r');
plot(kalman_ts(start_kf:end_kf),kalman_p(start_kf:end_kf,1), 'b*');
plot(Mocap_ts(start_GT:end_GT),Mocap_p(start_GT:end_GT,1), 'g');
legend('estimated KF','estimated px', 'kalman px', 'ground truth px');
xlabel('timestamp');
ylabel('px');
title('X position estimates VS Mocap ground truth');

subplot(3,2,3);
hold on;
plot(KF_ts, est_KF(:,2), 'r*');
plot(t_after, p_after(:,2), 'r');
plot(kalman_ts(start_kf:end_kf),kalman_p(start_kf:end_kf,2), 'b*');
plot(Mocap_ts(start_GT:end_GT),Mocap_p(start_GT:end_GT,2), 'g');
legend('estimated KF','estimated py', 'kalman py','ground truth py');
xlabel('timestamp');
ylabel('py');
title('Y position estimates VS Mocap ground truth');

subplot(3,2,5);
hold on;
plot(KF_ts, est_KF(:,3), 'r*');
plot(t_after, p_after(:,3), 'r');
plot(kalman_ts(start_kf:end_kf),kalman_p(start_kf:end_kf,3), 'b*');
plot(Mocap_ts(start_GT:end_GT),Mocap_p(start_GT:end_GT,3), 'g');
legend('estimated KF','estimated pz', 'kalman pz','ground truth pz');
xlabel('timestamp');
ylabel('pz');
title('Z position estimates VS Mocap ground truth');

subplot(3,2,2);
hold on;
plot(t_after, err_p(:,1), 'r');
plot(kalman_ts(start_kf:end_kf), kalman_err_p(start_kf:end_kf,1), 'b');
legend('estimation error', 'kalman error');
xlabel('timestamp');
ylabel('px err');
title('X position errors');

subplot(3,2,4);
hold on;
plot(t_after, err_p(:,2), 'r');
plot(kalman_ts(start_kf:end_kf), kalman_err_p(start_kf:end_kf,2), 'b');
legend('estimation error', 'kalman error');
xlabel('timestamp');
ylabel('py err');
title('Y position errors');

subplot(3,2,6);
hold on;
plot(t_after, err_p(:,3), 'r');
plot(kalman_ts(start_kf:end_kf), kalman_err_p(start_kf:end_kf,3), 'b');
legend('estimation error', 'kalman error');
xlabel('timestamp');
ylabel('pz err');
title('Z position errors');


figure()
hold on;
plot3(est_KF(:,1),est_KF(:,2),est_KF(:,3),'r');
plot3(Mocap_p(start_GT:end_GT,1),Mocap_p(start_GT:end_GT,2),Mocap_p(start_GT:end_GT,3),'g');
plot3(kalman_p(start_kf:end_kf,1),kalman_p(start_kf:end_kf,2),kalman_p(start_kf:end_kf,3),'b');
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
title('kalman plot3D');
legend('Wolf estimate', 'ground truth (Mocap)', 'kalman estimate');

%% plot compare orientation to ground truth and kalman specifically

% err_ox = o_after(:,1) - Mocap_o(1:size(p_after,1),1);
% err_oy = o_after(:,2) - Mocap_o(1:size(p_after,1),2);
% err_oz = o_after(:,3) - Mocap_o(1:size(p_after,1),3);
% err_o = [err_ox err_oy err_oz];
% 
% kalman_err_o = kalman_o;
% correspondency_mask = [];
% for i=1:size(kalman_o,1)
%     k_ts = kalman_ts(i);
%     n = find(Mocap_ts == kalman_ts(i));
%     if(isempty(n))
%         kalman_err_o(i,1) = 0;
%         kalman_err_o(i,2) = 0;
%         kalman_err_o(i,3) = 0;
%         correspondency_mask(i) = 0;
%     else
%         kalman_err_o(i,1) = kalman_o(i,1) - Mocap_o(n,1);
%         kalman_err_o(i,2) = kalman_o(i,2) - Mocap_o(n,2);
%         kalman_err_o(i,3) = kalman_o(i,3) - Mocap_o(n,3);
%         correspondency_mask(i) = n;
%     end
% end  
% 
% figure();
% subplot(3,2,1);
% hold on;
% plot(t_after, o_after(:,1), 'r');
% plot(kalman_ts(start_kf:end_kf),kalman_o(start_kf:end_kf,1), 'b');
% plot(Mocap_ts(start_GT:end_GT),Mocap_o(start_GT:end_GT,1), 'g');
% legend('estimated ox', 'kalman ox', 'ground truth ox');
% xlabel('timestamp');
% ylabel('ox (rad)');
% title('X orientation estimates VS Mocap ground truth');
% 
% subplot(3,2,3);
% hold on;
% plot(t_after, o_after(:,2), 'r');
% plot(kalman_ts(start_kf:end_kf),kalman_o(start_kf:end_kf,2), 'b');
% plot(Mocap_ts(start_GT:end_GT),Mocap_o(start_GT:end_GT,2), 'g');
% legend('estimated oy', 'kalman oy','ground truth oy');
% xlabel('timestamp');
% ylabel('oy (rad)');
% title('Y orientation estimates VS Mocap ground truth');
% 
% subplot(3,2,5);
% hold on;
% plot(t_after, o_after(:,3), 'r');
% plot(kalman_ts(start_kf:end_kf),kalman_o(start_kf:end_kf,3), 'b');
% plot(Mocap_ts(start_GT:end_GT),Mocap_o(start_GT:end_GT,3), 'g');
% legend('estimated oz', 'kalman oz','ground truth oz');
% xlabel('timestamp');
% ylabel('oz (rad)');
% title('Z orientation estimates VS Mocap ground truth');
% 
% subplot(3,2,2);
% hold on;
% plot(t_after, err_o(:,1), 'r');
% plot(kalman_ts(start_kf:end_kf), kalman_err_o(start_kf:end_kf,1), 'b');
% legend('estimation error', 'kalman error');
% xlabel('timestamp');
% ylabel('ox err (rad)');
% title('X orientation errors');
% 
% subplot(3,2,4);
% hold on;
% plot(t_after, err_o(:,2), 'r');
% plot(kalman_ts(start_kf:end_kf), kalman_err_o(start_kf:end_kf,2), 'b');
% legend('estimation error', 'kalman error');
% xlabel('timestamp');
% ylabel('oy err (rad)');
% title('Y orientation errors');
% 
% subplot(3,2,6);
% hold on;
% plot(t_after, err_o(:,3), 'r');
% plot(kalman_ts(start_kf:end_kf), kalman_err_o(start_kf:end_kf,3), 'b');
% legend('estimation error', 'kalman error');
% xlabel('timestamp');
% ylabel('oz err (rad)');
% title('Z orientation errors');
% 
% 
% figure()
% hold on;
% plot3(o_after(:,1),o_after(:,2),o_after(:,3),'r');
% plot3(Mocap_o(start_GT:end_GT,1),Mocap_o(start_GT:end_GT,2),Mocap_o(start_GT:end_GT,3),'g');
% plot3(kalman_o(start_kf:end_kf,1),kalman_o(start_kf:end_kf,2),kalman_o(start_kf:end_kf,3),'b');
% title('kalman plot3D');

%% 
set(ttl1,'FontSize',35);
set(ttl2,'FontSize',35);
set(ttl3,'FontSize',35);

set(xlb1,'FontSize', 25);
set(xlb2,'FontSize', 25);
set(xlb3,'FontSize', 25);

set(ylb1,'FontSize', 25);
set(ylb2,'FontSize', 25);
set(ylb3,'FontSize', 25);
% 
% set(zlb1,'FontSize', 18);
% set(zlb2,'FontSize', 18);
% set(zlb3,'FontSize', 18);

set(lgd1,'FontSize', 21);
set(lgd2,'FontSize', 21);
set(lgd3,'FontSize', 21);

tightfig(fbias);
% saveas(10, 'bias_orientation.eps', 'epsc');