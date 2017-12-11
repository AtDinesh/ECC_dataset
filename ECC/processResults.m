% clear all;
% close all;

addpath('./Rotations');
g = [0 0 9.806];
v = [-pi/5 3.9*pi/5 0];
v = [pi/5 6*pi/5 0];
g_rot = [-5.8 -1.37 -7.8]
R = q2R(v2q(v));

g * inv(R)

Rv = vrrotvec2mat(vrrotvec(g,g_rot))
v_r = q2v(R2q(Rv))

g * q2R(v2q([2.884 0.7173 -1.011]))
%% process data old

% straight_walk_workspace_path = './straight_walk/workspace.mat';
% Eightshape_workspace_path = './8shape/workspace.mat';
% speedWalk_workspace_path = './speedWalk/workspace.mat';
% curve_workspace_path = './curve/workspace.mat';

%needed variables : kalman_err_p, kalman_err_o, err_p, err_o,
%KF_stdev,start_kf, end_kf, 

% straight_walk_workspace = load(straight_walk_workspace_path,'kalman_err_p', 'kalman_err_o', 'err_p', 'err_o', 'KF_stdev', 'start_kf', 'end_kf');
% Eightshape_workspace = load(Eightshape_workspace_path,'kalman_err_p', 'kalman_err_o', 'err_p', 'err_o', 'KF_stdev', 'start_kf', 'end_kf');
% speedWalk_workspace = load(speedWalk_workspace_path,'kalman_err_p', 'kalman_err_o', 'err_p', 'err_o', 'KF_stdev', 'start_kf', 'end_kf');
% curve_workspace = load(curve_workspace_path,'kalman_err_p', 'kalman_err_o', 'err_p', 'err_o', 'KF_stdev', 'start_kf', 'end_kf');

% figure()
% subplot(3,2,1);
% histogram(straight_walk_workspace.kalman_err_p(:,1), 20);
% title('histogram error in straight walk case : px (kalman)');
% subplot(3,2,2);
% histogram(straight_walk_workspace.err_p(:,1), 20);
% title('histogram error in straight walk case : px (graph)');
% subplot(3,2,3);
% histogram(straight_walk_workspace.kalman_err_p(:,2), 20);
% title('histogram error in straight walk case : py (kalman)');
% subplot(3,2,4);
% histogram(straight_walk_workspace.err_p(:,2), 20);
% title('histogram error in straight walk case : py (graph)');
% subplot(3,2,5);
% histogram(straight_walk_workspace.kalman_err_p(:,3), 20);
% title('histogram error in straight walk case : pz (kalman)');
% subplot(3,2,6);
% histogram(straight_walk_workspace.err_p(:,3), 20);
% title('histogram error in straight walk case : pz (graph)');

%% process data new
clear all;
close all;
addpath('./Rotations');
straight_walk_workspace_path = './8shape/workspace.mat';

%needed variables : est_KF, p_after, o_after, start_GT, end_GT, Mocap_p, kalman_p, start_kf, end_kf,
%                   Mocap_o, kalman_o, t_after

straight_walk_workspace = load(straight_walk_workspace_path,'kalman_p', 'kalman_o', 'p_after', 'o_after', 'est_KF', 'Mocap_p', 'Mocap_o', 'start_kf', 'end_kf', 'start_GT', 'end_GT', 't_after');
start_GT = straight_walk_workspace.start_GT;
end_GT = straight_walk_workspace.end_GT;
start_kf = straight_walk_workspace.start_kf;
end_kf = straight_walk_workspace.end_kf;
%get translation between origin KF compared to mocap
kalman_to_mocap_tr = straight_walk_workspace.Mocap_p(1,:) - straight_walk_workspace.kalman_p(start_kf,:);
wolf_to_mocap_tr = straight_walk_workspace.Mocap_p(1,:) - straight_walk_workspace.est_KF(1,:);

kalman_p_tr = straight_walk_workspace.kalman_p;
tmp =  ones(size(kalman_p_tr,1), size(kalman_p_tr,2)) ;

tmp(:,1) = kalman_to_mocap_tr(1);
tmp(:,2) = kalman_to_mocap_tr(2);
tmp(:,3) = kalman_to_mocap_tr(3);

kalman_p_tr = straight_walk_workspace.kalman_p + tmp;

wolf_p_tr = straight_walk_workspace.est_KF;
tmp =  ones(size(wolf_p_tr,1), size(wolf_p_tr,2));

tmp(:,1) = wolf_to_mocap_tr(1);
tmp(:,2) = wolf_to_mocap_tr(2);
tmp(:,3) = wolf_to_mocap_tr(3);

wolf_p_tr = straight_walk_workspace.est_KF + tmp;

figure()
hold on;
plot3(wolf_p_tr(:,1),wolf_p_tr(:,2),wolf_p_tr(:,3),'r');
plot3(straight_walk_workspace.Mocap_p(start_GT:end_GT,1),straight_walk_workspace.Mocap_p(start_GT:end_GT,2),straight_walk_workspace.Mocap_p(start_GT:end_GT,3),'g');
plot3(kalman_p_tr(start_kf:end_kf,1),kalman_p_tr(start_kf:end_kf,2),kalman_p_tr(start_kf:end_kf,3),'b');
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
title('kalman plot3D');
legend('Wolf estimate', 'ground truth (Mocap)', 'kalman estimate');

%get orientation between origin KF compared to mocap

%kalman_rotate = qProd(v2q(straight_walk_workspace.Mocap_o(1,:)), q2qc(v2q(straight_walk_workspace.kalman_o(start_kf+4,:))));
kalman_rotate = v2q([0 0 4*pi/6]);
for i=1:1:size(kalman_p_tr,1)
    kalman_p_tr(i,:) = qRot(kalman_p_tr(i,:)',kalman_rotate);
end

wolf_rotate = v2q([0 0 9*pi/7]);
for i=1:1:size(wolf_p_tr,1)
    wolf_p_tr(i,:) = qRot(wolf_p_tr(i,:)',wolf_rotate);
end


f_est3D = figure(10);
hold on;
plot3(wolf_p_tr(:,1),wolf_p_tr(:,2),wolf_p_tr(:,3),'r*');
plot3(straight_walk_workspace.Mocap_p(start_GT:end_GT,1),straight_walk_workspace.Mocap_p(start_GT:end_GT,2),straight_walk_workspace.Mocap_p(start_GT:end_GT,3),'g');
plot3(kalman_p_tr(start_kf:end_kf,1),kalman_p_tr(start_kf:end_kf,2),kalman_p_tr(start_kf:end_kf,3),'b');
plot3(wolf_p_tr(:,1),wolf_p_tr(:,2),wolf_p_tr(:,3),'r');
xlb1 = xlabel('x (m)');
ylb1 = ylabel('y (m)');
zlb1 = zlabel('z (m)');
ttl1 = title('position estimation - 3D view');
lgd1 = legend('Graph estimate', 'ground truth (Mocap)', 'kalman estimate');
grid on;

figure();
plot3(straight_walk_workspace.p_after(411:end,1),straight_walk_workspace.p_after(411:end,2),straight_walk_workspace.p_after(411:end,3),'r');

figure();
plot(straight_walk_workspace.t_after(411:end),straight_walk_workspace.p_after(411:end,3),'r');

f_est = figure(11);
subplot(1,2,1);
hold on;
plot(wolf_p_tr(:,1),wolf_p_tr(:,2),'r*');
plot(straight_walk_workspace.Mocap_p(start_GT:end_GT,1),straight_walk_workspace.Mocap_p(start_GT:end_GT,2),'g');
plot(kalman_p_tr(start_kf:end_kf,1),kalman_p_tr(start_kf:end_kf,2),'b*');

plot(wolf_p_tr(:,1),wolf_p_tr(:,2),'r');
plot(kalman_p_tr(start_kf:end_kf,1),kalman_p_tr(start_kf:end_kf,2),'b');
xlb2 = xlabel('x (m)');
ylb2 = ylabel('y (m)');
ttl2 = title('Position estimation (XY-view)');
lgd2 = legend('Graph estimate (keyframes)', 'ground truth (Mocap)', 'kalman estimate');
grid on;

subplot(1,2,2);
hold on;
plot(wolf_p_tr(:,1),wolf_p_tr(:,3),'r*');
plot(straight_walk_workspace.Mocap_p(start_GT:end_GT,1),straight_walk_workspace.Mocap_p(start_GT:end_GT,3),'g');
plot(kalman_p_tr(start_kf:end_kf,1),kalman_p_tr(start_kf:end_kf,3),'b*');

plot(wolf_p_tr(:,1),wolf_p_tr(:,3),'r');
plot(kalman_p_tr(start_kf:end_kf,1),kalman_p_tr(start_kf:end_kf,3),'b');
xlb3 = xlabel('x (m)');
ylb3 = ylabel('z (m)');
ttl3 = title('Position estimation (XZ-view)');
lgd3 = legend('Graph estimate (keyframes)', 'ground truth (Mocap)', 'kalman estimate');
grid on;

%% 

set(ttl1,'FontSize',30);
set(ttl2,'FontSize',30);
set(ttl3,'FontSize',30);

set(xlb1,'FontSize', 29);
set(xlb2,'FontSize', 29);
set(xlb3,'FontSize', 29);

set(ylb1,'FontSize', 29);
set(ylb2,'FontSize', 29);
set(ylb3,'FontSize', 29);

set(zlb1,'FontSize', 29);
% set(zlb2,'FontSize', 25);
% set(zlb3,'FontSize', 25);

set(lgd1,'FontSize', 22);
set(lgd2,'FontSize', 22);
set(lgd3,'FontSize', 22);

tightfig(f_est);

%%
saveas(11, '8_XY_XZ_viewsRotated.eps', 'epsc');

