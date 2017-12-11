%clear all;
%close all;

% This script is mad eto visualize the data from DLR's dataset (high precision with IMU on foot)
% @inproceedings{angermann2010high,
%   title={A high precision reference data set for pedestrian navigation using foot-mounted inertial sensors},
%   author={Angermann, Michael and Robertson, Patrick and Kemptner, Thomas and Khider, Mohammed},
%   booktitle={Indoor Positioning and Indoor Navigation (IPIN), 2010 International Conference on},
%   pages={1--6},
%   year={2010},
%   organization={IEEE}
% }

addpath('./Rotations');


%filenameIMU = '1stWalk_rectangles_020810_15_39/IMURaw_sync.txt';
%filenameMocap = '1stWalk_rectangles_020810_15_39/HolodeckOutput/1stWalk_rectangles_020810_15_39.txt';
%filename_zupt = '1stWalk_rectangles_020810_15_39/KalmanOutput/externalZuptsShoe1_b.txt';
%filename_kalman = '1stWalk_rectangles_020810_15_39/KalmanOutput/kalman_pose.txt';

% filenameIMU = '3rdWalk_straight_020810_16_41/IMURaw_sync.txt';
% filenameMocap = '3rdWalk_straight_020810_16_41/HolodeckOutput/3rdWalk_straight_sync.txt';
% filename_zupt = '3rdWalk_straight_020810_16_41/KalmanOutput/externalZuptsShoe1_b.txt';
% filename_kalman = '3rdWalk_straight_020810_16_41/KalmanOutput/kalman_pose.txt';

% filenameIMU = '4thWalk_8_020810_16_46/IMURaw_sync.txt';
% filenameMocap = '4thWalk_8_020810_16_46/HolodeckOutput/4thWalk_sync.txt';
% filename_zupt = '4thWalk_8_020810_16_46/KalmanOutput/externalZuptsShoe1_sync.txt';
% filename_kalman = '4thWalk_8_020810_16_46/KalmanOutput/kalman_pose.txt';

% filenameIMU = '5thWalk_straight_fast_020810_16_51/IMURaw_sync.txt';
% filenameMocap = '5thWalk_straight_fast_020810_16_51/HolodeckOutput/5thWalk_straight_fast_sync.txt';
% filename_zupt = '5thWalk_straight_fast_020810_16_51/KalmanOutput/externalZuptsShoe1_sync.txt';
% filename_kalman = '5thWalk_straight_fast_020810_16_51/KalmanOutput/Holodeck-ShoeOdometry-1-Thu_Sep_09_17-29-53_CEST_2010.txt';
 
filenameIMU = '6thWalk_onTable_030810_15_14/IMURaw_sync.txt';
filenameMocap = '6thWalk_onTable_030810_15_14/HolodeckOutput/6thWalk_onTable_sync.txt';
filename_zupt = '6thWalk_onTable_030810_15_14/KalmanOutput/externalZuptsShoe1_sync.txt';
filename_kalman = '6thWalk_onTable_030810_15_14/KalmanOutput/Holodeck-ShoeOdometry-1-Thu_Sep_09_17-31-46_CEST_2010.txt';
 
IMUdata = load(filenameIMU);
MOCAPdata = load(filenameMocap);
zupt = load(filename_zupt);
kalman_pose = load(filename_kalman);

IMU_ts = IMUdata(:,3);
IMU_acc = IMUdata(:,4:6);
IMU_gyro = IMUdata(:,7:9);

Mocap_ts = MOCAPdata(:,1);
Mocap_pos = MOCAPdata(:,3:5)*0.001;
M_ori = MOCAPdata(:,6:14);

Mocap_ori = [];
for i=1:size(M_ori,1)
    R = [M_ori(i,1:3); M_ori(i,4:6); M_ori(i,7:9)];
    Mocap_ori = [Mocap_ori; q2v((R2q(R)))'];
end

Zupt_ts = zupt(:,1);
Zupt_out = 1*5;

fullZupt_ts = zupt(1,1)-1 : 0.01 : zupt(end,1);
fullZupt_data = zeros(int16(size(fullZupt_ts,2)),1);

j_1 = 1;
j_2 = 0;
for i = fullZupt_ts(1,1):0.0100:fullZupt_ts(1,end)
    j_2 = j_2 + 1;
    if(i == Zupt_ts(j_1,1))
        fullZupt_data(j_2,1) = 10;
        j_1 = j_1 + 1;
    else
        fullZupt_data(j_2,1) = 0;
    end
end

kalman_ts = kalman_pose(:,1)*0.001;
kalman_p = kalman_pose(:,3:5);
kalman_o = kalman_pose(:,6:8);

Mocap_output_ts = IMU_ts(1,1):0.01:(IMU_ts(1,1)+size(Mocap_ts,1)*0.01 - 0.01);
Mocap_out = [Mocap_output_ts' Mocap_pos Mocap_ori];
IMU_out = [IMU_ts IMU_acc IMU_gyro];

figure;
subplot(3,1,1);
hold on;
plot(IMU_ts, IMU_acc(:,1), '--r');
plot(IMU_ts, IMU_acc(:,2), '--g');
plot(IMU_ts, IMU_acc(:,3), '--b');
plot(IMU_ts, IMU_gyro(:,1),'-.r');
plot(IMU_ts, IMU_gyro(:,2),'-.g');
plot(IMU_ts, IMU_gyro(:,3),'-.b');
plot(Zupt_ts, 10, 'm*');
legend('ax', 'ay', 'az', 'wx', 'wy', 'wz', 'zupt');

subplot(3,1,2);
hold on;
plot(Mocap_output_ts,Mocap_pos(:,1), 'r');
plot(Mocap_output_ts,Mocap_pos(:,2), 'g');
plot(Mocap_output_ts,Mocap_pos(:,3), 'b');
plot(Zupt_ts, 0, 'm*');
legend('px', 'py', 'pz', 'zupt');

subplot(3,1,3);
hold on;
plot(Mocap_output_ts,Mocap_ori(:,1), 'r');
plot(Mocap_output_ts,Mocap_ori(:,2), 'g');
plot(Mocap_output_ts,Mocap_ori(:,3), 'b');
legend('ox', 'oy', 'oz');

figure();
subplot(2,1,1);
hold on;
plot(kalman_ts,kalman_p(:,1), 'r');
plot(kalman_ts,kalman_p(:,2), 'g');
plot(kalman_ts,kalman_p(:,3), 'b');
legend('px', 'py', 'pz');
subplot(2,1,2);
hold on;
plot(kalman_ts,kalman_o(:,1), 'r');
plot(kalman_ts,kalman_o(:,2), 'g');
plot(kalman_ts,kalman_o(:,3), 'b');
legend('ox', 'oy', 'oz');

figure();
plot3(Mocap_pos(:,1), Mocap_pos(:,2),Mocap_pos(:,3));
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');

save('6thWalk_onTable_030810_15_14/HolodeckOutput/3rdWalk_straight_syncIMU.txt', 'Mocap_out', '-ASCII','-double');
save('6thWalk_onTable_030810_15_14/KalmanOutput/externalZupts_sync.txt', 'kalman_ts', '-ASCII','-double');
save('6thWalk_onTable_030810_15_14/IMURaw_sync_b.txt', 'IMU_out', '-ASCII','-double');