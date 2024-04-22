%% Rock Climbing Continuous Relative Phase Angles
%% Prepare the Workspace and Initialize Variables
clear all; close all; clc;
FiltOrd = 2;
LowPassCutOff = 100;
SampleRate = 2000;
NyquistFreq = SampleRate/2;
Percentage = 1:1:100;
OutputExt = '.xlsx';
%% Read and Separate Data
[FileName, PathName] = uigetfile('*.csv');
    TrialName = FileName;  TrialName(end-3:end) = [];    
    FileLocation = strcat(PathName,FileName);
        RawData = readmatrix(FileLocation);
            Time = RawData(:,1);
            LinAcc = [RawData(:,2:4),RawData(:,8:10),RawData(:,14:16),RawData(:,20:22),RawData(:,26:28),RawData(:,32:34),RawData(:,38:40),RawData(:,44:46)];
            AngVel = [RawData(:,5:7),RawData(:,11:13),RawData(:,17:19),RawData(:,23:25),RawData(:,29:31),RawData(:,35:37),RawData(:,41:43),RawData(:,47:49)];
%% Design and Apply Low Pass Filter
[FiltB,FiltA] = butter(FiltOrd,LowPassCutOff/NyquistFreq);
FiltLinAcc = filtfilt(FiltB,FiltA,LinAcc);
FiltAngVel = filtfilt(FiltB,FiltA,AngVel);
%% Take Derivatives and Integrals
LinJerk = diff(FiltLinAcc);
    LinJerk(end+1,:) = LinJerk(end,:);
        LinJerk = LinJerk*SampleRate;
AngDis = cumtrapz(Time,FiltAngVel);
    FiltAngDis = filtfilt(FiltB,FiltA,AngDis);
%% Calculate Joint Kinematics
% Upper Extremity
% Wrist Flexion/Extension
WristFlexExtAng = FiltAngDis(:,14)-FiltAngDis(:,11); % rotating about the Y axis 
WristFlexExtAngVel = FiltAngVel(:,14)-FiltAngVel(:,11);
% Wrist Radial/Ulnar Deviation
WristRadUlnDevAng = FiltAngDis(:,15)-FiltAngDis(:,12);% rotating about the Z axis
WristRadUlnDevAngVel = FiltAngVel(:,15)-FiltAngVel(:,12);
% Elbow Flexion/Extension
ElbowFlexExtAng = FiltAngDis(:,12)-FiltAngDis(:,9); % rotating about the Z axis
ElbowFlexExtAngVel = FiltAngVel(:,12)-FiltAngVel(:,9);
% Shoulder Flexion/Extension
ShoulderFlexExtAng = FiltAngDis(:,8)-FiltAngDis(:,5); % rotating about the Y axis
ShoulderFlexExtAngVel = FiltAngVel(:,8)-FiltAngVel(:,5);
% Shoulder Abduction/Adduction
ShoulderAbAddAng = FiltAngDis(:,9)-FiltAngDis(:,6); % rotating about the Z axis
ShoulderAbAddAngVel = FiltAngVel(:,9)-FiltAngVel(:,6);
% Trunk Flexion/Extension
TrunkFlexExtAng = FiltAngDis(:,5)-FiltAngDis(:,2); % rotating about the Y axis
TrunkFlexExtAngVel = FiltAngVel(:,5)-FiltAngVel(:,2);
% Trunk Lateral Flexion
TrunkLatFlexAng = FiltAngDis(:,6)-FiltAngDis(:,3); % rotating about the Z axis
TrunkLatFlexAngVel = FiltAngVel(:,6)-FiltAngVel(:,3);
% Lower Extremity
% Hip Flexion/Extension
HipFlexExtAng = FiltAngDis(:,17)-FiltAngDis(:,2); % rotating about the Y axis
HipFlexExtAngVel = FiltAngVel(:,17)-FiltAngVel(:,2);
% Hip Abduction/Adduction
HipAbAddAng = FiltAngDis(:,18)-FiltAngDis(:,3); % rotating about the Z axis
HipAbAddAngVel = FiltAngVel(:,18)-FiltAngVel(:,3);
% Knee Flexion/Extension
KneeFlexExtAng = FiltAngDis(:,20)-FiltAngDis(:,17);% rotating about the Y axis
KneeFlexExtAngVel = FiltAngVel(:,20)-FiltAngVel(:,17);
% Knee Internal/External Rotation
KneeIntExtRotAng = FiltAngDis(:,16)-FiltAngDis(:,19); %rotation about the X axis
KneeIntExtRotAngVel = FiltAngVel(:,16)-FiltAngVel(:,19);
% Ankle Plantarflexion/Dorsiflexion
AnklePlantDorsAng = FiltAngDis(:,24)-FiltAngDis(:,20);% rotating about the Z axis on foot and Y axis on the lower leg
AnklePlantDorsAngVel = FiltAngVel(:,24)-FiltAngVel(:,20);
% Ankle Inversion/Eversion
AnkleInvEvAng = FiltAngDis(:,22)-FiltAngDis(:,21); % rotating about the X axis on the foot and Z axis on the lower leg
AnkleInvEvAngVel = FiltAngVel(:,22)-FiltAngVel(:,21);
% Create a figure with subplots
figure;

%% Create a figure with subplots
figure;

% Plot Shoulder Flexion/Extension Angle vs. Time
subplot(4, 1, 1);
plot(Time, ShoulderFlexExtAng);
title('Shoulder Flex/Ext Angular Position');
ylabel('\theta');

% Set y-axis limits with a buffer
ylim([min(ShoulderFlexExtAng) - range(ShoulderFlexExtAng)*0.1, max(ShoulderFlexExtAng) + range(ShoulderFlexExtAng)*0.1]);

% Set y-axis ticks
yticks([-60, 0, 40]);

% Plot Shoulder Flexion/Extension Angular Velocity vs. Time
subplot(4, 1, 2);
plot(Time, ShoulderFlexExtAngVel);
title('Shoulder Flex/Ext Angular Velocity');
ylabel('\omega');

% Set y-axis limits with a buffer
ylim([min(ShoulderFlexExtAngVel) - range(ShoulderFlexExtAngVel)*0.1, max(ShoulderFlexExtAngVel) + range(ShoulderFlexExtAngVel)*0.1]);

% Plot Hip Flexion/Extension Angle vs. Time
subplot(4, 1, 3);
plot(Time, HipFlexExtAng);
title('Hip Flex/Ext Angular Positiony');
ylabel('\theta');

% Set y-axis limits with a buffer
ylim([min(HipFlexExtAng) - range(HipFlexExtAng)*0.1, max(HipFlexExtAng) + range(HipFlexExtAng)*0.1]);

% Plot Hip Flexion/Extension Angular Velocity vs. Time
subplot(4, 1, 4);
plot(Time, HipFlexExtAngVel);
title('Hip Flex/Ext Angular Velocity');
xlabel('Time');
ylabel('\omega');

% Set y-axis limits with a buffer
ylim([min(HipFlexExtAngVel) - range(HipFlexExtAngVel)*0.1, max(HipFlexExtAngVel) + range(HipFlexExtAngVel)*0.1]);

% Save the entire figure
saveas(gcf, 'joint_kinematics_plots_AngPos_AngVel.png'); 

% Display a message prompting for a mouse click
disp('Please click with the mouse to continue...');

% Wait for mouse click
ginput(1);

% Once clicked, continue with the rest of the code
disp('Mouse clicked! Continuing with the code...');



%% Normalize Joint Kinematics
% Upper Extremity
% Wrist Flexion/Extension
NormWristFlexExtAng = (WristFlexExtAng-min(WristFlexExtAng))/range(WristFlexExtAng);
NormWristFlexExtAngVel = (WristFlexExtAngVel-min(WristFlexExtAngVel))/range(WristFlexExtAngVel);
% Wrist Radial/Ulnar Deviation
NormWristRadUlnDevAng = (WristRadUlnDevAng-min(WristRadUlnDevAng))/range(WristRadUlnDevAng);
NormWristRadUlnDevAngVel = (WristRadUlnDevAngVel-min(WristRadUlnDevAngVel))/range(WristRadUlnDevAngVel);
% Elbow Flexion/Extension
NormElbowFlexExtAng = (ElbowFlexExtAng-min(ElbowFlexExtAng))/range(ElbowFlexExtAng);
NormElbowFlexExtAngVel = (ElbowFlexExtAngVel-min(ElbowFlexExtAngVel))/range(ElbowFlexExtAngVel);
% Shoulder Flexion/Extension
NormShoulderFlexExtAng = (ShoulderFlexExtAng-min(ShoulderFlexExtAng))/range(ShoulderFlexExtAng);
NormShoulderFlexExtAngVel = (ShoulderFlexExtAngVel-min(ShoulderFlexExtAngVel))/range(ShoulderFlexExtAngVel);
% Shoulder Abduction/Adduction
NormShoulderAbAddAng = (ShoulderAbAddAng-min(ShoulderAbAddAng))/range(ShoulderAbAddAng);
NormShoulderAbAddAngVel = (ShoulderAbAddAngVel-min(ShoulderAbAddAngVel))/range(ShoulderAbAddAngVel);
% Trunk Flexion/Extension
NormTrunkFlexExtAng = (TrunkFlexExtAng-min(TrunkFlexExtAng))/range(TrunkFlexExtAng);
NormTrunkFlexExtAngVel = (TrunkFlexExtAngVel-min(TrunkFlexExtAngVel))/range(TrunkFlexExtAngVel);
% Trunk Lateral Flexion
NormTrunkLatFlexAng = (TrunkLatFlexAng-min(TrunkLatFlexAng))/range(TrunkLatFlexAng);
NormTrunkLatFlexAngVel = (TrunkLatFlexAngVel-min(TrunkLatFlexAngVel))/range(TrunkLatFlexAngVel);  
%Lower Extremity
% Hip Flexion/Extension
NormHipFlexExtAng = (HipFlexExtAng-min(HipFlexExtAng))/range(HipFlexExtAng);
NormHipFlexExtAngVel = (HipFlexExtAngVel-min(HipFlexExtAngVel))/range(HipFlexExtAngVel);
% Hip Abduction/Adduction
NormHipAbAddAng = (HipAbAddAng-min(HipAbAddAng))/range(HipAbAddAng);
NormHipAbAddAngVel = (HipAbAddAngVel-min(HipAbAddAngVel))/range(HipAbAddAngVel);
% Knee Flexion/Extension
NormKneeFlexExtAng = (KneeFlexExtAng-min(KneeFlexExtAng))/range(KneeFlexExtAng);
NormKneeFlexExtAngVel = (KneeFlexExtAngVel-min(KneeFlexExtAngVel))/range(KneeFlexExtAngVel);
% Knee Internal/External Rotation
NormKneeIntExtRotAng = (KneeIntExtRotAng-min(KneeIntExtRotAng))/range(KneeIntExtRotAng);
NormKneeIntExtRotAngVel = (KneeIntExtRotAngVel-min(KneeIntExtRotAngVel))/range(KneeIntExtRotAngVel);
% Ankle Plantarflexion/Dorsiflexion
NormAnklePlantDorsAng = (AnklePlantDorsAng-min(AnklePlantDorsAng))/range(AnklePlantDorsAng);
NormAnklePlantDorsAngVel = (AnklePlantDorsAngVel-min(AnklePlantDorsAngVel))/range(AnklePlantDorsAngVel);
% Ankle Inversion/Eversion
NormAnkleInvEvAng = (AnkleInvEvAng-min(AnkleInvEvAng))/range(AnkleInvEvAngVel);
NormAnkleInvEvAngVel = (AnkleInvEvAngVel-min(AnkleInvEvAngVel))/range(AnkleInvEvAngVel);

%% Plot Normalized Shoulder Flexion/Extension Angular Velocity vs. Angular Position
subplot(2, 1, 1);
plot(NormShoulderFlexExtAng, NormShoulderFlexExtAngVel);
title('Normalized Shoulder Angular Velocity vs Angular Position');
xlabel('\theta');
ylabel('\omega');

% Set x-axis and y-axis limits with a buffer
xlim([min(NormShoulderFlexExtAng) - range(NormShoulderFlexExtAng)*0.1, max(NormShoulderFlexExtAng) + range(NormShoulderFlexExtAng)*0.1]);
ylim([min(NormShoulderFlexExtAngVel) - range(NormShoulderFlexExtAngVel)*0.1, max(NormShoulderFlexExtAngVel) + range(NormShoulderFlexExtAngVel)*0.1]);

% Plot Normalized Hip Flexion/Extension Angular Velocity vs. Angular Position
subplot(2, 1, 2);
plot(NormHipFlexExtAng, NormHipFlexExtAngVel);
title('Normalized Hip Angular Velocity vs Angular Position');
xlabel('\theta');
ylabel('\omega');

% Set x-axis and y-axis limits with a buffer
xlim([min(NormHipFlexExtAng) - range(NormHipFlexExtAng)*0.1, max(NormHipFlexExtAng) + range(NormHipFlexExtAng)*0.1]);
ylim([min(NormHipFlexExtAngVel) - range(NormHipFlexExtAngVel)*0.1, max(NormHipFlexExtAngVel) + range(NormHipFlexExtAngVel)*0.1]);

% Display a message prompting for a mouse click
disp('Please click with the mouse to continue...');

% Wait for mouse click
ginput(1);

% Once clicked, continue with the rest of the code
disp('Mouse clicked! Continuing with the code...');

% Save the entire figure
saveas(gcf, 'normalized_joint_kinematics_plots.png'); % or choose your preferred file format and name


%% Calculate and Normalize Joint Phase Angles
% Upper Extremity
% Wrist Flexion/Extension
WristFlexExtPhAng = atand(NormWristFlexExtAngVel./NormWristFlexExtAng);
    NormWristFlexExtPhAng = resample(WristFlexExtPhAng,length(Percentage),length(WristFlexExtPhAng));
% Wrist Radial/Ulnar Deviation
WristRadUlnDevPhAng = atand(NormWristRadUlnDevAngVel./NormWristRadUlnDevAng);
    NormWristRadUlnPhAng = resample(WristRadUlnDevPhAng,length(Percentage),length(WristRadUlnDevPhAng));
% Elbow Flexion/Extension
ElbowFlexExtPhAng = atand(NormElbowFlexExtAngVel./NormElbowFlexExtAng);
    NormElbowFlexExtPhAng = resample(ElbowFlexExtPhAng,length(Percentage),length(ElbowFlexExtPhAng));
% Shoulder Flexion/Extension
ShoulderFlexExtPhAng = atand(NormShoulderFlexExtAngVel./NormShoulderFlexExtAng);
    NormShoulderFlexExtPhAng = resample(ShoulderFlexExtPhAng,length(Percentage),length(ShoulderFlexExtPhAng));
% Shoulder Abduction/Adduction
ShoulderAbAddPhAng = atand(NormShoulderAbAddAngVel./NormShoulderAbAddAng);
    NormShoulderAbAddPhAng = resample(ShoulderAbAddPhAng,length(Percentage),length(ShoulderAbAddPhAng));
% Trunk Flexion/Extension
TrunkFlexExtPhAng = atand(NormTrunkFlexExtAngVel./NormTrunkFlexExtAng);
    NormTrunkFlexExtPhAng = resample(TrunkFlexExtPhAng,length(Percentage),length(TrunkFlexExtPhAng));
% Trunk Lateral Flexion
TrunkLatFlexPhAng = atand(NormTrunkLatFlexAngVel./NormTrunkLatFlexAng);
    NormTrunkLatFlexPhAng = resample(TrunkLatFlexPhAng,length(Percentage),length(TrunkLatFlexPhAng));
%Lower Extremity
% Hip Flexion/Extension
HipFlexExtPhAng = atand(NormHipFlexExtAngVel./NormHipFlexExtAng);
    NormHipFlexExtPhAng = resample(HipFlexExtPhAng,length(Percentage),length(HipFlexExtPhAng));
% Hip Abduction/Adduction
HipAbAddPhAng = atand(NormHipAbAddAngVel./NormHipAbAddAng);
    NormHipAbAddPhAng = resample(HipAbAddPhAng,length(Percentage),length(HipAbAddPhAng));
% Knee Flexion/Extension
KneeFlexExtPhAng = atand(NormKneeFlexExtAngVel./NormKneeFlexExtAng);
    NormKneeFlexExtPhAng = resample(KneeFlexExtPhAng,length(Percentage),length(KneeFlexExtPhAng));
% Knee Internal/External Rotation
KneeIntExtRotPhAng = atand(NormKneeIntExtRotAngVel./NormKneeIntExtRotAng);
    NormKneeIntExtRotPhAng = resample(KneeIntExtRotPhAng,length(Percentage),length(KneeIntExtRotPhAng));
% Ankle Plantarflexion/Dorsiflexion
AnklePlantDorsPhAng = atand(NormAnklePlantDorsAngVel./NormAnklePlantDorsAng);
    NormAnklePlantDorsPhAng = resample(AnklePlantDorsPhAng,length(Percentage),length(AnklePlantDorsPhAng));
% Ankle Inversion/Eversion
AnkleInvEvPhAng = atand(NormAnkleInvEvAngVel./NormAnkleInvEvAng);
    NormAnkleInvEvPhAng = resample(AnkleInvEvPhAng,length(Percentage),length(AnkleInvEvPhAng));
%% Plot Normalized Shoulder Flexion/Extension Phase Angle vs Percentage
subplot(2, 1, 1);
plot(Percentage, NormShoulderFlexExtPhAng);
title('Normalized Shoulder Phase Angle vs Percentage');
xlabel('%');
ylabel('\phi_{\theta}');

% Display a message prompting for a mouse click
disp('Please click with the mouse to continue...');

% Wait for mouse click
ginput(1);

% Once clicked, continue with the rest of the code
disp('Mouse clicked! Continuing with the code...');

% Plot Normalized Hip Flexion/Extension Phase Angle vs Percentage
subplot(2, 1, 2);
plot(Percentage, NormHipFlexExtPhAng);
title('Normalized Hip Phase Angle vs Percentage');
xlabel('%');
ylabel('\phi_{\theta}');

% Display a message prompting for a mouse click
disp('Please click with the mouse to continue...');

% Wait for mouse click
ginput(1);

% Once clicked, continue with the rest of the code
disp('Mouse clicked! Continuing with the code...');

% Save the entire figure
saveas(gcf, 'normalized_joint_phase_angles_vs_percentage_plots.png'); % or choose your preferred file format and name

%% Calculate Relative Phase Angles
%Intralimb Coordination
% Wrist Ulnar/Radial Deviation to Elbow Flexion/Extension
RelPhAngWristToElbow = (NormWristRadUlnPhAng-NormElbowFlexExtPhAng);
% Elbow Flexion/Extension to Shoulder Abduction/Adduction
RelPhAngElbowToShoulder = (NormElbowFlexExtPhAng-NormShoulderAbAddPhAng);
% Shoulder Abduction/Adduction to Trunk Lateral Flexion
RelPhAngShoulderToTrunk = (NormShoulderAbAddPhAng-NormTrunkLatFlexPhAng);
% Hip Flexion/Extension to Trunk Flexion/Extension
RelPhAngHipToTrunk = (NormHipFlexExtPhAng-NormTrunkFlexExtPhAng);
% Hip Abduction/Adduction to Trunk Lateral Flexion
RelPhAngHipToTrunkFront = (NormHipAbAddPhAng-NormTrunkLatFlexPhAng);
% Knee Flexion/Extension to Hip Flexion/Extension
RelPhAngKneeToHip = (NormKneeFlexExtPhAng-NormHipFlexExtPhAng);
% Knee Flexion/Extension to Hip Abduction/Adduction
RelPhAngKneeToHipFront = (NormKneeFlexExtPhAng-NormHipAbAddPhAng);
% Ankle Plantarflexion/Dorsiflexion to Knee Flexion/Extension
RelPhAngAnkleToKnee = (NormAnklePlantDorsPhAng-NormKneeFlexExtPhAng);
%Interlimb Coordination
% Wrist Flexion/Extension to Ankle Plantarflexion/Dorsiflexion
RelPhAngWristToAnkle = (NormWristFlexExtPhAng-NormAnklePlantDorsPhAng);
% Elbow Flexion/Extension to Knee Flexion/Extension
RelPhAngElbowToKnee = (NormElbowFlexExtPhAng-NormKneeFlexExtPhAng);
% Shoulder Abduction/Adduction to Hip Flexion/Extension
RelPhAngShoulderToHip = (NormShoulderAbAddPhAng-NormHipAbAddPhAng);
%% Plot Relative Phase Angle between Shoulder and Hip
figure;
plot(Percentage, RelPhAngShoulderToHip);
title('Relative Phase Angle between Shoulder and Hip');
xlabel('%');
ylabel('CRP (\circ)');

% Display a message prompting for a mouse click
disp('Please click with the mouse to continue...');

% Wait for mouse click
ginput(1);

% Once clicked, continue with the rest of the code
disp('Mouse clicked! Continuing with the code...');

% Save the entire figure
saveas(gcf, 'CRP_shoulder_hip.png'); % or choose your preferred file format and name

% Inform the user that the figure has been saved
disp('Figure saved successfully!');


% % %% Calculate Discrete Variables for Analyses
% % % Intralimb Coordination
% % % Wrist to Elbow
% % RMSWristToElbow = rms(RelPhAngWristToElbow);
% % MinWristToElbow = min(RelPhAngWristToElbow);
% % MaxWristToElbow = max(RelPhAngWristToElbow);
% % % Elbow to Shoulder
% % RMSElbowToShoulder = rms(RelPhAngElbowToShoulder);
% % MinElbowToShoulder = min(RelPhAngElbowToShoulder);
% % MaxElbowToShoulder = max(RelPhAngElbowToShoulder);
% % % Shoulder to Trunk
% % RMSShoulderToTrunk = rms(RelPhAngShoulderToTrunk);
% % MinShoulderToTrunk = min(RelPhAngShoulderToTrunk);
% % MaxShoulderToTrunk = max(RelPhAngShoulderToTrunk);
% % % Hip to Trunk
% % RMSHipToTrunk = rms(RelPhAngHipToTrunk);
% % MinHipToTrunk = min(RelPhAngHipToTrunk);
% % MaxHipToTrunk = max(RelPhAngHipToTrunk);
% % % Hip to Trunk Frontal Plane
% % RMSHipToTrunkFront = rms(RelPhAngHipToTrunkFront);
% % MinHipToTrunkFront = min(RelPhAngHipToTrunkFront);
% % MaxHipToTrunkFront = max(RelPhAngHipToTrunkFront);
% % % Knee to Hip
% % RMSKneeToHip = rms(RelPhAngKneeToHip);
% % MinKneeToHip = min(RelPhAngKneeToHip);
% % MaxKneeToHip = max(RelPhAngKneeToHip);
% % % Knee to Hip Frontal Plane
% % RMSKneeToHipFront = rms(RelPhAngKneeToHipFront);
% % MinKneeToHipFront = min(RelPhAngKneeToHipFront);
% % MaxKneeToHipFront = max(RelPhAngKneeToHipFront);
% % % Ankle to Knee
% % RMSAnkleToKnee = rms(RelPhAngAnkleToKnee);
% % MinAnkleToKnee = min(RelPhAngAnkleToKnee);
% % MaxAnkleToKnee = max(RelPhAngAnkleToKnee);
% % %Interlimb Coordination
% % % Wrist to Ankle
% % RMSWristToAnkle = rms(RelPhAngWristToAnkle);
% % MinWristToAnkle = min(RelPhAngWristToAnkle);
% % MaxWristToAnkle = max(RelPhAngWristToAnkle);
% % % Elbow to Knee
% % RMSElbowToKnee = rms(RelPhAngElbowToKnee);
% % MinElbowToKnee = min(RelPhAngElbowToKnee);
% % MaxElbowToKnee = max(RelPhAngElbowToKnee);
% % % Shoulder to Hip
% % RMSShoulderToHip = rms(RelPhAngShoulderToHip);
% % MinShoulderToHip = min(RelPhAngShoulderToHip);
% % MaxShoulderToHip = max(RelPhAngShoulderToHip);
% % %% Output Table and File
% % OutputTable = table(RMSWristToElbow,MinWristToElbow,MaxWristToElbow,RMSElbowToShoulder,MinElbowToShoulder,MaxElbowToShoulder,RMSShoulderToTrunk,MinShoulderToTrunk,MaxShoulderToTrunk,RMSHipToTrunk,MinHipToTrunk,MaxHipToTrunk,...
% %     RMSHipToTrunkFront,MinHipToTrunkFront,MaxHipToTrunkFront,RMSKneeToHip,MinKneeToHip,MaxKneeToHip,RMSKneeToHipFront,MinKneeToHipFront,MaxKneeToHipFront,RMSAnkleToKnee,MinAnkleToKnee,MaxAnkleToKnee,...
% %     RMSWristToAnkle,MinWristToAnkle,MaxWristToAnkle,RMSElbowToKnee,MinElbowToKnee,MaxElbowToKnee,RMSShoulderToHip,MinShoulderToHip,MaxShoulderToHip);
% %         OutputTable.Properties.VariableNames(1:33) = {'RMSWristToElbow','MinWristToElbow','MaxWristToElbow','RMSElbowToShoulder','MinElbowToShoulder','MaxElbowToShoulder','RMSShoulderToTrunk','MinShoulderToTrunk','MaxShoulderToTrunk'...
% %             'RMSHipToTrunk','MinHipToTrunk','MaxHipToTrunk','RMSHipToTrunkFront','MinHipToTrunkFront','MaxHipToTrunkFront','RMSKneeToHip','MinKneeToHip','MaxKneeToHip','RMSKneeToHipFront','MinKneeToHipFront','MaxKneeToHipFront'...
% %             'RMSAnkleToKnee','MinAnkleToKnee','MaxAnkleToKnee','RMSWristToAnkle','MinWristToAnkle','MaxWristToAnkle','RMSElbowToKnee','MinElbowToKnee','MaxElbowToKnee','RMSShoulderToHip','MinShoulderToHip','MaxShoulderToHip'};
% %             TableName = strcat(TrialName,OutputExt);
% %                 writetable(OutputTable,TableName)
% % close all;

