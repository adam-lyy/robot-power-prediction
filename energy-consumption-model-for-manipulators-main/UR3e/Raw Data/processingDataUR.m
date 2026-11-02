% author: Juan Heredia, Christian Schlette, Mikkel Baun Kjærgaard
% date: September, 2022
% 
% -------------------------------------------------
% Training Dynamic Model v. 1.0
% -------------------------------------------------
% J. Heredia, C. Schlette and M. B. Kjærgaard, "Data-Driven Energy Estimation 
% of Individual Instructions in User-Defined Robot Programs for Collaborative 
% Robots," in IEEE Robotics and Automation Letters, vol. 6, no. 4, pp. 6836-6843,
% Oct. 2021, doi: 10.1109/LRA.2021.3094781.
%
% the following code has been tested on Matlab 2020a

% Prepocessing Data for UR3e

% Read data for testing
URData =[csvread("UR_DataDynamics_record7.csv",2,0);
        csvread("UR_DataDynamics_record8.csv",2,0);
        csvread("UR_DataDynamics_record9.csv",2,0);
        csvread("UR_DataDynamics_record11.csv",2,0);
        csvread("UR_DataDynamics_record12.csv",2,0);
        csvread("UR_DataDynamics_record13.csv",2,0);
        csvread("UR3record14.csv",2,0);
        csvread("UR3record15.csv",2,0);
        csvread("UR3record16.csv",2,0);
        csvread("UR3record17.csv",2,0);
        csvread("UR3record18.csv",2,0);
        csvread("UR3record19.csv",2,0);
        csvread("UR3record20.csv",2,0);
        csvread("UR3record21.csv",2,0)];
     
T = URData(:,:);
t = T(:,1);%read time
dt = diff(t);
k_1 = find(abs(diff(t))>0.3)+1;
k_1 = [1;k_1;size(t,1)];
t_aux = 0;
for i = 1:1: length(k_1)-1
    t_aux = [t_aux; t(k_1(i):1:k_1(i+1)-1,:)- t(k_1(i),1) + t_aux(end,1)+0.01];
end
t = t_aux;
%read the joint positions
expq1 = T(:,2); 
expq2 = T(:,3);
expq3 = T(:,4);
expq4 = T(:,5);
expq5 = T(:,6);
expq6 = T(:,7);
%read the joint velocities
expdq1 = T(:,8);
expdq2 = T(:,9);
expdq3 = T(:,10);
expdq4 = T(:,11);
expdq5 = T(:,12);
expdq6 = T(:,13);
%read the motor currents
expI1 = T(1:1:end,14);
expI2 = T(1:1:end,15);
expI3 = T(1:1:end,16);
expI4 = T(1:1:end,17);
expI5 = T(1:1:end,18);
expI6 = T(1:1:end,19);
expIT = T(1:1:end,34);
expVT = T(1:1:end,33);
Y = expIT.*expVT;
PC = Y(1:1:end,:);
t = t(1:1:end,:);

d = designfilt('lowpassiir','FilterOrder',12, ...
    'HalfPowerFrequency',0.15,'DesignMethod', 'butter');

expddq1 = diff(expdq1)./dt;
expddq1 = filtfilt(d,expddq1);
expddq2 = diff(expdq2)./dt;
expddq2 = filtfilt(d,expddq2);
expddq3 = diff(expdq3)./dt;
expddq3 = filtfilt(d,expddq3);
expddq4 = diff(expdq4)./dt;
expddq4 = filtfilt(d,expddq4);
expddq5 = diff(expdq5)./dt;
expddq5 = filtfilt(d,expddq5);
expddq6 = diff(expdq6)./dt;
expddq6 = filtfilt(d,expddq6);

expIT = T(1:1:end,34);
expVT = T(1:1:end,33);
Y = expIT.*expVT;

expq = [expq1 expq2 expq3 expq4 expq5 expq6];
expq = expq(2:1:end,:);
expdq = [expdq1 expdq2 expdq3 expdq4 expdq5 expdq6];
expdq = expdq(2:1:end,:);
expddq = [expddq1 expddq2 expddq3 expddq4 expddq5 expddq6];
expI = [expI1 expI2 expI3 expI4 expI5 expI6];
expI = expI(2:1:end,:);
PC = Y(2:1:end,:);
PC = filtfilt(d,PC);
t = t(2:1:end,:);

Data2 = [t expq expdq expddq expI PC];
RowName = {'t','q1','q2','q3','q4','q5','q6',...
    'dq1','dq2','dq3','dq4','dq5','dq6',...
    'ddq1','ddq2','ddq3','ddq4','ddq5','ddq6',...
    'I1','I2','I3','I4','I5','I6', 'PC' };
T = array2table(Data2);
T.Properties.VariableNames = RowName;
writetable(T,'URRobotData.csv','Delimiter',',','QuoteStrings',true)


%% Testing Data UR

% Read data for testing
URData = [ csvread("UR3recordTesting2.csv",2,0);
         csvread("UR3recordTesting4.csv",2,0)];
     
     
T = URData(:,:);
t = T(:,1);%read time
dt = diff(t);
k_1 = find(abs(diff(t))>0.3)+1;
k_1 = [1;k_1;size(t,1)];
t_aux = 0;
for i = 1:1: length(k_1)-1
    t_aux = [t_aux; t(k_1(i):1:k_1(i+1)-1,:)- t(k_1(i),1) + t_aux(end,1)+0.01];
end
t = t_aux;

%read the joint positions
expq1 = T(:,2); 
expq2 = T(:,3);
expq3 = T(:,4);
expq4 = T(:,5);
expq5 = T(:,6);
expq6 = T(:,7);
%read the joint velocities
expdq1 = T(:,8);
expdq2 = T(:,9);
expdq3 = T(:,10);
expdq4 = T(:,11);
expdq5 = T(:,12);
expdq6 = T(:,13);
%read the motor currents
expI1 = T(1:1:end,14);
expI2 = T(1:1:end,15);
expI3 = T(1:1:end,16);
expI4 = T(1:1:end,17);
expI5 = T(1:1:end,18);
expI6 = T(1:1:end,19);
expIT = T(1:1:end,34);
expVT = T(1:1:end,33);
Y = expIT.*expVT;
PC = Y(1:1:end,:);
t = t(1:1:end,:);

d = designfilt('lowpassiir','FilterOrder',12, ...
    'HalfPowerFrequency',0.15,'DesignMethod', 'butter');

expddq1 = diff(expdq1)./dt;
expddq1 = filtfilt(d,expddq1);
expddq2 = diff(expdq2)./dt;
expddq2 = filtfilt(d,expddq2);
expddq3 = diff(expdq3)./dt;
expddq3 = filtfilt(d,expddq3);
expddq4 = diff(expdq4)./dt;
expddq4 = filtfilt(d,expddq4);
expddq5 = diff(expdq5)./dt;
expddq5 = filtfilt(d,expddq5);
expddq6 = diff(expdq6)./dt;
expddq6 = filtfilt(d,expddq6);

expIT = T(1:1:end,34);
expVT = T(1:1:end,33);
Y = expIT.*expVT;

expq = [expq1 expq2 expq3 expq4 expq5 expq6];
expq = expq(2:1:end,:);
expdq = [expdq1 expdq2 expdq3 expdq4 expdq5 expdq6];
expdq = expdq(2:1:end,:);
expddq = [expddq1 expddq2 expddq3 expddq4 expddq5 expddq6];
expI = [expI1 expI2 expI3 expI4 expI5 expI6];
expI = expI(2:1:end,:);
PC = Y(2:1:end,:);
PC = filtfilt(d,PC);
t = t(2:1:end,:);

Data2 = [t expq expdq expddq expI PC];
RowName = {'t','q1','q2','q3','q4','q5','q6',...
    'dq1','dq2','dq3','dq4','dq5','dq6',...
    'ddq1','ddq2','ddq3','ddq4','ddq5','ddq6',...
    'I1','I2','I3','I4','I5','I6', 'PC' };
T = array2table(Data2);
T.Properties.VariableNames = RowName;

writetable(T,'URRobotData_Test.csv','Delimiter',',','QuoteStrings',true)