clc; clear; close all;
warning off;
rng default;


% load experiment settings
fid = fopen('exp_settings.txt');
exp_folder_read = [];
tline = fgetl(fid);
while ischar(tline)
    exp_folder_read = [exp_folder_read; tline];
    tline = fgetl(fid);
end
fclose(fid);

exp_folder = strrep(exp_folder_read,'\','/');

exp_folder = strcat('.',exp_folder);
data_filename = strcat(exp_folder,'/MATLAB_RT.mat');
csv_filename = strcat(exp_folder,'/gravity_log_file_2024_2_6_21429.csv.csv');

load(data_filename);
%% 

T = readtable(csv_filename);
%%
A = table2array(T);

%% 
matlab_unix_epoch = [];
matlab_q = [];
for i = 1 : length(MATLAB_RT)
    matlab_unix_epoch = [matlab_unix_epoch, MATLAB_RT(i).curr_unix_epoch];
    matlab_q = [matlab_q, MATLAB_RT(i).des_q];
end


%% 

figure, plot(A(:,3)/10^6,A(:,120:126)), hold on,
plot(matlab_unix_epoch,matlab_q)