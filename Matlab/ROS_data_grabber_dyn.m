rosshutdown
rosinit('192.168.1.2')


global PID_data
global PID_indx
global temp_index
PID_indx = PID_indx + 1;
temp_index = 1;
sub = rossubscriber('/auto/gate_detection_result_dynamic','std_msgs/Float64MultiArray',@nav_callback);

fprintf('connected, recording on index %d\n',PID_indx)



%%
clear sub

%%
save('PID.mat','PID_data','PID_indx')
fprintf('Saved the data')

%%
load('PID.mat','PID_data','PID_indx')
fprintf('Data loaded')

%%
PID_indx = PID_indx - 1

%%
PID_indx = PID_indx + 1

%%

PID_indx = 0;
PID_data = cell(1000,10000);



%%

function nav_callback(src, msg)
    global PID_data
    global PID_indx
    global temp_index
    PID_data{PID_indx,temp_index} = msg.Data;
    temp_index = temp_index+1;
end