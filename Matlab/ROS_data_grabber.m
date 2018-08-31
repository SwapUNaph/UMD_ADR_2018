% rosshutdown
% rosinit('192.168.1.2')

global NavPID_data
global NavPID_indx
global temp_index_Nav
global Gate_data
global Gate_indx
global temp_index_Gate


if ~exist('NavPID_indx')
    load('NavPID.mat','NavPID_data','NavPID_indx')
    if ~exist('NavPID_indx')
        NavPID_indx = 0;
        NavPID_data = cell(1000,1000);
        display('starting NavPID over')
    end
end

if ~exist('Gate_indx')
    load('GateData.mat','Gate_data','Gate_indx')
    if ~exist('Gate_indx')
        Gate_indx = 0;
        Gate_data = cell(1000,1000);
        display('starting Gate over')
    end
end


NavPID_indx = NavPID_indx + 1;
Gate_indx = Gate_indx+1;

temp_index_Nav = 1;
temp_index_Gate = 1;

sub_nav = rossubscriber('/auto/navigation_logger','std_msgs/String',@nav_callback);
sub_gate = rossubscriber('/auto/visual_logger','std_msgs/String',@gate_callback);
fprintf('connected, recording on index %d and %d \n',NavPID_indx, Gate_indx)




%%
clear sub_nav
clear sub_gate

%%
save('GateData.mat','Gate_data','Gate_indx')
save('NavPID.mat','NavPID_data','NavPID_indx')
fprintf('Saved the data')

%%
load('NavPID.mat','NavPID_data','NavPID_indx')
load('GateData.mat','Gate_data','Gate_indx')

%%
NavPID_indx = NavPID_indx - 1
Gate_indx = Gate_indx - 1

%%
NavPID_indx = NavPID_indx + 1
Gate_indx = Gate_indx + 1

%%

function nav_callback(src, msg)
    global NavPID_data
    global NavPID_indx
    global temp_index_Nav
    NavPID_data{NavPID_indx,temp_index_Nav} = cellstr(msg.Data);
    temp_index_Nav = temp_index_Nav+1;
end
function gate_callback(src, msg)
    global Gate_data
    global Gate_indx
    global temp_index_Gate
    Gate_data{Gate_indx,temp_index_Gate} = cellstr(msg2.Data);
    temp_index_Gate = temp_index_Gate+1;
end