rosshutdown
rosinit('192.168.1.2')

clear matlab_pose_tracker
clear sub

matlab_pose_tracker = robotics.ros.Node('/matlab_tracker');
sub_nav = robotics.ros.Subscriber(matlab_pose_tracker,'/auto/navigation_logger','std_msgs/String');
sub_gate = robotics.ros.Subscriber(matlab_pose_tracker,'/auto/visual_logger','std_msgs/String');


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





fprintf('connected, recording on index %d and %d \n',temp_index_Nav, temp_index_Gate)

temp_index_Nav = 1;
temp_index_Gate = 1;
while(1)
    try
        msg1 = receive(sub_nav,.001);
        NavPID_data{NavPID_indx,temp_index_Nav} = cellstr(msg1.Data);
        temp_index_Nav = temp_index_Nav+1;
    end
    try
        msg2 = receive(sub_gate,.001);
        Gate_data{Gate_indx,temp_index_Gate} = cellstr(msg2.Data);
        temp_index_Gate = temp_index_Gate+1;
    end
end

%%
save('GateData.mat','Gate_data','Gate_indx')
save('NavPID.mat','NavPID_data','NavPID_indx')

%%
NavPID_indx = NavPID_indx - 1
Gate_indx = Gate_indx - 1