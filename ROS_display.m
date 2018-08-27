%%
% 1 wp_current.pos[0]
% 2 wp_current.pos[1]
% 3 wp_current.pos[2]
% 4 wp_current.hdg
% 5 wp_average.pos[0]
% 6 wp_average.pos[1]
% 7 wp_average.pos[2]
% 8 wp_average.hdg
% 9 bebop_p[0][0]
% 10 bebop_p[1][0]
% 11 bebop_p[2][0]
% 12 bebop_q[3]
% 13 bebop_q[0]
% 14 bebop_q[1]
% 15 bebop_q[2]
% 16 heading_to_gate
                     
%% X Total
dataTemp2 = dataVar(indx,:);
dataTemp = [];
for k = 1:1000
    dataTemp = [dataTemp;cellfun(@str2num,strsplit(dataTemp2{k}{1},', '))];
    
    if ~iscell(dataVar{indx,k+1})
       break 
    end
end

figure
hold on
grid on
plot(dataTemp(:,1))
plot(dataTemp(:,2))
plot(dataTemp(:,3))
plot(dataTemp(:,4))
plot(dataTemp(:,5))
plot(dataTemp(:,6))
plot(dataTemp(:,7))
plot(dataTemp(:,8))
plot(dataTemp(:,9))
plot(dataTemp(:,10))
plot(dataTemp(:,11))
plot(dataTemp(:,12))
legend('x pos error','X Pos P','X Pos I','X Pos D','X Vel Des','Vel x','X vel error','X Vel P','X Vel I','X Vel D','X Vel Out','msg.x')

%% X Pos/Vel
dataTemp2 = dataVar(indx,:);
dataTemp = [];
for k = 1:1000
    dataTemp = [dataTemp;cellfun(@str2num,strsplit(dataTemp2{k}{1},', '))];
    
    if ~iscell(dataVar{indx,k+1})
       break 
    end
end

figure
hold on
grid on
plot(dataTemp(:,1))
plot(dataTemp(:,2))
plot(dataTemp(:,3))
plot(dataTemp(:,4))
plot(dataTemp(:,5))
plot(dataTemp(:,6))
legend('X pos error','X Pos P','X Pos I','X Pos D','X Vel Des','Vel x')
title('Position')


figure
hold on
grid on
plot(dataTemp(:,5))
plot(dataTemp(:,6))
plot(dataTemp(:,7))
plot(dataTemp(:,8))
plot(dataTemp(:,9))
plot(dataTemp(:,10))
plot(dataTemp(:,11))
plot(dataTemp(:,12))
legend('X Vel Des','Vel x','X vel error','X Vel P','X Vel I','X Vel D','X Vel Out','msg.x')
title('Velocity')

%% Y Total
dataTemp2 = dataVar(indx,:);
dataTemp = [];
for k = 1:1000
    dataTemp = [dataTemp;cellfun(@str2num,strsplit(dataTemp2{k}{1},', '))];
    
    if ~iscell(dataVar{indx,k+1})
       break 
    end
end

figure
hold on
grid on
plot(dataTemp(:,13))
plot(dataTemp(:,14))
plot(dataTemp(:,15))
plot(dataTemp(:,16))
plot(dataTemp(:,17))
plot(dataTemp(:,18))
plot(dataTemp(:,19))
plot(dataTemp(:,20))
plot(dataTemp(:,21))
plot(dataTemp(:,22))
plot(dataTemp(:,23))
plot(dataTemp(:,24))
legend('Y Pos Error','Y Pos P','Y Pos I','Y Pos D','Y Vel Des','Vel Y','Y Vel Error','Y Vel P','Y Vel I','Y Vel D','Y Vel Out','msg.y')




%% Y Pos/Vel
dataTemp2 = dataVar(indx,:);
dataTemp = [];
for k = 1:1000
    dataTemp = [dataTemp;cellfun(@str2num,strsplit(dataTemp2{k}{1},', '))];
    
    if ~iscell(dataVar{indx,k+1})
       break 
    end
end

figure
hold on
grid on
plot(dataTemp(:,13))
plot(dataTemp(:,14))
plot(dataTemp(:,15))
plot(dataTemp(:,16))
plot(dataTemp(:,17))
plot(dataTemp(:,18))
legend('Y Pos Error','Y Pos P','Y Pos I','Y Pos D','Y Vel Des','Vel Y')
title('Position')

figure
hold on
grid on
plot(dataTemp(:,18))
plot(dataTemp(:,19))
plot(dataTemp(:,20))
plot(dataTemp(:,21))
plot(dataTemp(:,22))
plot(dataTemp(:,23))
plot(dataTemp(:,24))
legend('Vel Y','Y Vel Error','Y Vel P','Y Vel I','Y Vel D','Y Vel Out','msg.y')
title('Velocity')


%% Z
dataTemp2 = dataVar(indx,:);
dataTemp = [];
for k = 1:1000
    dataTemp = [dataTemp;cellfun(@str2num,strsplit(dataTemp2{k}{1},', '))];
    
    if ~iscell(dataVar{indx,k+1})
       break 
    end
end

figure
hold on
grid on
plot(dataTemp(:,26))
plot(dataTemp(:,27))
plot(dataTemp(:,28))
plot(dataTemp(:,29))
plot(dataTemp(:,30))
plot(dataTemp(:,31))
legend('Z Error','Z Pos P','Z Pos I','D term','Z Vel Out','msg.z')

%% R
dataTemp2 = dataVar(indx,:);
dataTemp = [];
for k = 1:1000
    dataTemp = [dataTemp;cellfun(@str2num,strsplit(dataTemp2{k}{1},', '))];
    
    if ~iscell(dataVar{indx,k+1})
       break 
    end
end

figure
hold on
grid on
plot(dataTemp(:,32))
plot(dataTemp(:,33))
plot(dataTemp(:,34))
plot(dataTemp(:,35))
plot(dataTemp(:,36))
plot(dataTemp(:,37))
plot(dataTemp(:,38))
plot(dataTemp(:,39))
legend('Angle Des','Vehicle Angle','R error','P term','I term','D term','nav cmd R','msg.r')


%% mics
dataTemp2 = dataVar(indx,:);
dataTemp = [];
for k = 1:1000
    dataTemp = [dataTemp;cellfun(@str2num,strsplit(dataTemp2{k}{1},', '))];
    
    if ~iscell(dataVar{indx,k+1})
       break 
    end
end
figure
hold on
grid on
plot(dataTemp(:,40))
plot(dataTemp(:,41))
plot(dataTemp(:,42))
plot(dataTemp(:,43))
plot(dataTemp(:,44))
legend('1','2','3','4','5')
