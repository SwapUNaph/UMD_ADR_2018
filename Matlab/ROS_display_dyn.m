close all

dataTemp1 = PID_data(PID_indx,:);
dataTemp = [];
for k = 1:1000
    dataTemp = [dataTemp;dataTemp1{k}'];    
end

dataTemp1 = dataTemp;
dataTemp1(:,2)=unwrap(dataTemp1(:,2));
diff_T=diff(dataTemp1);
p = diff_T(:,2)./diff_T(:,1);
m=mean(p)

dataTemp2 = dataTemp;
k=2;
% while k<=length(dataTemp2)
%     expect = dataTemp2(k-1,2) + (dataTemp2(k,1)-dataTemp2(k-1,1))*m;
%     a_diff = abs(dataTemp2(k,2) - expect);
%    if 30*pi/180 < a_diff && a_diff < 330*pi/180
%        dataTemp2(k,:) = [];
%    else
%        k=k+1;
%    end
% end
%dataTemp2(:,2)=unwrap(dataTemp2(:,2));
diff_T=diff(dataTemp2);
% p = diff_T(:,2)./diff_T(:,1);
% m=mean(p)
% 
% if m>0
%     for k=find(p<-.2*m)
%         p2 = p(k) + 2*pi/diff_T(k,1)
%         if abs(m-p(k)) > abs(m-p2)
%             p(k) = p2;
%         end
%     end
% end


figure
hold on
grid on
plot(dataTemp1(:,1),dataTemp1(:,2),'x-')
plot(dataTemp2(:,1),dataTemp2(:,2),'o-')
plot(dataTemp2(1:(end-1),1),p)

plot(dataTemp(:,1),1:length(dataTemp(:,1)),'x-')


legend('X pos error','X vel des','X vel','Y pos error','Y vel des','Y vel')
title('Error and Des')
