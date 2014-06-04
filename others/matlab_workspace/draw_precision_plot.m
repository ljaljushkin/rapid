arr = csvread('output/precision/precision.txt');

sp = arr( : , 1 );
ms = arr( : , 2 ); 
rsp = arr( : , 3 ); 
fsp = arr( : , 4 );

n = length(sp);
x = (5:1:n+4);

% psp = polyfit(x', sp, 9); 
% pms = polyfit(x', ms, 9);
% prsp = polyfit(x', rsp, 9);
% pfsp = polyfit(x', fsp, 9);

% fsp = polyval(psp, x);
% fms = polyval(pms, x);
% frsp = polyval(prsp, x);
% ffsp = polyval(pfsp, x);

figure
hold on

% plot(x, sp, 'bo-', x, ms,'ro-', x, rsp, 'go-', x, fsp,'yo-')
% plot(x', fsp, 'bo-', x', fms,'ro-', x', frsp, 'go-', x', ffsp,'yo-')
plot(x, smooth(sp,20), 'b.-', x, smooth(ms,20), 'r.-', x, smooth(rsp,20), 'g.-', x, smooth(fsp,20),'y.-')

% grid on 
title('Precision') 
xlabel('frame number ') 
ylabel('total reprojection error') 
legend('SolvePnP only',  'RansacSolvePnP', 'MeanShift', 'finalSolvePnP')
