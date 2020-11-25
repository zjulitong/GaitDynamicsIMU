function [COMVf,COMAf]=COM_diff(COMP,aux)

% NOS=auxdata.NOS;
% idx_RHS=GaitEvent.idx_RHS;
% frame_all=[idx_RHS(1):auxdata.N_Frame];
frame_count=size(COMP,1);

%Kinematics Filter
mkr_freq=aux.TrajFrameRate;
[fb_mkr, fa_mkr] = butter(2,2*6/mkr_freq);

%%
% COMPf = filtfilt(fb_mkr,fa_mkr,COMP);

COMV = (COMP(3:end,:)-COMP(1:end-2,:))/2./(1/mkr_freq);
COMVf = filtfilt(fb_mkr,fa_mkr,COMV);

COMA = (COMVf(3:end,:)-COMVf(1:end-2,:))/2./(1/mkr_freq);
COMAf = filtfilt(fb_mkr,fa_mkr,COMA);

COMVf=interp1((2:frame_count-1).',COMVf,(1:frame_count).','spline');
COMAf=interp1((3:frame_count-2).',COMAf,(1:frame_count).','spline');
