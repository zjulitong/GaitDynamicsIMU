function [COMPf,COMVf,COMAf]=posi_diff(COMP,mkr_freq,filtflag)


frame_count=length(COMP);

[fb_mkr, fa_mkr] = butter(2,2*6/mkr_freq);

%%
% if filtflag==1
%     COMPf = filtfilt(fb_mkr,fa_mkr,COMP);
% else
% end
    COMPf=COMP;


COMV = (COMPf(3:end,:)-COMPf(1:end-2,:))/2./(1/mkr_freq);
if filtflag==1
    COMVf = filtfilt(fb_mkr,fa_mkr,COMV);
else
    COMVf=COMV;
end

COMA = (COMVf(3:end,:)-COMVf(1:end-2,:))/2./(1/mkr_freq);
if filtflag==1
    COMAf = filtfilt(fb_mkr,fa_mkr,COMA);
else
    COMAf=COMA;
end

% COMVf=interp1((2:frame_count-1).',COMVf,(1:frame_count).','spline');
% COMAf=interp1((3:frame_count-2).',COMAf,(1:frame_count).','spline');
COMVf=TLinterp1_nan((2:frame_count-1).',COMVf,(1:frame_count).','spline');
COMAf=TLinterp1_nan((3:frame_count-2).',COMAf,(1:frame_count).','spline');

%%
return;
%%
figure;hold on;
plot(COMP)
plot(COMPf)

figure;hold on;
plot(COMV)
plot(COMVf)










end










