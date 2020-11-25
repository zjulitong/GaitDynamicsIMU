function [Events,aux]=GaitEvent(aux)

%%
[Devicenum,~,~]=xlsread(aux.DynDevice_FileName);
aux.ForceFrameRate=Devicenum(1,1); 


F3z=Devicenum(5:end,3)/10*400*9.8*2;
F3z=F3z-mean(F3z(1:300));

if mean(Devicenum(5:end,10))<mean(Devicenum(5:end,10+9)) % left on is F1, right is F2
    FLz=-Devicenum(5:end,6);
    FRz=-Devicenum(5:end,6+9);
else
    FRz=-Devicenum(5:end,6);
    FLz=-Devicenum(5:end,6+9);
end

[fb_Fz,fa_Fz]=butter(6,20*2/aux.ForceFrameRate);
F3z_filt=filtfilt(fb_Fz,fa_Fz,F3z);
FRz_filt=filtfilt(fb_Fz,fa_Fz,FRz);
FLz_filt=filtfilt(fb_Fz,fa_Fz,FLz);


thre_AMTI=10;
thre_LCell=10;
for i=2:length(FRz_filt)
    if FRz_filt(i)>=thre_AMTI && (FRz_filt(i-1)<thre_AMTI)
        idx_HS1=i;
    end 
    
    if F3z_filt(i)>=thre_LCell && (F3z_filt(i-1)<thre_LCell)
        idx_HS2=i;
    end 
    if FLz_filt(i)<=thre_AMTI && (FLz_filt(i-1)>thre_AMTI)
        idx_TO1=i;
    end 
end
if aux.plotEvent
    figure;hold on;set(gcf,'defaultLineLineWidth',1.5);
    plot(FLz,'b','LineWidth',0.5);
    plot(FRz,'g','LineWidth',0.5);
    plot(F3z,'c','LineWidth',0.5);
    plot(FLz_filt,'k--');
    plot(FRz_filt,'k--');
    plot(F3z_filt,'k--');
    legend({'F1z','F2z','F3z','F2z-filt','F3z-filt'});
    
    plot(idx_HS1,FRz_filt(idx_HS1),'r*');
    plot(idx_TO1,FLz_filt(idx_TO1),'rd');
    plot(idx_HS2,F3z_filt(idx_HS2),'r+');
end

Events.idx_HS1=idx_HS1;
Events.idx_TO1=idx_TO1;
Events.idx_HS2=idx_HS2;














