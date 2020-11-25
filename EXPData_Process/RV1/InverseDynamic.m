function [JForce2D,JMoment2D,JPower2D]=InverseDynamic(Res,aux)

%% Anthropometry
g      = aux.g;
BM     = aux.BM; %[Kg]
mt     = 0.1000*BM; % thigh
ms     = 0.0465*BM; % shank
mf     = 0.0145*BM; % shank

Len_Thi=(Res.Traj.Len_RThi+Res.Traj.Len_LThi)/2;
Len_Sha=(Res.Traj.Len_RSha+Res.Traj.Len_LSha)/2;
Len_Foo=(Res.Traj.Len_RFoo+Res.Traj.Len_LFoo)/2;

It     = mt*(0.323*Len_Thi)^2;
Is     = ms*(0.302*Len_Sha)^2;
If     = mf*(0.475*Len_Foo)^2;

frame_count=size(Res.Traj.AngA_RHip,1);
% preallocate space
JF2D_RHip=zeros(frame_count,2);
JF2D_RKne=zeros(frame_count,2);
JF2D_RAnk=zeros(frame_count,2);
JF2D_LHip=zeros(frame_count,2);
JF2D_LKne=zeros(frame_count,2);
JF2D_LAnk=zeros(frame_count,2);

JM2D_RHip=zeros(frame_count,1);
JM2D_RKne=zeros(frame_count,1);
JM2D_RAnk=zeros(frame_count,1);
JM2D_LHip=zeros(frame_count,1);
JM2D_LKne=zeros(frame_count,1);
JM2D_LAnk=zeros(frame_count,1);

if aux.plotStick==1
figure; hold on;title([aux.TrialName]);
set(gcf,'Position',[1000 200 400 400],'defaultLineLineWidth',1.5);
end
%% ID
for i=1:frame_count
    xrh=Res.Traj.Traj_RHJC(i,2);
    yrh=Res.Traj.Traj_RHJC(i,3);
    xrk=Res.Traj.Traj_RKJC(i,2);
    yrk=Res.Traj.Traj_RKJC(i,3);
    xra=Res.Traj.Traj_RAJC(i,2);
    yra=Res.Traj.Traj_RAJC(i,3);
    

    xlh=Res.Traj.Traj_LHJC(i,2);
    ylh=Res.Traj.Traj_LHJC(i,3);
    xlk=Res.Traj.Traj_LKJC(i,2);
    ylk=Res.Traj.Traj_LKJC(i,3);
    xla=Res.Traj.Traj_LAJC(i,2);
    yla=Res.Traj.Traj_LAJC(i,3);
    
    xrtm=Res.Traj.COMP_RThi(i,2);
    yrtm=Res.Traj.COMP_RThi(i,3);
    xrsm=Res.Traj.COMP_RSha(i,2);
    yrsm=Res.Traj.COMP_RSha(i,3);
    xrfm=Res.Traj.COMP_RFoo(i,2);
    yrfm=Res.Traj.COMP_RFoo(i,3);
    
    xltm=Res.Traj.COMP_LThi(i,2);
    yltm=Res.Traj.COMP_LThi(i,3);
    xlsm=Res.Traj.COMP_LSha(i,2);
    ylsm=Res.Traj.COMP_LSha(i,3);
    xlfm=Res.Traj.COMP_LFoo(i,2);
    ylfm=Res.Traj.COMP_LFoo(i,3);
    
    xrtmddot=Res.Traj.COMA_RThi(i,2);
    yrtmddot=Res.Traj.COMA_RThi(i,3);
    xrsmddot=Res.Traj.COMA_RSha(i,2);
    yrsmddot=Res.Traj.COMA_RSha(i,3);
    xrfmddot=Res.Traj.COMA_RFoo(i,2);
    yrfmddot=Res.Traj.COMA_RFoo(i,3);
    
    xltmddot=Res.Traj.COMA_LThi(i,2);
    yltmddot=Res.Traj.COMA_LThi(i,3);
    xlsmddot=Res.Traj.COMA_LSha(i,2);
    ylsmddot=Res.Traj.COMA_LSha(i,3);
    xlfmddot=Res.Traj.COMA_LFoo(i,2);
    ylfmddot=Res.Traj.COMA_LFoo(i,3);
    
    qrtddot=Res.Traj.AngA_RThi(i);
    qrsddot=Res.Traj.AngA_RSha(i);
    qrfddot=Res.Traj.AngA_RFoo(i);
    qltddot=Res.Traj.AngA_LThi(i);
    qlsddot=Res.Traj.AngA_LSha(i);
    qlfddot=Res.Traj.AngA_LFoo(i);
    
    
    Frgx=Res.GRFMCOP.F2_g(i,2);
    Frgy=Res.GRFMCOP.F2_g(i,3);
    Flgx=Res.GRFMCOP.F1_g(i,2);
    Flgy=Res.GRFMCOP.F1_g(i,3);
    
    Frax=Frgx-mf*xrfmddot;
    Fray=Frgy-mf*yrfmddot-mf*g;
    Flax=Flgx-mf*xlfmddot;
    Flay=Flgy-mf*ylfmddot-mf*g;
    
    copRx=Res.GRFMCOP.P2_g(i,2);
    copRy=Res.GRFMCOP.P2_g(i,3);
    copLx=Res.GRFMCOP.P1_g(i,2);
    copLy=Res.GRFMCOP.P1_g(i,3);
    
    
    Frkx=Frax-ms*xrsmddot;
    Frky=Fray-ms*yrsmddot-ms*g;
    Flkx=Flax-ms*xlsmddot;
    Flky=Flay-ms*ylsmddot-ms*g;
    
    Frhx=Frkx-mt*xrtmddot;
    Frhy=Frky-mt*yrtmddot-mt*g;
    Flhx=Flkx-mt*xltmddot;
    Flhy=Flky-mt*yltmddot-mt*g;
    
    Mrg=Frgx.*(yrfm-copRy)+Frgy.*(copRx-xrfm);
    Mlg=Flgx.*(ylfm-copLy)+Flgy.*(copLx-xlfm);
    
    Tra=Mrg+Frax*(yra-yrfm)+Fray*(xrfm-xra)-If*qrfddot;
    Tla=Mlg+Flax*(yla-ylfm)+Flay*(xlfm-xla)-If*qlfddot;
    
    Trk=Frax*(yrsm-yra)+Fray*(xra-xrsm)+Frkx*(yrk-yrsm)+Frky*(xrsm-xrk)+Tra-Is*qrsddot;
    Tlk=Flax*(ylsm-yla)+Flay*(xla-xlsm)+Flkx*(ylk-ylsm)+Flky*(xlsm-xlk)+Tla-Is*qlsddot;
    
    Trh=Frkx*(yrtm-yrk)+Frky*(xrk-xrtm)+Frhx*(yrh-yrtm)+Frhy*(xrtm-xrh)+Trk-It*qrtddot;
    Tlh=Flkx*(yltm-ylk)+Flky*(xlk-xltm)+Flhx*(ylh-yltm)+Flhy*(xltm-xlh)+Tlk-It*qltddot;
    
    JF2D_RHip(i,:)=[Frhx,Frhy];
    JF2D_RKne(i,:)=[Frkx,Frky];
    JF2D_RAnk(i,:)=[Frax,Fray];
    JF2D_LHip(i,:)=[Flhx,Flhy];
    JF2D_LKne(i,:)=[Flkx,Flky];
    JF2D_LAnk(i,:)=[Flax,Flay];
    
    JM2D_RHip(i,:)=Trh;
    JM2D_RKne(i,:)=Trk;
    JM2D_RAnk(i,:)=Tra;
    JM2D_LHip(i,:)=Tlh;
    JM2D_LKne(i,:)=Tlk;
    JM2D_LAnk(i,:)=Tla;
    
    if aux.plotStick==1
        if (mod(i,10)==1) || (i==frame_count)
            plot(xrtm,yrtm,'b*');
            plot(xrsm,yrsm,'b*');
            plot(xrfm,yrfm,'b*');
            
            plot(xltm,yltm,'g*');
            plot(xlsm,ylsm,'g*');
            plot(xlfm,ylfm,'g*');
            
            line([xrh,xrk],[yrh,yrk],'Color','b')
            line([xrk,xra],[yrk,yra],'Color','b')
            line([xlh,xlk],[ylh,ylk],'Color','g')
            line([xlk,xla],[ylk,yla],'Color','g')
            
            line([xra,Res.Traj.Traj_RHEE(i,2)],[yra,Res.Traj.Traj_RHEE(i,3)],'Color','b')
            line([xra,Res.Traj.Traj_RTOE(i,2)],[yra,Res.Traj.Traj_RTOE(i,3)],'Color','b')
            line([Res.Traj.Traj_RHEE(i,2),Res.Traj.Traj_RTOE(i,2)],...
                [Res.Traj.Traj_RHEE(i,3),Res.Traj.Traj_RTOE(i,3)],'Color','b')
            
            
            line([xla,Res.Traj.Traj_LHEE(i,2)],[yla,Res.Traj.Traj_LHEE(i,3)],'Color','g')
            line([xla,Res.Traj.Traj_LTOE(i,2)],[yla,Res.Traj.Traj_LTOE(i,3)],'Color','g')
            line([Res.Traj.Traj_LHEE(i,2),Res.Traj.Traj_LTOE(i,2)],...
                [Res.Traj.Traj_LHEE(i,3),Res.Traj.Traj_LTOE(i,3)],'Color','g')
            
            Fc=2000;
            line(copRx+[0,Frax/Fc],copRy+[0,Fray/Fc],'Color','k');
            line(copLx+[0,Flax/Fc],copLy+[0,Flay/Fc],'Color','k');
            axis equal
        end
    end
end
%%

[fb_mom,fa_mom]=butter(6,6*2/aux.TrajFrameRate);

JM2D_RHip=filtfilt(fb_mom,fa_mom,JM2D_RHip);
JM2D_RKne=filtfilt(fb_mom,fa_mom,JM2D_RKne);
JM2D_RAnk=filtfilt(fb_mom,fa_mom,JM2D_RAnk);
JM2D_LHip=filtfilt(fb_mom,fa_mom,JM2D_LHip);
JM2D_LKne=filtfilt(fb_mom,fa_mom,JM2D_LKne);
JM2D_LAnk=filtfilt(fb_mom,fa_mom,JM2D_LAnk);

%% Joint Power %%%%%%%%%%%%
JP2D_RHip = -JM2D_RHip.*Res.Traj.AngV_RHip;
JP2D_RKne = -JM2D_RKne.*Res.Traj.AngV_RKne;
JP2D_RAnk = -JM2D_RAnk.*Res.Traj.AngV_RAnk;
JP2D_LHip = -JM2D_LHip.*Res.Traj.AngV_LHip;
JP2D_LKne = -JM2D_LKne.*Res.Traj.AngV_LKne;
JP2D_LAnk = -JM2D_LAnk.*Res.Traj.AngV_LAnk;

%%
JForce2D  = struct('JF2D_RHip',JF2D_RHip,'JF2D_RKne',JF2D_RKne,'JF2D_RAnk',JF2D_RAnk,...
                   'JF2D_LHip',JF2D_LHip,'JF2D_LKne',JF2D_LKne,'JF2D_LAnk',JF2D_LAnk);
               
JMoment2D = struct('JM2D_RHip',JM2D_RHip,'JM2D_RKne',JM2D_RKne,'JM2D_RAnk',JM2D_RAnk,...
                   'JM2D_LHip',JM2D_LHip,'JM2D_LKne',JM2D_LKne,'JM2D_LAnk',JM2D_LAnk);
               
JPower2D  = struct('JP2D_RHip',JP2D_RHip,'JP2D_RKne',JP2D_RKne,'JP2D_RAnk',JP2D_RAnk,...
                   'JP2D_LHip',JP2D_LHip,'JP2D_LKne',JP2D_LKne,'JP2D_LAnk',JP2D_LAnk);


















