% change the peri part 
clear;clc;
set(0,'DefaultFigureVisible', 'on')
load('IMUVICONData20201124_175000.mat');

SubjectCounts=length(fieldnames(IMUVICONData));

plotAllFlag= 0 ;
GaitTypes={'Norm','Slow','Fast','Rmin','Lmin'};
%%
if plotAllFlag
%% check results
for subi=1 :SubjectCounts
    figure;hold on;
    cmap=colormap('hsv');
    set(gcf,'Position',[10 10 800 700],'defaultLineLineWidth',1.5);
    nr=4;nc=3;
    for typei=1:5
        GaitType=GaitTypes{typei};
        for trii=1:6 % TrialCounts
            eval(['trial=IMUVICONData.Subject',num2str(subi),'.',GaitType,'.Trial',num2str(trii),';']);
            traj=trial.Traj;
            GRF=trial.GRFMCOP;
            JM2D=trial.JM2D;
            time=[traj.t_mkr;traj.t_mkr+traj.Time_step];
            time=linspace(0,100,length(traj.t_mkr)*2).';
            
            c0=floor(64/5);
            subplot(nr,nc,1);hold on;title('Angle-Hip')
            plot(time,rad2deg([traj.AngP_RHip;traj.AngP_LHip]),'Color',cmap(typei*c0,:));
            grid minor
            subplot(nr,nc,2);hold on;title('Angle-Kne')
            plot(time,rad2deg([traj.AngP_RKne;traj.AngP_LKne]),'Color',cmap(typei*c0,:));
            grid minor
            subplot(nr,nc,3);hold on;title('Angle-Ank')
            plot(time,rad2deg([traj.AngP_RAnk;traj.AngP_LAnk]),'Color',cmap(typei*c0,:));
            grid minor
            
            
            subplot(nr,nc,1+3);hold on;title('Angle-Thi')
            plot(time,rad2deg([traj.AngP_RThi;traj.AngP_LThi]),'Color',cmap(typei*c0,:));
            grid minor
            subplot(nr,nc,2+3);hold on;title('Angle-Sha')
            plot(time,rad2deg([traj.AngP_RSha;traj.AngP_LSha]),'Color',cmap(typei*c0,:));
            grid minor
            subplot(nr,nc,3+3);hold on;title('Angle-Foo')
            plot(time,rad2deg([traj.AngP_RFoo;traj.AngP_LFoo]),'Color',cmap(typei*c0,:));
            grid minor
            
            
            subplot(nr,nc,1+6);hold on;title('Angle-Thi')
            plot(time,[GRF.F2_g(:,2);GRF.F1_g(:,2)],'Color',cmap(typei*c0,:));
            grid minor
            subplot(nr,nc,2+6);hold on;title('Angle-Sha')
            plot(time,[GRF.F2_g(:,3);GRF.F1_g(:,3)],'Color',cmap(typei*c0,:));
            grid minor
            
            
            subplot(nr,nc,1+9);hold on;title('Moment-Thi')
            plot(time,[JM2D.JM2D_RHip;JM2D.JM2D_LHip],'Color',cmap(typei*c0,:));
            grid minor
            subplot(nr,nc,2+9);hold on;title('Moment-Sha')
            plot(time,[JM2D.JM2D_RKne;JM2D.JM2D_LKne],'Color',cmap(typei*c0,:));
            grid minor
            subplot(nr,nc,3+9);hold on;title('Moment-Foo')
            plot(time,[JM2D.JM2D_RAnk;JM2D.JM2D_LAnk],'Color',cmap(typei*c0,:));
            grid minor
        end
    end
%     legend show
    suptitle(['Subj=',num2str(subi)]);
end
end

%% Collect and average

plot_sing=0;
idii=0;

NSubj=12;
Ntri=6;

%%
for subi=1:NSubj
    %%
    eval(['Subj=IMUVICONData.Subject',num2str(subi),';']);
    for typei=1: 5
        GaitType=GaitTypes{typei};
        
    BH=Subj.aux.BH/100;
    BM=Subj.aux.BM;
    LegL=(Subj.aux.Thigh_Len+Subj.aux.Shank_Len+Subj.aux.FootH_Len)/100;
    
    for trii=1:Ntri
        eval(['trial=Subj.',GaitType,'.Trial',num2str(trii),';']);
        eval(['Res_Sta=Subj.',GaitType,'.Res_Sta;']);

        idii=idii+1;
        id_all(idii,:)=[idii,subi,typei,trii];
        Err_Shank_allrms(idii,:)=trial.IMURes.Err_Shank_rms;
        Err_Shank(idii,:)=trial.IMURes.Err_Shank_totrms;
        Err_event(idii,:)=trial.IMURes.Err_event;
        TimeLen(idii,:)=[trial.ViconLen,trial.IMULen,trial.LenError];
        Range_Shank(idii,:)=[range([trial.Traj.Traj_RAJC(:,2);trial.Traj.Traj_LAJC(:,2)]),...
                     range([trial.Traj.Traj_RAJC(:,3);trial.Traj.Traj_LAJC(:,3)]),...
                     rad2deg(range([trial.Traj.AngP_RSha;trial.Traj.AngP_RSha]))];
        Tstep_mkr(idii,:)=trial.Traj.Time_step;
        Tstep_IMU(idii,:)=trial.IMURes.Tstep;
        Lstep(idii,:)=trial.Traj.Traj_LAJC(end,2)-trial.Traj.Traj_RAJC(1,2);

        
        Lstep_L(subi,typei,trii)=trial.Traj.Lstep_L;
        Lstep_R(subi,typei,trii)=trial.Traj.Lstep_R;
        Lstep_RdL(subi,typei,trii)=trial.Traj.Lstep_RdL;
        Lstep_LdA(subi,typei,trii)=trial.Traj.Lstep_L/(trial.Traj.Lstep_R+trial.Traj.Lstep_L);
        Lstep_RdA(subi,typei,trii)=trial.Traj.Lstep_R/(trial.Traj.Lstep_R+trial.Traj.Lstep_L);
        Velo_All(subi,typei,trii)=(trial.Traj.Traj_OPGT(end,2)-trial.Traj.Traj_OPGT(1,2))/trial.Traj.Time_step;
        
        
        MOCOffsetx=trial.Traj.Traj_RAJC(round(end/2),2);
        
        pJoi_trial=[trial.Traj.Traj_R_GT(:,2:3),trial.Traj.Traj_RKJC(:,2:3),trial.Traj.Traj_RAJC(:,2:3),...        
            trial.Traj.Traj_L_GT(:,2:3),trial.Traj.Traj_LKJC(:,2:3),trial.Traj.Traj_LAJC(:,2:3)];
        pJoi_trial(:,1:2:end)=pJoi_trial(:,1:2:end)-MOCOffsetx;
        
        pdotJoi_trial=[trial.Traj.TrajV_R_GT(:,2:3),trial.Traj.TrajV_RKJC(:,2:3),trial.Traj.TrajV_RAJC(:,2:3),...        
            trial.Traj.TrajV_L_GT(:,2:3),trial.Traj.TrajV_LKJC(:,2:3),trial.Traj.TrajV_LAJC(:,2:3)];
        pddotJoi_trial=[trial.Traj.TrajA_R_GT(:,2:3),trial.Traj.TrajA_RKJC(:,2:3),trial.Traj.TrajA_RAJC(:,2:3),...        
            trial.Traj.TrajA_L_GT(:,2:3),trial.Traj.TrajA_LKJC(:,2:3),trial.Traj.TrajA_LAJC(:,2:3)];
        
        
        qSeg_trial_old=[(trial.Traj.Traj_OPGT(:,2)-trial.Traj.Traj_RAJC(1,2))/LegL,... % use BH to norm.
                       trial.Traj.Traj_OPGT(:,3)/LegL,... % use BH to norm.
                       trial.Traj.AngP_Pelv-Res_Sta.AngP_Pelv_Sta,...
                       trial.Traj.AngP_RThi-Res_Sta.AngP_RThi_Sta,...
                       trial.Traj.AngP_RSha-Res_Sta.AngP_RSha_Sta,...
                       trial.Traj.AngP_RFoo-Res_Sta.AngP_RFoo_Sta,...
                       trial.Traj.AngP_LThi-Res_Sta.AngP_LThi_Sta,...
                       trial.Traj.AngP_LSha-Res_Sta.AngP_LSha_Sta,...
                       trial.Traj.AngP_LFoo-Res_Sta.AngP_LFoo_Sta];

        qSeg_trial=[(trial.Traj.Traj_OPGT(:,2)-trial.Traj.Traj_RAJC(1,2))/LegL,... % use BH to norm.
                       (trial.Traj.Traj_OPGT(:,3))/LegL,... % use BH to norm.
                       trial.Traj.AngP_Pelv-mean(trial.Traj.AngP_Pelv),...
                       trial.Traj.AngP_RThi-Res_Sta.AngP_RThi_Sta,...
                       trial.Traj.AngP_RSha-Res_Sta.AngP_RSha_Sta,...
                       trial.Traj.AngP_RFoo-Res_Sta.AngP_RFoo_Sta,...
                       trial.Traj.AngP_LThi-Res_Sta.AngP_LThi_Sta,...
                       trial.Traj.AngP_LSha-Res_Sta.AngP_LSha_Sta,...
                       trial.Traj.AngP_LFoo-Res_Sta.AngP_LFoo_Sta];

        Velo(idii,:)=(trial.Traj.Traj_OPGT(end,2)-trial.Traj.Traj_OPGT(1,2))/trial.Traj.Time_step;
             
        qdotSeg_trial=[trial.Traj.TrajV_OPGT(:,2)/LegL,...
                       trial.Traj.TrajV_OPGT(:,3)/LegL,...
                        trial.Traj.AngV_Pelv,...
                        trial.Traj.AngV_RThi,trial.Traj.AngV_RSha,trial.Traj.AngV_RFoo,...
                        trial.Traj.AngV_LThi,trial.Traj.AngV_LSha,trial.Traj.AngV_LFoo];
                    
        qddotSeg_trial=[trial.Traj.TrajA_OPGT(:,2)/LegL,...
                        trial.Traj.TrajA_OPGT(:,3)/LegL,...
                        trial.Traj.AngA_Pelv,...
                        trial.Traj.AngA_RThi,trial.Traj.AngA_RSha,trial.Traj.AngA_RFoo,...
                        trial.Traj.AngA_LThi,trial.Traj.AngA_LSha,trial.Traj.AngA_LFoo];
        
        
        q_trial_old=[qSeg_trial_old(:,1:3),...
                 trial.Traj.AngP_RHip-Res_Sta.AngP_RHip_Sta,...
                 trial.Traj.AngP_RKne-Res_Sta.AngP_RKne_Sta,...
                 trial.Traj.AngP_RAnk-Res_Sta.AngP_RAnk_Sta,...
                 trial.Traj.AngP_LHip-Res_Sta.AngP_LHip_Sta,...
                 trial.Traj.AngP_LKne-Res_Sta.AngP_LKne_Sta,...
                 trial.Traj.AngP_LAnk-Res_Sta.AngP_LAnk_Sta];
             
        q_trial=[qSeg_trial(:,1:3),...
                 trial.Traj.AngP_RThi-Res_Sta.AngP_RThi_Sta-(trial.Traj.AngP_Pelv-mean(trial.Traj.AngP_Pelv)),...
                 trial.Traj.AngP_RKne-Res_Sta.AngP_RKne_Sta,...
                 trial.Traj.AngP_RAnk-Res_Sta.AngP_RAnk_Sta,...
                 trial.Traj.AngP_LThi-Res_Sta.AngP_LThi_Sta-(trial.Traj.AngP_Pelv-mean(trial.Traj.AngP_Pelv)),...
                 trial.Traj.AngP_LKne-Res_Sta.AngP_LKne_Sta,...
                 trial.Traj.AngP_LAnk-Res_Sta.AngP_LAnk_Sta];
             
             if plot_sing==1
                 figure;hold on;
                 plot(rad2deg(qSeg_trial_old(:,[3 4 7])),'--');
                 plot(rad2deg(qSeg_trial(:,[3 4 7])));
                 figure;hold on;
                 plot(rad2deg(q_trial_old(:,[3 4 7])),'--');
                 plot(rad2deg(q_trial(:,[3 4 7])));
             end
        
        qdot_trial=[qdotSeg_trial(:,1:3),...
                    trial.Traj.AngV_RHip,trial.Traj.AngV_RKne,trial.Traj.AngV_RAnk,...
            trial.Traj.AngV_LHip,trial.Traj.AngV_LKne,trial.Traj.AngV_LAnk];
        qddot_trial=[qddotSeg_trial(:,1:3),...
                    trial.Traj.AngA_RHip,trial.Traj.AngA_RKne,trial.Traj.AngA_RAnk,...
            trial.Traj.AngA_LHip,trial.Traj.AngA_LKne,trial.Traj.AngA_LAnk];
        
        JM2D_trial=[trial.JM2D.JM2D_RHip,trial.JM2D.JM2D_RKne,trial.JM2D.JM2D_RAnk,...
            trial.JM2D.JM2D_LHip,trial.JM2D.JM2D_LKne,trial.JM2D.JM2D_LAnk]/BM/LegL;  % ADD: use BM*BH to norm.
        
        GRF_trial=[trial.GRFMCOP.F2_g(:,2:3),trial.GRFMCOP.F1_g(:,2:3)] /BM; %  ADD: use BM to norm.
        
        
        t_mkr=trial.Traj.t_mkr;
        t_pst=linspace(t_mkr(1),t_mkr(end),51).';
        q(idii,:,:)=interp1(t_mkr,q_trial,t_pst);
        qdot(idii,:,:)=interp1(t_mkr,qdot_trial,t_pst);
        qddot(idii,:,:)=interp1(t_mkr,qddot_trial,t_pst);
        qSeg(idii,:,:)=interp1(t_mkr,qSeg_trial,t_pst);
        qdotSeg(idii,:,:)=interp1(t_mkr,qdotSeg_trial,t_pst);
        qddotSeg(idii,:,:)=interp1(t_mkr,qddotSeg_trial,t_pst);
        pJoi(idii,:,:)=interp1(t_mkr,pJoi_trial,t_pst);
        pdotJoi(idii,:,:)=interp1(t_mkr,pdotJoi_trial,t_pst);
        pddotJoi(idii,:,:)=interp1(t_mkr,pddotJoi_trial,t_pst);
        MomJoi(idii,:,:)=interp1(t_mkr,JM2D_trial,t_pst);
        GRFall(idii,:,:)=interp1(t_mkr,GRF_trial,t_pst);
        
        qmin(idii,:)=min(q_trial);
        qdotmin(idii,:)=min(qdot_trial);
        qddotmin(idii,:)=min(qddot_trial);
        qSegmin(idii,:)=min(qSeg_trial);
        qdotSegmin(idii,:)=min(qdotSeg_trial);
        qddotSegmin(idii,:)=min(qddotSeg_trial);
        pJoimin(idii,:)=min(pJoi_trial);    
        pdotJoimin(idii,:)=min(pdotJoi_trial);      
        pddotJoimin(idii,:)=min(pddotJoi_trial);         
        JM2Dmin(idii,:)=min(JM2D_trial);
           
        GRFmin(idii,:)=min(GRF_trial);

        qmax(idii,:)=max(q_trial);
        qdotmax(idii,:)=max(qdot_trial);
        qddotmax(idii,:)=max(qddot_trial);
        qSegmax(idii,:)=max(qSeg_trial);
        qdotSegmax(idii,:)=max(qdotSeg_trial);
        qddotSegmax(idii,:)=max(qddotSeg_trial);
        pJoimax(idii,:)=max(pJoi_trial);    
        pdotJoimax(idii,:)=max(pdotJoi_trial);      
        pddotJoimax(idii,:)=max(pddotJoi_trial);     
        JM2Dmax(idii,:)=max(JM2D_trial);
        GRFmax(idii,:)=max(GRF_trial);

        
        qSeg_perierr(idii,:)=[qSeg_trial(end,1:3)-qSeg_trial(1,1:3),...
                              qSeg_trial(end,4:6)-qSeg_trial(1,7:9),...
                              qSeg_trial(end,7:9)-qSeg_trial(1,4:6)];
        qdotSeg_perierr(idii,:)=[qdotSeg_trial(end,1:3)-qdotSeg_trial(1,1:3),...
                              qdotSeg_trial(end,4:6)-qdotSeg_trial(1,7:9),...
                              qdotSeg_trial(end,7:9)-qdotSeg_trial(1,4:6)];
        qddotSeg_perierr(idii,:)=[qddotSeg_trial(end,1:3)-qddotSeg_trial(1,1:3),...
                              qddotSeg_trial(end,4:6)-qddotSeg_trial(1,7:9),...
                              qddotSeg_trial(end,7:9)-qddotSeg_trial(1,4:6)];
        
        q_perierr(idii,:)=[q_trial(end,1:3)-q_trial(1,1:3),...
                           q_trial(end,4:6)-q_trial(1,7:9),...
                           q_trial(end,7:9)-q_trial(1,4:6)];
        qdot_perierr(idii,:)=[qdot_trial(end,1:3)-qdot_trial(1,1:3),...
                              qdot_trial(end,4:6)-qdot_trial(1,7:9),...
                              qdot_trial(end,7:9)-qdot_trial(1,4:6)];
        qddot_perierr(idii,:)=[qddot_trial(end,1:3)-qddot_trial(1,1:3),...
                               qddot_trial(end,4:6)-qddot_trial(1,7:9),...
                               qddot_trial(end,7:9)-qddot_trial(1,4:6)];
        
        JM2D_perierr(idii,:)=[JM2D_trial(end,1:3)-JM2D_trial(1,4:6),...
                              JM2D_trial(end,4:6)-JM2D_trial(1,1:3)];
        
        GRF_perierr(idii,:)=[GRF_trial(end,1:2)-GRF_trial(1,3:4),...
                             GRF_trial(end,3:4)-GRF_trial(1,1:2)];% [trial.GRFMCOP.F2_g(end)-trial.GRFMCOP.F1_g(1)];
        
        
        
    end
    end
end

%%
Lstep_RdL_mn=mean(Lstep_RdL,3);
Lstep_LdA_mn=mean(Lstep_LdA,3);
Lstep_RdA_mn=mean(Lstep_RdA,3);

Lstep_LdA_mn_mn=mean(Lstep_LdA_mn);
Lstep_LdA_mn_sd=std(Lstep_LdA_mn);
Lstep_RdA_mn_mn=mean(Lstep_RdA_mn);
Lstep_RdA_mn_sd=std(Lstep_RdA_mn);

Lstep_L_mn=mean(Lstep_L,3);
Lstep_R_mn=mean(Lstep_R,3);

Lstep_RdL_mn2=mean(Lstep_R./Lstep_L,3);
Lstep_LdR_mn2=mean(Lstep_L./Lstep_R,3);

Velo_All_mn=mean(Velo_All,3);

Lstep_Asym=[Lstep_RdL_mn2(:,4),Lstep_LdR_mn2(:,5)];

return;
%%
Err_Shank_allrms_SubjAve=zeros(NSubj,size(Err_Shank_allrms,2));
Err_Shank_SubjAve=zeros(NSubj,size(Err_Shank,2));
Err_event_SubjAve=zeros(NSubj,size(Err_event,2));
for subj=1:NSubj
    Err_Shank_allrms_SubjAve(subj,:)=mean(Err_Shank_allrms(5*Ntri*(subj-1)+1:5*Ntri*subj,:));
    Err_Shank_SubjAve(subj,:)=mean(Err_Shank(5*Ntri*(subj-1)+1:5*Ntri*subj,:));
    Err_event_SubjAve(subj,:)=mean(Err_event(5*Ntri*(subj-1)+1:5*Ntri*subj,:));
end

%%

id_Norm=reshape((((1:NSubj).'-1)*5*Ntri+(0*Ntri+1:1*Ntri)).',[],1);
id_Slow=reshape((((1:NSubj).'-1)*5*Ntri+(1*Ntri+1:2*Ntri)).',[],1);
id_Fast=reshape((((1:NSubj).'-1)*5*Ntri+(2*Ntri+1:3*Ntri)).',[],1);
id_Rmin=reshape((((1:NSubj).'-1)*5*Ntri+(3*Ntri+1:4*Ntri)).',[],1);
id_Lmin=reshape((((1:NSubj).'-1)*5*Ntri+(4*Ntri+1:5*Ntri)).',[],1);

id_normal=reshape((((1:NSubj).'-1)*5*Ntri+(1:3*Ntri)).',[],1);
id_abnormal=reshape((((1:NSubj).'-1)*5*Ntri+(3*Ntri+1:5*Ntri)).',[],1);

Types=cell(length(Velo),1);
for i=1:length(id_Lmin)
Types{id_Norm(i)}='Norm';
Types{id_Slow(i)}='Slow';
Types{id_Fast(i)}='Fast';
Types{id_Rmin(i)}='Rmin';
Types{id_Lmin(i)}='Lmin';
end
%%

PostRes.RMSE_Shank_mn=[mean(Err_Shank(id_Slow,:));mean(Err_Shank(id_Norm,:));mean(Err_Shank(id_Fast,:));mean(Err_Shank(id_Rmin,:));mean(Err_Shank(id_Lmin,:));mean(Err_Shank(:,:))];
PostRes.RMSE_Shank_sd=[std(Err_Shank(id_Slow,:));std(Err_Shank(id_Norm,:));std(Err_Shank(id_Fast,:));std(Err_Shank(id_Rmin,:));std(Err_Shank(id_Lmin,:));std(Err_Shank(:,:))];

PostRes.Event_mn=[mean(abs(reshape(Err_event(id_Slow,:),[],1)));
    mean(abs(reshape(Err_event(id_Norm,:),[],1)));
    mean(abs(reshape(Err_event(id_Fast,:),[],1)));
    mean(abs(reshape(Err_event(id_Rmin,:),[],1)));
    mean(abs(reshape(Err_event(id_Lmin,:),[],1)));
    mean(abs(reshape(Err_event(:,:),[],1)))];
PostRes.Event_sd=[std(abs(reshape(Err_event(id_Slow,:),[],1)));
    std(abs(reshape(Err_event(id_Norm,:),[],1)));
    std(abs(reshape(Err_event(id_Fast,:),[],1)));
    std(abs(reshape(Err_event(id_Rmin,:),[],1)));
    std(abs(reshape(Err_event(id_Lmin,:),[],1)));
    std(abs(reshape(Err_event(:,:),[],1)))];

PostRes.Velo_mn=[mean(Velo(id_Slow,:));mean(Velo(id_Norm,:));mean(Velo(id_Fast,:));mean(Velo(id_Rmin,:));mean(Velo(id_Lmin,:));mean(Velo(:,:))];
PostRes.Velo_sd=[std(Velo(id_Slow,:));std(Velo(id_Norm,:));std(Velo(id_Fast,:));std(Velo(id_Rmin,:));std(Velo(id_Lmin,:));std(Velo(:,:))];

PostRes.Lstep_mn=[mean(Lstep(id_Slow,:));mean(Lstep(id_Norm,:));mean(Lstep(id_Fast,:));mean(Lstep(id_Rmin,:));mean(Lstep(id_Lmin,:));mean(Lstep(:,:))];
PostRes.Lstep_sd=[std(Lstep(id_Slow,:));std(Lstep(id_Norm,:));std(Lstep(id_Fast,:));std(Lstep(id_Rmin,:));std(Lstep(id_Lmin,:));std(Lstep(:,:))];

ErrTable=cell(6,4);
Err_mn=[PostRes.Lstep_mn,PostRes.Velo_mn,PostRes.Event_mn*1000,PostRes.RMSE_Shank_mn(:,1:2)*100,PostRes.RMSE_Shank_mn(:,3)];
Err_sd=[PostRes.Lstep_sd,PostRes.Velo_sd,PostRes.Event_sd*1000,PostRes.RMSE_Shank_sd(:,1:2)*100,PostRes.RMSE_Shank_sd(:,3)];
Err_Len=[2 2 1 2 2 2];
for i=1:6
    for j=1:6
        form=['%.',num2str(Err_Len(j)),'f'];
        ErrTable{i,j}=[num2str(Err_mn(i,j),form),...
            ' (',num2str(Err_sd(i,j),form),')'];
    end
end

%%
[p,tbl,stats] = anova1(Velo,Types);
[c,m,h,gnames]=multcompare(stats);
PostRes.Velo_anova=[p;c(:,end)];

[p,tbl,stats] = anova1(Lstep,Types);
[c,m,h,gnames]=multcompare(stats);
PostRes.Lstep_anova=[p;c(:,end)];

[p,tbl,stats] = anova1(abs([Err_event(:,1);Err_event(:,2)]),[Types;Types]);
[c,m,h,gnames]=multcompare(stats);
PostRes.Event_anova=[p;c(:,end)];

for i=1:3
[p,tbl,stats] = anova1(Err_Shank(:,i),Types);
[c,m,h,gnames]=multcompare(stats);
PostRes.RMSE_Shank_anova(:,i)=[p;c(:,end)];
end    
Err_anova=[PostRes.Lstep_anova,PostRes.Velo_anova,PostRes.Event_anova,PostRes.RMSE_Shank_anova];


ErrPst_Shank=Err_Shank./Range_Shank;
ErrPst_event=[Err_event(:,1)./Tstep,Err_event(:,2)./Tstep];

[p,tbl,stats] = anova1(abs([ErrPst_event(:,1);ErrPst_event(:,2)]),[Types;Types]);
[c,m,h,gnames]=multcompare(stats);
PostRes.EventPst_anova=[p;c(:,end)];

for i=1:3
[p,tbl,stats] = anova1(ErrPst_Shank(:,i),Types);
[c,m,h,gnames]=multcompare(stats);
PostRes.RMSEPst_Shank_anova(:,i)=[p;c(:,end)];
end    


[PostRes.ttest_Velo.h,PostRes.ttest_Velo.p]=ttest2(Velo(id_normal,:),Velo(id_abnormal,:));

[PostRes.ttest_HSevent.h,PostRes.ttest_HSevent.p]=ttest2(abs(reshape(Err_event(id_normal,:),[],1)),abs(reshape(Err_event(id_abnormal,:),[],1)));

[PostRes.ttest_shank1.h,PostRes.ttest_shank1.p]=ttest2(Err_Shank(id_normal,1),Err_Shank(id_abnormal,1));
[PostRes.ttest_shank2.h,PostRes.ttest_shank2.p]=ttest2(Err_Shank(id_normal,2),Err_Shank(id_abnormal,2));
[PostRes.ttest_shank3.h,PostRes.ttest_shank3.p]=ttest2(Err_Shank(id_normal,3),Err_Shank(id_abnormal,3));

[ttest_HSeventpst.h,ttest_HSeventpst.p]=ttest2(reshape(ErrPst_event(id_normal,:),[],1),reshape(ErrPst_event(id_abnormal,:),[],1));

[PostRes.ttest_shankpst1.h,PostRes.ttest_shankpst1.p]=ttest2(ErrPst_Shank(id_normal,1),ErrPst_Shank(id_abnormal,1));
[PostRes.ttest_shankpst2.h,PostRes.ttest_shankpst2.p]=ttest2(ErrPst_Shank(id_normal,2),ErrPst_Shank(id_abnormal,2));
[PostRes.ttest_shankpst3.h,PostRes.ttest_shankpst3.p]=ttest2(ErrPst_Shank(id_normal,3),ErrPst_Shank(id_abnormal,3));




%%

PostRes.qmin_mn=mean(qmin);
PostRes.qdotmin_mn=mean(qdotmin);
PostRes.qddotmin_mn=mean(qddotmin);
PostRes.qSegmin_mn=mean(qSegmin);
PostRes.qdotSegmin_mn=mean(qdotSegmin);
PostRes.qddotSegmin_mn=mean(qddotSegmin);

PostRes.pJoimin_mn=mean(pJoimin);
PostRes.pdotJoimin_mn=mean(pdotJoimin);
PostRes.pddotJoimin_mn=mean(pddotJoimin);
PostRes.JM2Dmin_mn=mean(JM2Dmin);
PostRes.GRFmin_mn=mean(GRFmin);



PostRes.qmax_mn=mean(qmax);
PostRes.qdotmax_mn=mean(qdotmax);
PostRes.qddotmax_mn=mean(qddotmax);
PostRes.qSegmax_mn=mean(qSegmax);
PostRes.qdotSegmax_mn=mean(qdotSegmax);
PostRes.qddotSegmax_mn=mean(qddotSegmax);

PostRes.pJoimax_mn=mean(pJoimax);
PostRes.pdotJoimax_mn=mean(pdotJoimax);
PostRes.pddotJoimax_mn=mean(pddotJoimax);
PostRes.JM2Dmax_mn=mean(JM2Dmax);
PostRes.GRFmax_mn=mean(GRFmax);


PostRes.q_perierr_mn=mean(abs(q_perierr));
PostRes.qdot_perierr_mn=mean(abs(qdot_perierr));
PostRes.qddot_perierr_mn=mean(abs(qddot_perierr));
PostRes.qSeg_perierr_mn=mean(abs(qSeg_perierr));
PostRes.qdotSeg_perierr_mn=mean(abs(qdotSeg_perierr));
PostRes.qddotSeg_perierr_mn=mean(abs(qddotSeg_perierr));
PostRes.JM2D_perierr_mn=mean(abs(JM2D_perierr));
PostRes.GRF_perierr_mn=mean(abs(GRF_perierr));

PostRes.q_perierr_max=max(abs(q_perierr));
PostRes.qdot_perierr_max=max(abs(qdot_perierr));
PostRes.qddot_perierr_max=max(abs(qddot_perierr));
PostRes.qSeg_perierr_max=max(abs(qSeg_perierr));
PostRes.qdotSeg_perierr_max=max(abs(qdotSeg_perierr));
PostRes.qddotSeg_perierr_max=max(abs(qddotSeg_perierr));
PostRes.JM2D_perierr_max=max(abs(JM2D_perierr));
PostRes.GRF_perierr_max=max(abs(GRF_perierr));

PostRes.q_mn=squeeze(mean(q,1));
PostRes.qdot_mn=squeeze(mean(qdot,1));
PostRes.qddot_mn=squeeze(mean(qddot,1));
PostRes.qSeg_mn=squeeze(mean(qSeg,1));
PostRes.qdotSeg_mn=squeeze(mean(qdotSeg,1));
PostRes.qddotSeg_mn=squeeze(mean(qddotSeg,1));
PostRes.pJoi_mn=squeeze(mean(pJoi,1));
PostRes.pdotJoi_mn=squeeze(mean(pdotJoi,1));
PostRes.pddotJoi_mn=squeeze(mean(pddotJoi,1));
PostRes.MomJoi_mn=squeeze(mean(MomJoi,1));
PostRes.GRF_mn=squeeze(mean(GRFall,1));


PostRes.q_sd=squeeze(std(q,0,1));
PostRes.qdot_sd=squeeze(std(qdot,0,1));
PostRes.qddot_sd=squeeze(std(qddot,0,1));
PostRes.qSeg_sd=squeeze(std(qSeg,0,1));
PostRes.qdotSeg_sd=squeeze(std(qdotSeg,0,1));
PostRes.qddotSeg_sd=squeeze(std(qddotSeg,0,1));
PostRes.pJoi_sd=squeeze(std(pJoi,0,1));
PostRes.pdotJoi_sd=squeeze(std(pdotJoi,0,1));
PostRes.pddotJoi_sd=squeeze(std(pddotJoi,0,1));
PostRes.MomJoi_sd=squeeze(std(MomJoi,0,1));
PostRes.GRF_sd=squeeze(std(GRFall,0,1));

PostRes.q_min=squeeze(min(q,[],1));
PostRes.qdot_min=squeeze(min(qdot,[],1));
PostRes.qddot_min=squeeze(min(qddot,[],1));
PostRes.qSeg_min=squeeze(min(qSeg,[],1));
PostRes.qdotSeg_min=squeeze(min(qdotSeg,[],1));
PostRes.qddotSeg_min=squeeze(min(qddotSeg,[],1));
PostRes.pJoi_min=squeeze(min(pJoi,[],1));
PostRes.pdotJoi_min=squeeze(min(pdotJoi,[],1));
PostRes.pddotJoi_min=squeeze(min(pddotJoi,[],1));
PostRes.MomJoi_min=squeeze(min(MomJoi,[],1));
PostRes.GRF_min=squeeze(min(GRFall,[],1));

PostRes.q_max=squeeze(max(q,[],1));
PostRes.qdot_max=squeeze(max(qdot,[],1));
PostRes.qddot_max=squeeze(max(qddot,[],1));
PostRes.qSeg_min=squeeze(min(qSeg,[],1));
PostRes.qdotSeg_max=squeeze(max(qdotSeg,[],1));
PostRes.qddotSeg_max=squeeze(max(qddotSeg,[],1));
PostRes.pJoi_max=squeeze(max(pJoi,[],1));
PostRes.pdotJoi_max=squeeze(max(pdotJoi,[],1));
PostRes.pddotJoi_max=squeeze(max(pddotJoi,[],1));
PostRes.MomJoi_max=squeeze(max(MomJoi,[],1));
PostRes.GRF_max=squeeze(max(GRFall,[],1));


%%
C1= [110 110 110]/255;
C2= [79 129 189]/255;
C3= [255 48 48]/255;
C1S=[230 230 230]/255;
% C2S=[220 230 255]/255;% ori
% C3S=[255 230 230]/255;% ori
C2S=[210 210 255]/255;
C3S=[255 210 210]/255;
Jois={'Hip','Knee','Ankle'};
Segs={'Pelvis','Thigh','Shank','Foot'};
qNames={'xb','yb','qb','Hip','Knee','Ankle'};
figure;hold on;
set(gcf,'Position',[100 100 1200 700],'defaultLineLineWidth',1.5);
nr=4;nc=6;
t50=(0:50).';
for flag=1:2
    for i=1:2
         subplot(nr,nc,i);hold on;title([qNames{i},''])
        plot_mnsd(t50,PostRes.q_mn(:,i),PostRes.q_sd(:,i),flag,C2S,C2,'-',1.5);
         subplot(nr,nc,i+6);hold on;title([qNames{i},'dot'])
        plot_mnsd(t50,PostRes.qdot_mn(:,i),PostRes.qdot_sd(:,i),flag,C2S,C2,'-',1.5);
        subplot(nr,nc,i+12);hold on;title([qNames{i},'ddot'])
        plot_mnsd(t50,PostRes.qddot_mn(:,i),PostRes.qddot_sd(:,i),flag,C2S,C2,'-',1.5);
        subplot(nr,nc,i+18);hold on;title(['GRF'])
        plot_mnsd(t50,PostRes.GRF_mn(:,i),PostRes.GRF_sd(:,i),flag,C2S,C2,'-',1.5);
        plot_mnsd(t50+50,PostRes.GRF_mn(:,i+2),PostRes.GRF_sd(:,i+2),flag,C3S,C3,'-',1.5);

    end
    
    for i=3
        subplot(nr,nc,i);hold on;title([qNames{i},''])
        plot_mnsd(t50,rad2deg(PostRes.q_mn(:,i)),rad2deg(PostRes.q_sd(:,i)),flag,C2S,C2,'-',1.5);
        subplot(nr,nc,i+6);hold on;title([qNames{i},'dot'])
        plot_mnsd(t50,rad2deg(PostRes.qdot_mn(:,i)),rad2deg(PostRes.qdot_sd(:,i)),flag,C2S,C2,'-',1.5);
        subplot(nr,nc,i+12);hold on;title([qNames{i},'ddot'])
        plot_mnsd(t50,rad2deg(PostRes.qddot_mn(:,i)),rad2deg(PostRes.qddot_sd(:,i)),flag,C2S,C2,'-',1.5);
    end
    for i=4:6
        subplot(nr,nc,i);hold on;title([qNames{i},''])
        plot_mnsd(t50,rad2deg(PostRes.q_mn(:,i)),rad2deg(PostRes.q_sd(:,i)),flag,C2S,C2,'-',1.5);
        plot_mnsd(t50+50,rad2deg(PostRes.q_mn(:,i+3)),rad2deg(PostRes.q_sd(:,i+3)),flag,C3S,C3,'-',1.5);
        subplot(nr,nc,i+6);hold on;title([qNames{i},'dot'])
        plot_mnsd(t50,rad2deg(PostRes.qdot_mn(:,i)),rad2deg(PostRes.qdot_sd(:,i)),flag,C2S,C2,'-',1.5);
        plot_mnsd(t50+50,rad2deg(PostRes.qdot_mn(:,i+3)),rad2deg(PostRes.qdot_sd(:,i+3)),flag,C3S,C3,'-',1.5);
        subplot(nr,nc,i+12);hold on;title([qNames{i},'ddot'])
        plot_mnsd(t50,rad2deg(PostRes.qddot_mn(:,i)),rad2deg(PostRes.qddot_sd(:,i)),flag,C2S,C2,'-',1.5);
        plot_mnsd(t50+50,rad2deg(PostRes.qddot_mn(:,i+3)),rad2deg(PostRes.qddot_sd(:,i+3)),flag,C3S,C3,'-',1.5);
        
        subplot(nr,nc,i+18);hold on;title([qNames{i},' Moment'])
        plot_mnsd(t50,rad2deg(PostRes.MomJoi_mn(:,i-3)),rad2deg(PostRes.MomJoi_sd(:,i-3)),flag,C2S,C2,'-',1.5);
        plot_mnsd(t50+50,rad2deg(PostRes.MomJoi_mn(:,i-3+3)),rad2deg(PostRes.MomJoi_sd(:,i-3+3)),flag,C3S,C3,'-',1.5);
        
    end
end


figure;hold on;
set(gcf,'Position',[100 100 1200 700],'defaultLineLineWidth',1.5);
nr=2;nc=6;
t50=(0:50).';
pJNames={'Hipx','Hipy','Knex','Kney','Ankx','Anky'};
qSegNames={'xb','yb','qb','Thigh','Shank','Foot'};
for flag=1:2
    for i=1:6
        subplot(nr,nc,i);hold on;title(pJNames{i});
        plot_mnsd(t50,PostRes.pJoi_mn(:,i),PostRes.pJoi_sd(:,i),flag,C2S,C2,'-',1.5);
        plot_mnsd(t50+50,PostRes.pJoi_mn(:,i+6),PostRes.pJoi_sd(:,i+6),flag,C3S,C3,'-',1.5);
        
        
        subplot(nr,nc,i+6);hold on;title([qSegNames{i},''])
        if i<3
            plot_mnsd(t50,PostRes.qSeg_mn(:,i),PostRes.qSeg_sd(:,i),flag,C2S,C2,'-',1.5);
        elseif i==3
            plot_mnsd(t50,rad2deg(PostRes.qSeg_mn(:,i)),rad2deg(PostRes.qSeg_sd(:,i)),flag,C2S,C2,'-',1.5);
            
        elseif i>3
            plot_mnsd(t50,rad2deg(PostRes.qSeg_mn(:,i)),rad2deg(PostRes.qSeg_sd(:,i)),flag,C2S,C2,'-',1.5);
            plot_mnsd(t50+50,rad2deg(PostRes.qSeg_mn(:,i+3)),rad2deg(PostRes.qSeg_sd(:,i+3)),flag,C3S,C3,'-',1.5);
        end
        
    end
    
    
end
%%
figure;hold on;
set(gcf,'Position',[100 100 1200 700],'defaultLineLineWidth',0.5);
nr=4;nc=9;
flag=1;
N_SD=3;
for i=1:9
    subplot(nr,nc,i);hold on;title(['q'])
    if i>=3 r2d=180/pi; else r2d=1; end
    for ti=1:size(q,1)
        plot(t50,q(ti,:,i)*r2d);
    end
    plot_mnsd(t50,PostRes.q_mn(:,i)*r2d,PostRes.q_sd(:,i)*N_SD*r2d,flag,C2S,C2,'-',1.5);
    plot(t50,PostRes.q_min(:,i)*r2d,'k','LineWidth',1.5);
    plot(t50,PostRes.q_max(:,i)*r2d,'k','LineWidth',1.5);
    
    subplot(nr,nc,i+9);hold on;title(['qdot'])
    for ti=1:size(q,1)
        plot(t50,qdot(ti,:,i));
    end
    plot_mnsd(t50,PostRes.qdot_mn(:,i),PostRes.qdot_sd(:,i)*N_SD,flag,C2S,C2,'-',1.5);
    plot(t50,PostRes.qdot_min(:,i),'k','LineWidth',1.5);
    plot(t50,PostRes.qdot_max(:,i),'k','LineWidth',1.5);
    
    subplot(nr,nc,i+18);hold on;title(['qddot'])
    for ti=1:size(q,1)
        plot(t50,qddot(ti,:,i));
    end
    plot_mnsd(t50,PostRes.qddot_mn(:,i),PostRes.qddot_sd(:,i)*N_SD,flag,C2S,C2,'-',1.5);
    plot(t50,PostRes.qddot_min(:,i),'k','LineWidth',1.5);
    plot(t50,PostRes.qddot_max(:,i),'k','LineWidth',1.5);
    
    subplot(nr,nc,i+27);hold on;title(['tor'])
    if i>3
        for ti=1:size(q,1)
            plot(t50,MomJoi(ti,:,i-3));
        end
        plot_mnsd(t50,PostRes.MomJoi_mn(:,i-3),PostRes.MomJoi_sd(:,i-3)*N_SD,flag,C2S,C2,'-',1.5);
        plot(t50,PostRes.MomJoi_min(:,i-3),'k','LineWidth',1.5);
        plot(t50,PostRes.MomJoi_max(:,i-3),'k','LineWidth',1.5);
    
    end
end
%% check the relation between shank angle peri and thigh angle peri

qSeg_perierr_deg=qSeg_perierr;
x2020=deg2rad(-20:0.01:20);

% qSeg_perierr_deg(:,3:end)=rad2deg(qSeg_perierr_deg(:,3:end));
% x2020=-20:0.01:20;

lev=0.95;



[RThiEfit.f,RThiEfit.gof,RThiEfit.output] = fit(qSeg_perierr_deg(:,5),qSeg_perierr_deg(:,4),'poly1');
RThiEfit.fpr=predint(RThiEfit.f,x2020,lev);
PostRes.RThiEfit=RThiEfit;

[RFooEfit.f,RFooEfit.gof,RFooEfit.output] = fit(qSeg_perierr_deg(:,5),qSeg_perierr_deg(:,6),'poly1');
RFooEfit.fpr=predint(RFooEfit.f,x2020,lev);
PostRes.RFooEfit=RFooEfit;

[RThiSfit.f,RThiSfit.gof,RThiSfit.output] = fit(qSeg_perierr_deg(:,8),qSeg_perierr_deg(:,7),'poly1');
RThiSfit.fpr=predint(RThiSfit.f,x2020,lev);
PostRes.RThiSfit=RThiSfit;

[RFooSfit.f,RFooSfit.gof,RFooSfit.output] = fit(qSeg_perierr_deg(:,8),qSeg_perierr_deg(:,9),'poly1');
RFooSfit.fpr=predint(RFooSfit.f,x2020,lev);
PostRes.RFooSfit=RFooSfit;

disp(['p1=',num2str(RThiEfit.f.p1),'; p2=',num2str(RThiEfit.f.p2),'; gof=',num2str(RThiEfit.gof.rsquare)]); % (a)
disp(['p1=',num2str(RThiSfit.f.p1),'; p2=',num2str(RThiSfit.f.p2),'; gof=',num2str(RThiSfit.gof.rsquare)]);% (b)
disp(['p1=',num2str(RFooEfit.f.p1),'; p2=',num2str(RFooEfit.f.p2),'; gof=',num2str(RFooEfit.gof.rsquare)]);% (c)
disp(['p1=',num2str(RFooSfit.f.p1),'; p2=',num2str(RFooSfit.f.p2),'; gof=',num2str(RFooSfit.gof.rsquare)]);% (d)

disp(['y=',num2str(RThiEfit.f.p1,'%0.2f'),'x',num2str(RThiEfit.f.p2,'%+0.2f')]); % (a)
disp(['R^2=',num2str(RThiEfit.gof.adjrsquare,'%0.2f')]);
disp(['y=',num2str(RThiSfit.f.p1,'%0.2f'),'x',num2str(RThiSfit.f.p2,'%+0.2f')]); % (b)
disp(['R^2=',num2str(RThiSfit.gof.adjrsquare,'%0.2f')]);
disp(['y=',num2str(RFooEfit.f.p1,'%0.2f'),'x',num2str(RFooEfit.f.p2,'%+0.2f')]); % (c)
disp(['R^2=',num2str(RFooEfit.gof.adjrsquare,'%0.2f')]);
disp(['y=',num2str(RFooSfit.f.p1,'%0.2f'),'x',num2str(RFooSfit.f.p2,'%+0.2f')]); % (d)
disp(['R^2=',num2str(RFooSfit.gof.adjrsquare,'%0.2f')]);


[RybEfit.f,RybEfit.gof,RybEfit.output] = fit(qSeg_perierr_deg(:,5),qSeg_perierr_deg(:,2),'poly1');
RybEfit.fpr=predint(RybEfit.f,x2020,lev);
PostRes.ybEfit=RybEfit;

[RqbEfit.f,RqbEfit.gof,RqbEfit.output] = fit(qSeg_perierr_deg(:,5),qSeg_perierr_deg(:,3),'poly1');
RqbEfit.fpr=predint(RqbEfit.f,x2020,lev);
PostRes.qbEfit=RqbEfit;


[LybEfit.f,LybEfit.gof,LybEfit.output] = fit(qSeg_perierr_deg(:,8),qSeg_perierr_deg(:,2),'poly1');
LybEfit.fpr=predint(LybEfit.f,x2020,lev);
PostRes.ybEfit=LybEfit;

[LqbEfit.f,LqbEfit.gof,LqbEfit.output] = fit(qSeg_perierr_deg(:,8),qSeg_perierr_deg(:,3),'poly1');
LqbEfit.fpr=predint(LqbEfit.f,x2020,lev);
PostRes.qbEfit=LqbEfit;

% disp(['p1=',num2str(RybEfit.f.p1),'; p2=',num2str(RybEfit.f.p2),'; gof=',num2str(RybEfit.gof.adjrsquare)]);
% disp(['p1=',num2str(RqbEfit.f.p1),'; p2=',num2str(RqbEfit.f.p2),'; gof=',num2str(RqbEfit.gof.adjrsquare)]);
% disp(['p1=',num2str(LybEfit.f.p1),'; p2=',num2str(LybEfit.f.p2),'; gof=',num2str(LybEfit.gof.adjrsquare)]);
% disp(['p1=',num2str(LqbEfit.f.p1),'; p2=',num2str(LqbEfit.f.p2),'; gof=',num2str(LqbEfit.gof.adjrsquare)]);

nr=2;nc=2;
C1= [110 110 110]/255;
C2= [79 129 189]/255;
C3= [255 48 48]/255;
cmap=colormap('Lines');
% C2= cmap(1,:);
% C3= cmap(2,:);
C2= [79 129 189]/255;
C3= [255 48 48]/255;
C4= cmap(3,:);
C5= cmap(4,:);
C6= cmap(5,:);

figure;hold on;
% set(gcf,'Position',[100 100 1200 600],'defaultLineLineWidth',1.5);
% set(gcf,'Position',[100 100 800 800],'defaultLineLineWidth',1.5);
set(gcf,'Position',[100 100 600 600],'defaultLineLineWidth',1.5,'defaultLineMarkerSize',5);
mkrWid=1;
subplot(nr,nc,1);hold on; %title('(a)'); % axis equal
plot(qSeg_perierr_deg(id_Norm,5),qSeg_perierr_deg(id_Norm,4),'*','Color',C2,'LineWidth',mkrWid);
plot(qSeg_perierr_deg(id_Slow,5),qSeg_perierr_deg(id_Slow,4),'*','Color',C3,'LineWidth',mkrWid);
plot(qSeg_perierr_deg(id_Fast,5),qSeg_perierr_deg(id_Fast,4),'*','Color',C4,'LineWidth',mkrWid);
plot(qSeg_perierr_deg(id_Rmin,5),qSeg_perierr_deg(id_Rmin,4),'*','Color',C5,'LineWidth',mkrWid);
plot(qSeg_perierr_deg(id_Lmin,5),qSeg_perierr_deg(id_Lmin,4),'*','Color',C6,'LineWidth',mkrWid);
plot(x2020,RThiEfit.f.p1*x2020+RThiEfit.f.p2,'k');
plot(x2020,RThiEfit.fpr(:,1),'k--');
plot(x2020,RThiEfit.fpr(:,2),'k--');
line([0 0],ylim(),'Color',C1,'LineWidth',0.5);
line(xlim(),[0 0],'Color',C1,'LineWidth',0.5);
xlabel('$\Delta q_{s1}$','Interpreter','latex','FontSize',14);
ylabel('$\Delta q_{t1}$','Interpreter','latex','FontSize',14);
% title('(a)','Units','normalized','Position',[0.5 -0.25]); 

subplot(nr,nc,2);hold on; %title('(b)'); % axis equal
plot(qSeg_perierr_deg(id_Norm,8),qSeg_perierr_deg(id_Norm,7),'*','Color',C2,'LineWidth',mkrWid);
plot(qSeg_perierr_deg(id_Slow,8),qSeg_perierr_deg(id_Slow,7),'*','Color',C3,'LineWidth',mkrWid);
plot(qSeg_perierr_deg(id_Fast,8),qSeg_perierr_deg(id_Fast,7),'*','Color',C4,'LineWidth',mkrWid);
plot(qSeg_perierr_deg(id_Rmin,8),qSeg_perierr_deg(id_Rmin,7),'*','Color',C5,'LineWidth',mkrWid);
plot(qSeg_perierr_deg(id_Lmin,8),qSeg_perierr_deg(id_Lmin,7),'*','Color',C6,'LineWidth',mkrWid);
% plot(qSeg_perierr_deg(id_normal,8),qSeg_perierr_deg(id_normal,7),'b*');
% plot(qSeg_perierr_deg(id_abnormal,8),qSeg_perierr_deg(id_abnormal,7),'r*');
plot(x2020,RThiSfit.f.p1*x2020+RThiSfit.f.p2,'k');
plot(x2020,RThiSfit.fpr(:,1),'k--');
plot(x2020,RThiSfit.fpr(:,2),'k--');
line([0 0],ylim(),'Color',C1,'LineWidth',0.5);
line(xlim(),[0 0],'Color',C1,'LineWidth',0.5);
xlabel('$\Delta q_{s2}$','Interpreter','latex','FontSize',14);
ylabel('$\Delta q_{t2}$','Interpreter','latex','FontSize',14);
% title('(b)','Units','normalized','Position',[0.5 -0.25]);

subplot(nr,nc,4);hold on; %title('(d)'); % axis equal
plot(qSeg_perierr_deg(id_Norm,8),qSeg_perierr_deg(id_Norm,9),'*','Color',C2,'LineWidth',mkrWid);
plot(qSeg_perierr_deg(id_Slow,8),qSeg_perierr_deg(id_Slow,9),'*','Color',C3,'LineWidth',mkrWid);
plot(qSeg_perierr_deg(id_Fast,8),qSeg_perierr_deg(id_Fast,9),'*','Color',C4,'LineWidth',mkrWid);
plot(qSeg_perierr_deg(id_Rmin,8),qSeg_perierr_deg(id_Rmin,9),'*','Color',C5,'LineWidth',mkrWid);
plot(qSeg_perierr_deg(id_Lmin,8),qSeg_perierr_deg(id_Lmin,9),'*','Color',C6,'LineWidth',mkrWid);
% plot(qSeg_perierr_deg(id_normal,8),qSeg_perierr_deg(id_normal,9),'b*');
% plot(qSeg_perierr_deg(id_abnormal,8),qSeg_perierr_deg(id_abnormal,9),'r*');
plot(x2020,RFooSfit.f.p1*x2020+RFooSfit.f.p2,'k','HandleVisibility','off');
plot(x2020,RFooSfit.fpr(:,1),'k--','HandleVisibility','off');
plot(x2020,RFooSfit.fpr(:,2),'k--','HandleVisibility','off');
line([0 0],ylim(),'Color',C1,'LineWidth',0.5,'HandleVisibility','off');
line(xlim(),[0 0],'Color',C1,'LineWidth',0.5,'HandleVisibility','off');
xlabel('$\Delta q_{s2}$','Interpreter','latex','FontSize',14);
ylabel('$\Delta q_{f2}$','Interpreter','latex','FontSize',14);
legend({'Norm','Slow','Fast','Rmin','Lmin'},'Box','off','FontSize',9)
% title('(d)','Units','normalized','Position',[0.5 -0.25]);

subplot(nr,nc,3);hold on; %title('(c)'); % axis equal
plot(qSeg_perierr_deg(id_Norm,5),qSeg_perierr_deg(id_Norm,6),'*','Color',C2,'LineWidth',mkrWid,'HandleVisibility','off');
plot(qSeg_perierr_deg(id_Slow,5),qSeg_perierr_deg(id_Slow,6),'*','Color',C3,'LineWidth',mkrWid,'HandleVisibility','off');
plot(qSeg_perierr_deg(id_Fast,5),qSeg_perierr_deg(id_Fast,6),'*','Color',C4,'LineWidth',mkrWid,'HandleVisibility','off');
plot(qSeg_perierr_deg(id_Rmin,5),qSeg_perierr_deg(id_Rmin,6),'*','Color',C5,'LineWidth',mkrWid,'HandleVisibility','off');
plot(qSeg_perierr_deg(id_Lmin,5),qSeg_perierr_deg(id_Lmin,6),'*','Color',C6,'LineWidth',mkrWid,'HandleVisibility','off');
% plot(qSeg_perierr_deg(id_normal,5),qSeg_perierr_deg(id_normal,6),'b*');
% plot(qSeg_perierr_deg(id_abnormal,5),qSeg_perierr_deg(id_abnormal,6),'r*');
plot(x2020,RFooEfit.f.p1*x2020+RFooEfit.f.p2,'k');
plot(x2020,RFooEfit.fpr(:,1),'k--');
plot(x2020,RFooEfit.fpr(:,2),'k--');
line([0 0],ylim(),'Color',C1,'LineWidth',0.5);
line(xlim(),[0 0],'Color',C1,'LineWidth',0.5);
xlabel('$\Delta q_{s1}$','Interpreter','latex','FontSize',14);
ylabel('$\Delta q_{f1}$','Interpreter','latex','FontSize',14);
% title('(c)','Units','normalized','Position',[0.5 -0.25]);
legend({'Linear fit','95% boundary'},'Box','on','FontSize',9,'Color','w','EdgeColor','w')



if nr*nc>4
subplot(nr,nc,5);hold on;  title('RybEfit');
% axis equal
plot(qSeg_perierr_deg(id_Norm,5),qSeg_perierr_deg(id_Norm,2),'b*');
plot(qSeg_perierr_deg(id_abnormal,5),qSeg_perierr_deg(id_abnormal,2),'r*');
plot(x2020,RybEfit.f.p1*x2020+RybEfit.f.p2,'k');
plot(x2020,RybEfit.fpr(:,1),'k--');
plot(x2020,RybEfit.fpr(:,2),'k--');

subplot(nr,nc,6);hold on;  title('RqbEfit');
axis equal
plot(qSeg_perierr_deg(id_Norm,5),qSeg_perierr_deg(id_Norm,3),'b*');
plot(qSeg_perierr_deg(id_abnormal,5),qSeg_perierr_deg(id_abnormal,3),'r*');
plot(x2020,RqbEfit.f.p1*x2020+RqbEfit.f.p2,'k');
plot(x2020,RqbEfit.fpr(:,1),'k--');
plot(x2020,RqbEfit.fpr(:,2),'k--');

%

subplot(nr,nc,7);hold on;  title('LybEfit');
% axis equal
plot(qSeg_perierr_deg(id_Norm,8),qSeg_perierr_deg(id_Norm,2),'b*');
plot(qSeg_perierr_deg(id_abnormal,8),qSeg_perierr_deg(id_abnormal,2),'r*');
plot(x2020,LybEfit.f.p1*x2020+LybEfit.f.p2,'k');
plot(x2020,LybEfit.fpr(:,1),'k--');
plot(x2020,LybEfit.fpr(:,2),'k--');

subplot(nr,nc,8);hold on;  title('LqbEfit');
axis equal
plot(qSeg_perierr_deg(id_Norm,8),qSeg_perierr_deg(id_Norm,3),'b*');
plot(qSeg_perierr_deg(id_abnormal,8),qSeg_perierr_deg(id_abnormal,3),'r*');
plot(x2020,LqbEfit.f.p1*x2020+LqbEfit.f.p2,'k');
plot(x2020,LqbEfit.fpr(:,1),'k--');
plot(x2020,LqbEfit.fpr(:,2),'k--');
end

return;
%%
save(['PostRes',char(datetime('now','format','yyyyMMdd_HHmmss')),'.mat'],'PostRes');




























