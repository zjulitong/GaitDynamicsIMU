% This file is process the data from motion capture data

clear; clc;
set(0,'DefaultFigureVisible', 'off')
set(0,'DefaultFigureVisible', 'on') % !!! comment this,  if run all the trials.

%% Constants
aux.FolderName='../EXPData/';
GaitTypes={'Norm','Slow','Fast','Rmin','Lmin'};

Ages=[29;23;28;23;25;18;19;21;23;26;19;25;];% [year]
BHeights=[182;182;179;177;160;170;161;157;158;180;173;170;];%[cm]
BMasses=[66.0;73.5;78.3;76.8;46.8;59.0;54.5;58.7;57.3;78.0;71.5;68.9;];% [kg]
Sexes=[0;0;0;0;1;0;1;1;1;0;0;0;];% [0=Male,1=Female]
Thigh_Lens=[43.0;43.0;44.8;40.3;36.8;40.0;38.5;34.3;36.3;43.0;41.0;38.5;];
Shank_Lens=[39.8;41.5;38.3;39.3;34.8;39.3;36.3;33.8;34.0;43.3;39.0;36.3;];
FootH_Lens=[9.5;10.0;9.3;9.3;9.5;10.0;9.0;8.5;8.5;10.5;9.3;9.3;];
FootL_HEEL=[7.0;5.0;7.0;7.5;6.0;6.0;5.0;5.0;6.0;9.0;7.5;6.5];
FootL_TOE=[16.5;17.3;16.5;15.3;14.5;17.0;15.0;15.3;14.0;16.5;16.8;16.0];

disp(['Age_mnsd=',num2str(mean(Ages),'%0.1f'),' +- ',num2str(std(Ages),'%0.1f')]);
disp(['Mass_mnsd=',num2str(mean(BMasses),'%0.1f'),' +- ',num2str(std(BMasses),'%0.1f')]);
disp(['Height_mnsd=',num2str(mean(BHeights/100),'%0.2f'),' +- ',num2str(std(BHeights/100),'%0.2f')]);
disp(['F_Num=',num2str(sum(Sexes)),' ; M_Num= ',num2str(length(Sexes)-sum(Sexes))]);

CalTrialNums={[1,2, 1], [3,4, 1], [1,2, 1],[3,4, 1],[1,2, 1];... 
              [1,2, 1], [3,4, 1], [1,2, 1],[3,4, 1],[1,2, 1];... 
              [1,2, 1], [3,4, 1], [1,2, 1],[3,4, 1],[3,4, 1];... 
              [1,2, 1], [3,4, 1], [1,2, 1],[3,4, 1],[3,4, 1];... 
              [1,2, 1], [3,4, 2], [1,2, 1],[3,4, 1],[3,4, 1];... 
              [1,2, 1], [3,4, 1], [1,2, 1],[3,4, 1],[3,4, 1];... 
              [1,2, 1], [3,4, 1], [1,2, 1],[3,4,21],[3,4, 1];... 
              [1,2, 1], [3,4, 1], [1,2, 1],[3,4, 1],[3,4, 1];... 
              [1,2, 1], [3,4, 1], [1,2, 1],[3,4, 1],[3,4, 1];... 
              [1,2, 1], [3,4, 1], [1,2, 1],[3,4, 1],[3,4, 2];... 
              [1,2, 1], [3,4,21], [1,2, 1],[3,4, 1],[3,4, 2];... 
              [1,2, 1], [3,4, 1], [1,2, 1],[3,4, 1],[3,4, 2];... 
              }; % calibration of [FL,FR,subj].
           
ViconTrialNums={[2,4:8],   [2:7],   [2:7], [2:7],[2:7] ;...
                [2:4,6:8],  [2:7],  [2:7],  [2:7],[2,4:6,9:10];...
                [3:8], [2:7], [2:7],[2,4:6, 8:9],[4:7,9:10];...
                [4:9], [2:7], [2:5,7:8],[2:7],[2:7];... 
                [2:5,7:8], [3,5:7,9:10], [2:7],[2:6,8],[2:6,8];... 
                [2,5:9], [2:7], [3:8],[2:5,7:8],[3:8];... 
                [2:7], [2:7],[2:3,5:8], [22:27],[2,5:9];...
                [2:4,6:7,9], [2:7], [2:7],[2:4,6:8],[2:4,6:8];... 
                [2:7], [2:7], [2:7],[2:7],[2:7];... 
                [2:7], [2:7], [2:7],[2:7],[3:8];... 
                [3:6,9:10], [22:27], [3:5,7:9],[2:7],[4:5,7:10];... 
                [2:4,6:8], [2:7], [3:7,9],[4:5,7:9,12],[3:8];...
                };

ImuTrialNums={[2,4:8],   [2:7],   [2:7], [2:7],[2:7];... 
              [2:4,6:8],  [2:7],  [2:7],  [2:7],[2,4:6,9:10];...
              [3:8], [2:7], [2:7],[2,4:6, 8:9],[4:7,9:10];... 
              [4:9], [2:7], [2:5,7:8],[2:7],[2:7];... 
              [2:5,7:8], [2,4:6,8:9], [2:7],[2:6,8],[2:6,8];... 
              [2,4:8], [2:7], [3:8],[2:5,7:8],[3:8];...
              [2:7], [2:7],[2:3,5:8], [2:7],[2,5:9];...
              [2:4,6:7,9], [2:7], [2:7],[2:4,6:8],[2:4,6:8];... 
              [2:7], [2:7], [2:7],[2:7],[2:7];... 
              [2:7], [2:7], [2:7],[2:7],[3:8];... 
              [3:6,9:10], [2:7], [3:5,7:9],[2:7],[4:5,7:10];...
              [2:4,6:8], [2:7], [3:7,9],[2:3,5:7,9],[3:8];... 
              }; 

%% Flags for plot
plot0= 0 ; % !!! set this to ZERO, if run all the trials.
aux.plotFP=1*plot0;
aux.plotSta=1*plot0;
aux.plotEvent=1*plot0+0;
aux.plotGRFCOP=1*plot0+0;
aux.plotStick=1*plot0+0;
aux.plotIKID=1*plot0+0;
aux.plotJF=1*plot0;
aux.plotSeg=1*plot0;
aux.plotIMU=1*plot0+1;  % !!! set this to ZERO, if run all the trials.

idx=0; % for collecting IMU error
% Paper Fig IMU comparsion: use Subj=1 ,type 1,  trii=1
SubjectNum=   0;

%% ************************** Subject Loop
for SubjectNum= 1 %  : 12
    
    aux.SubjectNum=SubjectNum;
    aux.SubjectName=['S',num2str(SubjectNum,'%02d')];
    
    aux.BM=BMasses(aux.SubjectNum);
    aux.BH=BHeights(aux.SubjectNum);
    aux.Age=Ages(aux.SubjectNum);
    aux.Sex=Sexes(aux.SubjectNum);
    aux.Thigh_Len=Thigh_Lens(aux.SubjectNum);
    aux.Shank_Len=Shank_Lens(aux.SubjectNum);
    aux.FootH_Len=FootH_Lens(aux.SubjectNum);
    aux.FootL_HEEL=FootL_HEEL(aux.SubjectNum);
    aux.FootL_TOE=FootL_TOE(aux.SubjectNum);
    
    aux.g =9.8; %[m/s^2]
    
    GaitTypei=1;
    if exist('Subj','var')
        clear Subj;
    end
    %% ************************** GaitType Loop
    for GaitTypei= 1 % :5
        
    %% Location of GCS and two force plates
    aux.Tmat_0  =Tmaker([0  ;0   ;0],[ 1;0;0],[0;1;0],[0;0; 1]);
    aux.Tmat_LFP=Tmaker([0.2;0.3 ;0],[-1;0;0],[0;1;0],[0;0;-1]);
    if SubjectNum<3
        aux.Tmat_RFP=Tmaker([0.603;1.02;0],[-1;0;0],[0;1;0],[0;0;-1]);
    else
        aux.Tmat_RFP=Tmaker([0.6;1.02;0],[-1;0;0],[0;1;0],[0;0;-1]);
    end
    %% Get the true location of force plates
    aux.CalTrialNums=CalTrialNums{SubjectNum,GaitTypei};
    aux.FPLeft_FileName =[aux.FolderName,aux.SubjectName,'/Traj2End/','Cal 1',num2str(aux.CalTrialNums(1),'%02d'),'.csv'];
    aux.FPRight_FileName=[aux.FolderName,aux.SubjectName,'/Traj2End/','Cal 1',num2str(aux.CalTrialNums(2),'%02d'),'.csv'];
    
    aux=FPPosiCal(aux);
    %% Static Gait Model Calibration
    GaitType=GaitTypes{GaitTypei};
    ViconFilePrefix=[GaitType,' 1'];
    aux.StaTraj_FileName=[aux.FolderName,aux.SubjectName,'/Traj2End/',ViconFilePrefix,num2str(aux.CalTrialNums(3),'%02d'),'.csv'];
    Res_Sta=StaTrajProcessAve(aux.StaTraj_FileName,aux);
%     return;
    
    %% Dynamic Trial---------------
    aux.ViconTrialNums=ViconTrialNums{SubjectNum,GaitTypei};
    TrialCounts= 6 ;
    TrialNum=  1;
    %% ************************** Trial Loop
    for TrialNum= 1 % :  TrialCounts
        aux.TrialNum=TrialNum;
        aux.TrialName=[ViconFilePrefix,num2str(aux.ViconTrialNums(TrialNum),'%02d'),'.csv'];
        aux.DynTraj_FileName  =[aux.FolderName,aux.SubjectName,'/Traj2End/',aux.TrialName];
        aux.DynDevice_FileName=[aux.FolderName,aux.SubjectName,'/Device/',aux.TrialName];
        
        if exist('Res','var')
            clear Res;
        end
        %%
        [Trajnum,~,~]=xlsread(aux.DynTraj_FileName);
        Res.ViconLen=Trajnum(end,1);
        
        %% IMU
        IMUPrefix=['../../ShankMotionCal/RV1/ExtractData/',aux.SubjectName,'/',GaitType]; % change this 

        aux.ImuTrialNums=ImuTrialNums{SubjectNum,GaitTypei};
        aux.IMU_FileName  =[IMUPrefix,'_',num2str(aux.ImuTrialNums(TrialNum),'%02d'),'.mat'];
        
        IMULei=load(aux.IMU_FileName);
        Res.IMULen=size(IMULei.TrialLeft,1);
        %%
        Res.LenError=Res.ViconLen-Res.IMULen;
        if abs(Res.LenError)>0
            disp(['*************************************************************** Length error = ',num2str(Res.LenError),' *******']);
        end
        
        idx=idx+1;
        LenError(idx,:)=[SubjectNum,GaitTypei,TrialNum,...
            aux.ViconTrialNums(TrialNum),aux.ImuTrialNums(TrialNum),Res.ViconLen,Res.IMULen,Res.LenError];
        %%
%         disp(['SubjectNum = ',num2str(SubjectNum),' ; GaitType = ',GaitType,'; TrialNum = ',num2str(TrialNum),' -----done']);
%         continue;
          
        %%
        [Res.Events,aux]=GaitEvent(aux);
        
        %% IK
        [Res.Traj]=DynTrajProcess(Res,aux);
        
        %% GRF GRM COP
        [Res.GRFMCOP]=DynGRFMCOPCal(Res,aux);
        
        %% ID
        [Res.JF2D,Res.JM2D,Res.JP2D]=InverseDynamic(Res,aux);
        
        %%
        aux.IMUFrameRate=100;
        
        cycleR=IMULei.gaitCycle_Right;
        cycleL=IMULei.gaitCycle_Left;
        
        [minR,idR]=min(abs(cycleR(:,1)-Res.Traj.tid_mkr(1)));
        [minL,idL]=min(abs(cycleL(:,1)-Res.Traj.tid_mkr(end)));
        
        if exist('IMURes','var')
            clear IMURes;
        end
        IMURes.TrialLen=length(IMULei.TrialLeft(:,1));
        IMURes.fr_beg=cycleR(idR,1);
        IMURes.fr_end=cycleL(idL,1);
        IMURes.fr_DS=cycleL(idL,3);
        IMURes.fr_cnt=IMURes.fr_end-IMURes.fr_beg+1;  % the frame count
        IMURes.tid_IMU=(IMURes.fr_beg:IMURes.fr_end).';
        
        IMURes.Tstep=(IMURes.fr_end-IMURes.fr_beg)/aux.IMUFrameRate; % time length of this step
        IMURes.TDS=(IMURes.fr_DS-IMURes.fr_beg)/aux.IMUFrameRate; % time length of this step
        IMURes.t_IMU=(0:IMURes.fr_cnt-1).'/aux.IMUFrameRate; % time grids of this step
        
        rangeL=(cycleL(idL,2)-IMURes.fr_cnt+1):cycleL(idL,2); % range of left leg data
        rangeR=1:IMURes.fr_cnt;                         % range of right leg data
        
        IMURes.Err_event=[cycleR(idR,1)-Res.Traj.tid_mkr(1), cycleL(idL,1)-Res.Traj.tid_mkr(end)]/aux.IMUFrameRate;
        if max(abs(IMURes.Err_event))>0.05
            disp(['Large error in events, possible mismatch between vicon and IMU data']);
            disp(['Err_HS-Event: ',num2str(IMURes.Err_event),' sec in ',aux.IMU_FileName]);
        end
        %%
        IMURes.pLAJCy=IMULei.HorizontalDisplacementCycle_Left(idL,rangeL).';
        IMURes.pLAJCz=IMULei.VerticalDisplacementCycle_Left(idL,rangeL).';
        IMURes.qLSha=IMULei.RollCycle_Left(idL,rangeL).';
        
        IMURes.pRAJCy=IMULei.HorizontalDisplacementCycle_Right(idR+1,rangeR).';
        IMURes.pRAJCz=IMULei.VerticalDisplacementCycle_Right(idR+1,rangeR).';
        IMURes.qRSha=IMULei.RollCycle_Right(idR+1,rangeR).';
        
        deltaXR=mean(IMURes.pRAJCy(:))-mean(Res.Traj.Traj_RAJC(:,2));
        deltaXL=mean(IMURes.pLAJCy(:))-mean(Res.Traj.Traj_LAJC(:,2));
        
        deltaY=IMURes.pRAJCz(round(end/2))-Res.Traj.Traj_RAJC(round(end/2),3);
        
        C1= [110 110 110]/255;
        C2= [255 48 48]/255;
        
        AngP_RSha_cal=Res.Traj.AngP_RSha-Res_Sta.AngP_RSha_Sta;
        AngP_LSha_cal=Res.Traj.AngP_LSha-Res_Sta.AngP_LSha_Sta;
        if aux.plotIMU==1
            %%
            t_max=max(IMURes.Tstep,Res.Traj.Time_step)+0.1;
            t_offset=min(Res.Traj.tid_mkr(1),IMURes.tid_IMU(1))/100;
            x_offset=Res.Traj.Traj_RAJC(round(end/2),2);
            y_offset=Res.Traj.Traj_RAJC(round(end/2),3);
            figure;hold on;
            set(gcf,'Position',[100 100 600 250],'defaultLineLineWidth',2);
            subplot(1,3,1);hold on;%title('(a)','Units','normalized','Position',[0.5 -0.45])
            plot(Res.Traj.tid_mkr/100-t_offset,Res.Traj.Traj_RAJC(:,2)-x_offset,'','Color',C1,'LineWidth',2.5);
            plot(Res.Traj.tid_mkr/100-t_offset,Res.Traj.Traj_LAJC(:,2)-x_offset,'-.','Color',C1,'LineWidth',2.5);
            plot(IMURes.tid_IMU/100-t_offset,IMURes.pRAJCy-deltaXR-x_offset,'','Color',C2);
            plot(IMURes.tid_IMU/100-t_offset,IMURes.pLAJCy-deltaXL-x_offset,'-.','Color',C2);
            xlabel('Time (s)'); %ylabel('Position (m)');
            xlim([0 t_max]);ylim([-0.8 0.8]);
            ylabel('$p_{ax}$ (m)','interpreter','latex','FontSize',14);
            subplot(1,3,2);hold on;%title('(b)','Units','normalized','Position',[0.5 -0.25])
            plot(Res.Traj.tid_mkr/100-t_offset,Res.Traj.Traj_RAJC(:,3)-y_offset,'','Color',C1,'LineWidth',2.5);
            plot(Res.Traj.tid_mkr/100-t_offset,Res.Traj.Traj_LAJC(:,3)-y_offset,'-.','Color',C1,'LineWidth',2.5);
            plot(IMURes.tid_IMU/100-t_offset,IMURes.pRAJCz-deltaY-y_offset,'','Color',C2);
            plot(IMURes.tid_IMU/100-t_offset,IMURes.pLAJCz-deltaY-y_offset,'-.','Color',C2);
            xlabel('Time (s)'); %ylabel('Position (m)');
            xlim([0 t_max]);ylim([-0.02 0.25]);
            ylabel('$p_{ay}$ (m)','interpreter','latex','FontSize',14);
            legend({'MCS-R','MCS-L','IMU-R','IMU-L'},'FontSize',8,'Box','off','Location','northeast')
            
            subplot(1,3,3);hold on;%title('(c)','Units','normalized','Position',[0.5 -0.25])
            plot(Res.Traj.tid_mkr/100-t_offset,rad2deg(AngP_RSha_cal),'','Color',C1,'LineWidth',2.5);
            plot(Res.Traj.tid_mkr/100-t_offset,rad2deg(AngP_LSha_cal),'-.','Color',C1,'LineWidth',2.5);
            plot(IMURes.tid_IMU/100-t_offset,rad2deg(IMURes.qRSha),'','Color',C2);
            plot(IMURes.tid_IMU/100-t_offset,rad2deg(IMURes.qLSha),'-.','Color',C2);
            xlabel('Time (s)'); ylabel('Shank Angle (deg)');
            xlim([0 t_max]);
            ylabel('$q_{s} (^{\circ})$','interpreter','latex','FontSize',14);
            
        end
        
        IMURes.Err_Shank=[ interp1(IMURes.tid_IMU,IMURes.pRAJCy,Res.Traj.tid_mkr,'pchip')-deltaXR-Res.Traj.Traj_RAJC(:,2),...
            interp1(IMURes.tid_IMU,IMURes.pRAJCz,Res.Traj.tid_mkr,'pchip')-deltaY-Res.Traj.Traj_RAJC(:,3),...
            rad2deg(interp1(IMURes.tid_IMU,IMURes.qRSha ,Res.Traj.tid_mkr,'pchip')-AngP_RSha_cal),...
            interp1(IMURes.tid_IMU,IMURes.pLAJCy,Res.Traj.tid_mkr,'pchip')-deltaXL-Res.Traj.Traj_LAJC(:,2),...
            interp1(IMURes.tid_IMU,IMURes.pLAJCz,Res.Traj.tid_mkr,'pchip')-deltaY-Res.Traj.Traj_LAJC(:,3),...
            rad2deg(interp1(IMURes.tid_IMU,IMURes.qLSha ,Res.Traj.tid_mkr,'pchip')-AngP_LSha_cal)];
        IMURes.Err_Shank_rms=rms(IMURes.Err_Shank);
        IMURes.Err_Shank_totrms=rms([IMURes.Err_Shank(:,1:3);IMURes.Err_Shank(:,4:6)]);
        IMURes.Err_Shank_mn=mean(IMURes.Err_Shank);
        IMURes.Err_Shank_totmn=mean([IMURes.Err_Shank(:,1:3);IMURes.Err_Shank(:,4:6)]);
        
        Res.IMURes=IMURes;
        
        ErrShank_rms(idx,:)=[SubjectNum,TrialNum,IMURes.Err_Shank_rms,IMURes.Err_Shank_totrms];
        ErrShank_mn(idx,:)=[SubjectNum,TrialNum,IMURes.Err_Shank_mn,IMURes.Err_Shank_totmn];
        TimingErr(idx,:)=[SubjectNum,GaitTypei,TrialNum,...
            aux.ViconTrialNums(TrialNum),aux.ImuTrialNums(TrialNum),Res.ViconLen,Res.IMULen,Res.LenError,IMURes.Err_event];
        
        %%
        if aux.plotIKID==1
            %%
            figure; hold on;
            set(gcf,'Position',[100 100 800 600],'defaultLineLineWidth',1.5);
            nr=4;nc=3;
            subplot(nr,nc,1);hold on;title('Angle')
            plot(Res.Traj.t_mkr,rad2deg(Res.Traj.AngP_RHip),'b');
            plot(Res.Traj.t_mkr+Res.Traj.Time_step,rad2deg(Res.Traj.AngP_LHip),'g');
            plot(Res.Traj.t_mkr,rad2deg(Res.Traj.AngP_RHip-Res_Sta.AngP_RHip_Sta),'b--');
            plot(Res.Traj.t_mkr+Res.Traj.Time_step,rad2deg(Res.Traj.AngP_LHip-Res_Sta.AngP_LHip_Sta),'g--');
            grid minor
            subplot(nr,nc,2);hold on;title('Angle')
            plot(Res.Traj.t_mkr,rad2deg(Res.Traj.AngP_RKne),'b');
            plot(Res.Traj.t_mkr+Res.Traj.Time_step,rad2deg(Res.Traj.AngP_LKne),'g');
            plot(Res.Traj.t_mkr,rad2deg(Res.Traj.AngP_RKne-Res_Sta.AngP_RKne_Sta),'b--');
            plot(Res.Traj.t_mkr+Res.Traj.Time_step,rad2deg(Res.Traj.AngP_LKne-Res_Sta.AngP_LKne_Sta),'g--');
            grid minor
            subplot(nr,nc,3);hold on;title('Angle')
            plot(Res.Traj.t_mkr,rad2deg(Res.Traj.AngP_RAnk),'b');
            plot(Res.Traj.t_mkr+Res.Traj.Time_step,rad2deg(Res.Traj.AngP_LAnk),'g');
            plot(Res.Traj.t_mkr,rad2deg(Res.Traj.AngP_RAnk-Res_Sta.AngP_RAnk_Sta),'b--');
            plot(Res.Traj.t_mkr+Res.Traj.Time_step,rad2deg(Res.Traj.AngP_LAnk-Res_Sta.AngP_LAnk_Sta),'g--');
            grid minor
            
            subplot(nr,nc,1+3*1);hold on;title('AngleV')
            plot(Res.Traj.t_mkr,Res.Traj.AngV_RHip,'b');
            plot(Res.Traj.t_mkr+Res.Traj.Time_step,Res.Traj.AngV_LHip,'g');
            grid minor
            subplot(nr,nc,2+3*1);hold on;title('AngleV')
            plot(Res.Traj.t_mkr,Res.Traj.AngV_RKne,'b');
            plot(Res.Traj.t_mkr+Res.Traj.Time_step,Res.Traj.AngV_LKne,'g');
            grid minor
            subplot(nr,nc,3+3*1);hold on;title('AngleV')
            plot(Res.Traj.t_mkr,Res.Traj.AngV_RAnk,'b');
            plot(Res.Traj.t_mkr+Res.Traj.Time_step,Res.Traj.AngV_LAnk,'g');
            grid minor
            
            subplot(nr,nc,1+3*2);hold on;title('Moment')
            plot(Res.Traj.t_mkr,Res.JM2D.JM2D_RHip,'b');
            plot(Res.Traj.t_mkr+Res.Traj.Time_step,Res.JM2D.JM2D_LHip,'g');
            grid minor
            subplot(nr,nc,2+3*2);hold on;title('Moment')
            plot(Res.Traj.t_mkr,Res.JM2D.JM2D_RKne,'b');
            plot(Res.Traj.t_mkr+Res.Traj.Time_step,Res.JM2D.JM2D_LKne,'g');
            grid minor
            subplot(nr,nc,3+3*2);hold on;title('Moment')
            plot(Res.Traj.t_mkr,Res.JM2D.JM2D_RAnk,'b');
            plot(Res.Traj.t_mkr+Res.Traj.Time_step,Res.JM2D.JM2D_LAnk,'g');
            grid minor
            
            subplot(nr,nc,1+3*3);hold on;title('Power')
            plot(Res.Traj.t_mkr,Res.JP2D.JP2D_RHip,'b');
            plot(Res.Traj.t_mkr+Res.Traj.Time_step,Res.JP2D.JP2D_LHip,'g');
            grid minor
            subplot(nr,nc,2+3*3);hold on;title('Power')
            plot(Res.Traj.t_mkr,Res.JP2D.JP2D_RKne,'b');
            plot(Res.Traj.t_mkr+Res.Traj.Time_step,Res.JP2D.JP2D_LKne,'g');
            grid minor
            subplot(nr,nc,3+3*3);hold on;title('Power')
            plot(Res.Traj.t_mkr,Res.JP2D.JP2D_RAnk,'b');
            plot(Res.Traj.t_mkr+Res.Traj.Time_step,Res.JP2D.JP2D_LAnk,'g');
            grid minor
            suptitle([aux.TrialName])
        end
        %% ---------------------------------
        if aux.plotJF==1
            %%
            figure; hold on;
            set(gcf,'Position',[100 100 800 600],'defaultLineLineWidth',1.5);
            nr=2;nc=3;
            subplot(nr,nc,1+3*0);hold on;title('Joint F')
            plot(Res.Traj.t_mkr,Res.JF2D.JF2D_RHip,'b');
            plot(Res.Traj.t_mkr+Res.Traj.Time_step,Res.JF2D.JF2D_LHip,'g');
            subplot(nr,nc,2+3*0);hold on;title('Joint F')
            plot(Res.Traj.t_mkr,Res.JF2D.JF2D_RKne,'b');
            plot(Res.Traj.t_mkr+Res.Traj.Time_step,Res.JF2D.JF2D_LKne,'g');
            subplot(nr,nc,3+3*0);hold on;title('Joint F')
            plot(Res.Traj.t_mkr,Res.JF2D.JF2D_RAnk,'b');
            plot(Res.Traj.t_mkr+Res.Traj.Time_step,Res.JF2D.JF2D_LAnk,'g');
            
            subplot(nr,nc,1+3*1);hold on;title('GRF')
            plot(Res.Traj.t_mkr,Res.GRFMCOP.F2_g(:,2),'b');
            plot(Res.Traj.t_mkr+Res.Traj.Time_step,Res.GRFMCOP.F1_g(:,2),'g');
            
            subplot(nr,nc,2+3*1);hold on;title('GRF')
            plot(Res.Traj.t_mkr,Res.GRFMCOP.F2_g(:,3),'b');
            plot(Res.Traj.t_mkr+Res.Traj.Time_step,Res.GRFMCOP.F1_g(:,3),'g');
        end
        %%
        if aux.plotSeg==1
            %%
            figure; hold on;
            set(gcf,'Position',[100 100 800 600],'defaultLineLineWidth',1.5);
            nr=3;nc=3;
            subplot(nr,nc,1);hold on;title('Angle')
            plot(Res.Traj.t_mkr,rad2deg(Res.Traj.AngP_RThi),'b');
            plot(Res.Traj.t_mkr+Res.Traj.Time_step,rad2deg(Res.Traj.AngP_LThi),'g');
            subplot(nr,nc,2);hold on;title('Angle')
            plot(Res.Traj.t_mkr,rad2deg(Res.Traj.AngP_RSha),'b');
            plot(Res.Traj.t_mkr+Res.Traj.Time_step,rad2deg(Res.Traj.AngP_LSha),'g');
            subplot(nr,nc,3);hold on;title('Angle')
            plot(Res.Traj.t_mkr,rad2deg(Res.Traj.AngP_RFoo),'b');
            plot(Res.Traj.t_mkr+Res.Traj.Time_step,rad2deg(Res.Traj.AngP_LFoo),'g');
            
            subplot(nr,nc,1+3);hold on;title('AngleV')
            plot(Res.Traj.t_mkr,rad2deg(Res.Traj.AngV_RThi),'b');
            plot(Res.Traj.t_mkr+Res.Traj.Time_step,rad2deg(Res.Traj.AngV_LThi),'g');
            subplot(nr,nc,2+3);hold on;title('AngleV')
            plot(Res.Traj.t_mkr,rad2deg(Res.Traj.AngV_RSha),'b');
            plot(Res.Traj.t_mkr+Res.Traj.Time_step,rad2deg(Res.Traj.AngV_LSha),'g');
            subplot(nr,nc,3+3);hold on;title('AngleV')
            plot(Res.Traj.t_mkr,rad2deg(Res.Traj.AngV_RFoo),'b');
            plot(Res.Traj.t_mkr+Res.Traj.Time_step,rad2deg(Res.Traj.AngV_LFoo),'g');
            
            
            subplot(nr,nc,1+3*2);hold on;title('AngleA')
            plot(Res.Traj.t_mkr,rad2deg(Res.Traj.AngA_RThi),'b');
            plot(Res.Traj.t_mkr+Res.Traj.Time_step,rad2deg(Res.Traj.AngA_LThi),'g');
            subplot(nr,nc,2+3*2);hold on;title('AngleA')
            plot(Res.Traj.t_mkr,rad2deg(Res.Traj.AngA_RSha),'b');
            plot(Res.Traj.t_mkr+Res.Traj.Time_step,rad2deg(Res.Traj.AngA_LSha),'g');
            subplot(nr,nc,3+3*2);hold on;title('AngleA')
            plot(Res.Traj.t_mkr,rad2deg(Res.Traj.AngA_RFoo),'b');
            plot(Res.Traj.t_mkr+Res.Traj.Time_step,rad2deg(Res.Traj.AngA_LFoo),'g');
        end
        
        %%
        
        eval(['Subj.',GaitType,'.Trial',num2str(TrialNum),'=Res;']);
        
        disp(['SubjectNum = ',num2str(SubjectNum),' ; GaitType = ',GaitType,'; TrialNum = ',num2str(TrialNum),' -----done']);
        
    end
    eval(['Subj.',GaitType,'.Res_Sta=Res_Sta;']);
    Subj.aux=aux;
%     Subj.Res_Sta=Res_Sta;
    
    end
    eval(['IMUVICONData.Subject',num2str(aux.SubjectNum),'=Subj;']);
end
%%

ErrShank_rms_mn=mean(ErrShank_rms(:,3:end))
ErrShank_mn_mn=mean(ErrShank_mn(:,3:end))

return;
%%

save(['IMUVICONData',char(datetime('now','format','yyyyMMdd_HHmmss')),'.mat'],'IMUVICONData');

set(0,'DefaultFigureVisible', 'on')
return;



%%













