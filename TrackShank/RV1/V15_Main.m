% this is the main program to estimate gait dynamics of one step from IMU measurements
function [Result]=V15_Main(SubjectNum,TypeNum,TrialNum,savefolder,GuessFlag,ObjType)
showcount=30000;
auxdata.logiter=0;
auxdata.setting= ObjType ; %[0 simple,1=simple with periodical, 10=all settings]
auxdata.guessFlag= GuessFlag ;
auxdata.plotMain=0;

%% use this if run for a single step.
% clear;
% clc;
% close all;
% 
% SubjectNum= 1 ;
% TypeNum= 1 ;
% TrialNum= 1 ;
% savefolder='Test';
% 
% showcount= 1500;
% auxdata.logiter= 0 ; % whether log the design variable in each iteration. a LARGE file. 
% auxdata.setting= 10 ;
% auxdata.guessFlag= 10 ; % 10 seems better than 0, although longer time. 
% auxdata.plotMain=1;
% set(0,'DefaultFigureVisible', 'on')

%%
auxdata.savefolder=savefolder;
if ~exist(savefolder,'dir')
    mkdir(savefolder);
end
auxdata.objFlag='PD'; % predict
auxdata.ResultName=[auxdata.objFlag];
auxdata.t_start = datetime('now','format','yyyyMMdd_HHmmss');
auxdata.ResultFileName=[auxdata.savefolder,'/Result_',char(auxdata.t_start),'.mat'];

%% Global Variables
global test_count test_divide plot_flag;
test_count=0; % count to show temperary resutls.
test_divide=100000;
plot_flag=0;

% ---------------------------------------------------------------
%% Functions for kinematics and EOM
path(pathdef);
addpath('./TLFuns');
auxdata.EOM_Folder='./AutoGeneFuns';
auxdata.Grad_Folder=['./AutoGeneGrad_',auxdata.objFlag,'_N3'];
auxdata.Grad_Folder=['./AutoGeneGrad_',auxdata.objFlag,'_N30'];

addpath(auxdata.EOM_Folder);
addpath(auxdata.Grad_Folder);

auxdata.EOM_CONS     = load([auxdata.EOM_Folder ,'/','EOM_CONS.mat']);
auxdata.Torque_CONS  = load([auxdata.EOM_Folder ,'/','Torque_CONS.mat']);
auxdata.Grad_CONS    = load([auxdata.Grad_Folder,'/','Grad_CONS.mat']);

%-------------------------------
N_step = auxdata.Grad_CONS.N_step;
auxdata.N_step  = N_step;
auxdata.GaitTypes={'Norm','Slow','Fast','Rmin','Lmin'};

%%
load('../../EXPData_Process\RV1/IMUVICONData20201124_175000.mat'); 
load('../../EXPData_Process\RV1/PostRes20201124_175230.mat'); 

%%
eval(['Subj=IMUVICONData.Subject',num2str(SubjectNum),';']);
eval(['Gait=Subj.',auxdata.GaitTypes{TypeNum},';']);
eval(['trial=Subj.',auxdata.GaitTypes{TypeNum},'.Trial',num2str(TrialNum),';']);
auxdata.SubjectNum=SubjectNum;
auxdata.TypeNum=TypeNum;
auxdata.TrialNum=TrialNum;

BM=Subj.aux.BM;
BH=Subj.aux.BH/100;
if BH>2||BH<1.3
    disp(['wrong BH']);
    return
end

L_TH=Subj.aux.Thigh_Len/100;
L_SH=Subj.aux.Shank_Len/100;
L_AJCz=Subj.aux.FootH_Len/100;
L_Leg=L_TH+L_SH+L_AJCz;
L_FO=(Subj.aux.FootL_HEEL+Subj.aux.FootL_TOE)/100;
PosiX_HE=-Subj.aux.FootL_HEEL/100;
PosiX_TO=Subj.aux.FootL_TOE/100;

auxdata.L_TH=L_TH;
auxdata.L_SH=L_SH;
auxdata.L_FO=L_FO;
auxdata.L_AJCz=L_AJCz;
auxdata.PosiX_HE=PosiX_HE;
auxdata.PosiX_TO=PosiX_TO;
auxdata.L_Leg=L_Leg;

L_TH_vicon=(Gait.Res_Sta.Len_RThi+Gait.Res_Sta.Len_LThi)/2;
L_SH_vicon=(Gait.Res_Sta.Len_RSha+Gait.Res_Sta.Len_LSha)/2;
L_FO_vicon=(Gait.Res_Sta.Len_RFoo+Gait.Res_Sta.Len_LFoo)/2;

PosiX_HE_vicon=(Gait.Res_Sta.PosiX_LHEE+Gait.Res_Sta.PosiX_RHEE)/2;
PosiX_TO_vicon=(Gait.Res_Sta.PosiX_LTOE+Gait.Res_Sta.PosiX_RTOE)/2;
PosiY_AK_vicon=(Gait.Res_Sta.PosiY_LANK+Gait.Res_Sta.PosiY_RANK)/2;

Tstep_MOC=trial.Traj.Time_step;

Tstep_IMU=trial.IMURes.Tstep;

t_mkr=trial.Traj.t_mkr;
t_mkr_nan=t_mkr*nan;
t_mkr_0  =t_mkr*0;
t_mkr_1  =t_mkr*0+1;

t_IMU=trial.IMURes.t_IMU;

g   = 9.8;

auxdata.BM    = BM;  % body mass
auxdata.BH    = BH; % body hight
auxdata.g     = g;
auxdata.BW     = BM*g;
auxdata.TSP_exp  = Tstep_IMU;

Traj=trial.Traj;
Res_Sta=Gait.Res_Sta;
GRFCOP=trial.GRFMCOP;
JM2D=trial.JM2D;
IMURes=trial.IMURes;


MOCOffsetx=Traj.Traj_RAJC(round(end/2),2);

pJoi_MOC=[Traj.Traj_R_GT(:,2),Traj.Traj_R_GT(:,3),...
    Traj.Traj_RKJC(:,2),Traj.Traj_RKJC(:,3),...
    Traj.Traj_RAJC(:,2),Traj.Traj_RAJC(:,3),...
    Traj.Traj_L_GT(:,2),Traj.Traj_L_GT(:,3),...
    Traj.Traj_LKJC(:,2),Traj.Traj_LKJC(:,3),...
    Traj.Traj_LAJC(:,2),Traj.Traj_LAJC(:,3)];
pJoi_MOC(:,1:2:end)=pJoi_MOC(:,1:2:end)-MOCOffsetx;

OPy=Traj.Traj_OPGT(:,2)-MOCOffsetx;
OPz=Traj.Traj_OPGT(:,3);

qSeg_MOC=[OPy,OPz,Traj.AngP_Pelv-Res_Sta.AngP_Pelv_Sta,...
    Traj.AngP_RThi-Res_Sta.AngP_RThi_Sta,Traj.AngP_RSha-Res_Sta.AngP_RSha_Sta,Traj.AngP_RFoo-Res_Sta.AngP_RFoo_Sta,...
    Traj.AngP_LThi-Res_Sta.AngP_LThi_Sta,Traj.AngP_LSha-Res_Sta.AngP_LSha_Sta,Traj.AngP_LFoo-Res_Sta.AngP_LFoo_Sta];

q_MOC=[OPy,OPz,Traj.AngP_Pelv-Res_Sta.AngP_Pelv_Sta,...
    Traj.AngP_RThi-Res_Sta.AngP_RThi_Sta,...
    Traj.AngP_RKne-Res_Sta.AngP_RKne_Sta,Traj.AngP_RAnk-Res_Sta.AngP_RAnk_Sta,...
    Traj.AngP_LThi-Res_Sta.AngP_LThi_Sta,...
    Traj.AngP_LKne-Res_Sta.AngP_LKne_Sta,Traj.AngP_LAnk-Res_Sta.AngP_LAnk_Sta];


qSeg_track=interp1(t_mkr,qSeg_MOC,linspace(t_mkr(1),t_mkr(end),length(t_IMU).'));
pJoi_track=interp1(t_mkr,pJoi_MOC,linspace(t_mkr(1),t_mkr(end),length(t_IMU).'));


% TrackType= 3 ;
% auxdata.TrackType=TrackType;

pJoi_track(:,5 )=IMURes.pRAJCy; % RAJCy
pJoi_track(:,6 )=IMURes.pRAJCz; % RAJCy
pJoi_track(:,11)=IMURes.pLAJCy; % LAJCy
pJoi_track(:,12)=IMURes.pLAJCz; % LAJCy
use_expOS=0;

qSeg_track(:,5)=IMURes.qRSha; %qRSha
qSeg_track(:,8)=IMURes.qLSha; %qLSha


Fg_MOC=[GRFCOP.F2_g(:,2),GRFCOP.F2_g(:,3),...
        GRFCOP.F1_g(:,2),GRFCOP.F1_g(:,3)];
TJ_MOC=[JM2D.JM2D_RHip,JM2D.JM2D_RKne,JM2D.JM2D_RAnk,...
        JM2D.JM2D_LHip,JM2D.JM2D_LKne,JM2D.JM2D_LAnk];


[~,pdotJoi_track,pddotJoi_track]=TLposi_diff(pJoi_track,100,1);
[~,qdotSeg_track,~]=TLposi_diff(qSeg_track,100,1);

[~,pdotJoi_MOC,pddotJoi_MOC]=TLposi_diff(pJoi_MOC,100,1);
[~,qdotSeg_MOC,~]=TLposi_diff(qSeg_MOC,100,1);
[~,qdot_MOC,qddot_MOC]=TLposi_diff(q_MOC,100,1);


auxdata.pAy_rg=range([IMURes.pRAJCy;IMURes.pLAJCy]);
auxdata.pAz_rg=range([IMURes.pRAJCz;IMURes.pLAJCz]);
auxdata.qS_rg=range([IMURes.qRSha;IMURes.qLSha]);
% return;
%%
expdata.t_grid_step=linspace(0,Tstep_IMU,N_step);  
expdata.Time_step=Tstep_IMU;

t_mkr_IMU=linspace(0,Tstep_IMU,length(t_mkr)).';

expdata.q_MOC        =TLinterp1_nan(t_mkr_IMU,q_MOC     ,expdata.t_grid_step,'pchip');
expdata.qdot_MOC     =TLinterp1_nan(t_mkr_IMU,qdot_MOC  ,expdata.t_grid_step,'pchip');
expdata.qddot_MOC    =TLinterp1_nan(t_mkr_IMU,qddot_MOC ,expdata.t_grid_step,'pchip');

expdata.pJoi_MOC     =TLinterp1_nan(t_mkr_IMU,pJoi_MOC     ,expdata.t_grid_step,'pchip');
expdata.pdotJoi_MOC  =TLinterp1_nan(t_mkr_IMU,pdotJoi_MOC  ,expdata.t_grid_step,'pchip');
expdata.pddotJoi_MOC =TLinterp1_nan(t_mkr_IMU,pddotJoi_MOC ,expdata.t_grid_step,'pchip');

expdata.qSeg_MOC     =TLinterp1_nan(t_mkr_IMU,qSeg_MOC    ,expdata.t_grid_step,'pchip');
expdata.qdotSeg_MOC  =TLinterp1_nan(t_mkr_IMU,qdotSeg_MOC ,expdata.t_grid_step,'pchip');

expdata.Fg_MOC       =TLinterp1_nan(t_mkr_IMU,Fg_MOC     ,expdata.t_grid_step,'pchip');
expdata.TJ_MOC       =TLinterp1_nan(t_mkr_IMU,TJ_MOC     ,expdata.t_grid_step,'pchip');


expdata.pJoi_track     =TLinterp1_nan(t_IMU,pJoi_track     ,expdata.t_grid_step,'pchip');
expdata.pdotJoi_track  =TLinterp1_nan(t_IMU,pdotJoi_track  ,expdata.t_grid_step,'pchip');
expdata.pddotJoi_track =TLinterp1_nan(t_IMU,pddotJoi_track ,expdata.t_grid_step,'pchip');

expdata.qSeg_track     =TLinterp1_nan(t_IMU,qSeg_track    ,expdata.t_grid_step,'pchip');
expdata.qdotSeg_track  =TLinterp1_nan(t_IMU,qdotSeg_track ,expdata.t_grid_step,'pchip');

%% *******Define constants, import model, get info of the model

%-------------------------------
auxdata.mb     = 0.6780*BM; % body 
auxdata.mt     = 0.1000*BM; % thigh
auxdata.ms     = 0.0465*BM; % shank
auxdata.mf     = 0.0145*BM; % shank

%----------Length
auxdata.Lb     = BH-L_Leg; % body length
auxdata.Lbm    = 0.363*auxdata.Lb; % CM length
auxdata.Lt     = L_TH; % body length
auxdata.Ltm    = 0.433*auxdata.Lt; % CM length
auxdata.Ls     = L_SH; % body length
auxdata.Lsm    = 0.433*auxdata.Ls; % CM length
auxdata.Lf     = L_FO;
% auxdata.Lfm    = 0.500*auxdata.Lf;
auxdata.LfH     = L_AJCz;

auxdata.Ib     = auxdata.mb*(0.287*auxdata.Lb)^2;
auxdata.It     = auxdata.mt*(0.323*auxdata.Lt)^2;
auxdata.Is     = auxdata.ms*(0.302*auxdata.Ls)^2;
auxdata.If     = auxdata.mf*(0.475*auxdata.Lf)^2;

N_cts=auxdata.EOM_CONS.N_cts;

Res_cts=load('../../ContactIdentify/RV1/ParaIDres20201124_060903.mat');
auxdata.cts.ctsk=Res_cts.Res_ctsk_mn*BM*9.8;
auxdata.cts.ctsc=Res_cts.Res_ctsc_mn*BM*9.8;
auxdata.cts.ctsud=Res_cts.Res_ctsud_mn;
auxdata.cts.ctsV=Res_cts.Res_ctsV_mn;
auxdata.cts.ctsR=Res_cts.Res_ctsR_mn*L_FO;

auxdata.dhex=PosiX_HE;
auxdata.dtox=PosiX_TO;
auxdata.dhey=-L_AJCz;
auxdata.dtoy=-L_AJCz;
auxdata.cts.ctsx=linspace(auxdata.dhex+auxdata.cts.ctsR(1),auxdata.dtox-auxdata.cts.ctsR(5),5);
auxdata.cts.ctsy=-L_AJCz+auxdata.cts.ctsR;

auxdata.dfmx    = 0.3*L_FO;
auxdata.dfmy    =-0.6*L_AJCz;

CON_Len=[auxdata.Lb,  auxdata.Lbm, auxdata.Lt, auxdata.Ltm, auxdata.Ls, auxdata.Lsm,...
         auxdata.dhex,auxdata.dhey,auxdata.dtox,auxdata.dtoy,auxdata.dfmx, auxdata.dfmy];

auxdata.CON_Len=CON_Len;

% return;
%%
if auxdata.plotMain==1
figure;hold on;
set(gcf,'Position',[100 100 1400 800],'defaultLineLineWidth',1.5);
for i=1:6
    subplot(4,6,i);hold on;title(['q',num2str(i)]);
    if i<3
        plot(t_mkr,q_MOC(:,i),'k-');
        plot(expdata.t_grid_step,expdata.q_MOC(:,i),'r--');
    elseif i==3
        plot(t_mkr,rad2deg(q_MOC(:,i)),'k-');
        plot(expdata.t_grid_step,rad2deg(expdata.q_MOC(:,i)),'r--');
    else
        plot(t_mkr,rad2deg(q_MOC(:,i)),'k-');
        plot(expdata.t_grid_step,rad2deg(expdata.q_MOC(:,i)),'r--');
        plot(t_mkr+Tstep_IMU,rad2deg(q_MOC(:,i+3)),'k-.');
        plot(expdata.t_grid_step+Tstep_IMU,rad2deg(expdata.q_MOC(:,i+3)),'r--');
        
    end
    
    subplot(4,6,i+6);hold on;title(['qdot',num2str(i)]);
    if i<3
        plot(t_mkr,qdot_MOC(:,i),'k-');
        plot(expdata.t_grid_step,expdata.qdot_MOC(:,i),'r--');
    elseif i==3
        plot(t_mkr,rad2deg(qdot_MOC(:,i)),'k-');
        plot(expdata.t_grid_step,rad2deg(expdata.qdot_MOC(:,i)),'r--');
    else
        plot(t_mkr,rad2deg(qdot_MOC(:,i)),'k-');
        plot(expdata.t_grid_step,rad2deg(expdata.qdot_MOC(:,i)),'r--');
        plot(t_mkr+Tstep_IMU,rad2deg(qdot_MOC(:,i+3)),'k-.');
        plot(expdata.t_grid_step+Tstep_IMU,rad2deg(expdata.qdot_MOC(:,i+3)),'r--');
    end
    subplot(4,6,i+12);hold on;title(['qddot',num2str(i)]);
    if i<3
        plot(t_mkr,qddot_MOC(:,i),'k-');
        plot(expdata.t_grid_step,expdata.qddot_MOC(:,i),'r--');
    elseif i==3
        plot(t_mkr,rad2deg(qddot_MOC(:,i)),'k-');
        plot(expdata.t_grid_step,rad2deg(expdata.qddot_MOC(:,i)),'r--');
    else
        plot(t_mkr,rad2deg(qddot_MOC(:,i)),'k-');
        plot(expdata.t_grid_step,rad2deg(expdata.qddot_MOC(:,i)),'r--');
        plot(t_mkr+Tstep_IMU,rad2deg(qddot_MOC(:,i+3)),'k-.');
        plot(expdata.t_grid_step+Tstep_IMU,rad2deg(expdata.qddot_MOC(:,i+3)),'r--');
        
    end
end

for i=1:2 % GRF
    subplot(4,6,i+18);hold on;title(['GRF',num2str(i)]);
    plot(t_mkr,Fg_MOC(:,i),'k-');
    plot(expdata.t_grid_step,expdata.Fg_MOC(:,i),'r--');
    plot(t_mkr+Tstep_IMU,Fg_MOC(:,i+2),'k-.');
    plot(expdata.t_grid_step+Tstep_IMU,expdata.Fg_MOC(:,i+2),'r--');
end

subplot(4,6,21);hold on;title(['GRF',num2str(i)]);
plot(t_mkr,Fg_MOC(:,1)+Fg_MOC(:,3),'k-');
plot(t_mkr,qddot_MOC(:,1)*BM,'r');
plot(t_mkr,Fg_MOC(:,2)+Fg_MOC(:,4),'k-');
plot(t_mkr,(qddot_MOC(:,2)+9.8)*BM,'r');
ylim([-300 1100])

for i=1:3 % TJ
    subplot(4,6,i+18+3);hold on;title(['TJ',num2str(i)]);
    plot(t_mkr,TJ_MOC(:,i),'k-');
    plot(expdata.t_grid_step,expdata.TJ_MOC(:,i),'r--');
    plot(t_mkr+Tstep_IMU,TJ_MOC(:,i+3),'k-.');
    plot(expdata.t_grid_step+Tstep_IMU,expdata.TJ_MOC(:,i+3),'r--');
end

figure;
set(gcf,'Position',[100 100 1400 600],'defaultLineLineWidth',1.5);
tit={'hx','hy','kx','ky','ax','ay'};
Segs={'xb','yb','qb','qThi','qSha','qFoo'};
wid_b=1;
nr=5;nc=6;
for i=1:6
    subplot(nr,nc,i);hold on;title([Segs{i}]);
    if i<3
        plot(t_mkr,qSeg_MOC(:,i),'k-');
        plot(expdata.t_grid_step,expdata.qSeg_MOC(:,i),'r--');
        plot(t_IMU,qSeg_track(:,i),'b--','LineWidth',wid_b);
    elseif i==3
        plot(t_mkr,rad2deg(qSeg_MOC(:,i)),'k-');
        plot(expdata.t_grid_step,rad2deg(expdata.qSeg_MOC(:,i)),'r--');
        plot(t_IMU,rad2deg(qSeg_track(:,i)),'b--','LineWidth',wid_b);
    else
        plot(t_mkr,rad2deg(qSeg_MOC(:,i)),'k-');
        plot(expdata.t_grid_step,rad2deg(expdata.qSeg_MOC(:,i)),'r--');
        plot(t_mkr+Tstep_IMU,rad2deg(qSeg_MOC(:,i+3)),'k-.');
        plot(expdata.t_grid_step+Tstep_IMU,rad2deg(expdata.qSeg_MOC(:,i+3)),'r--');
        
        plot(t_IMU,rad2deg(qSeg_track(:,i)),'b--','LineWidth',1.5);
        plot(t_IMU+Tstep_IMU,rad2deg(qSeg_track(:,i+3)),'b--','LineWidth',1.5);
    end
    subplot(nr,nc,i+6);hold on;title([Segs{i},'dot']);
    if i<3
        plot(t_mkr,qdotSeg_MOC(:,i),'b-');
        plot(expdata.t_grid_step,expdata.qdotSeg_MOC(:,i),'r--');
    plot(t_IMU,qdotSeg_track(:,i),'b--','LineWidth',wid_b);
    elseif i==3
        plot(t_mkr,rad2deg(qdotSeg_MOC(:,i)),'b-');
        plot(expdata.t_grid_step,rad2deg(expdata.qdotSeg_MOC(:,i)),'r--');
    plot(t_IMU,rad2deg(qdotSeg_track(:,i)),'b--','LineWidth',wid_b);
    else
        plot(t_mkr,rad2deg(qdotSeg_MOC(:,i)),'k-');
        plot(expdata.t_grid_step,rad2deg(expdata.qdotSeg_MOC(:,i)),'r--');
        plot(t_mkr+Tstep_IMU,rad2deg(qdotSeg_MOC(:,i+3)),'k-');
        plot(expdata.t_grid_step+Tstep_IMU,rad2deg(expdata.qdotSeg_MOC(:,i+3)),'r--');
    plot(t_IMU,rad2deg(qdotSeg_track(:,i)),'b--','LineWidth',wid_b);
    plot(t_IMU+Tstep_IMU,rad2deg(qdotSeg_track(:,i+3)),'b--','LineWidth',wid_b);
    end
    
    subplot(nr,nc,i+6*2);hold on; title(tit{i})
    plot(t_mkr,pJoi_MOC(:,i),'k-');
    plot(expdata.t_grid_step,expdata.pJoi_MOC(:,i),'r--');
    plot(t_mkr+Tstep_IMU,pJoi_MOC(:,i+6),'k-.');
    plot(expdata.t_grid_step+Tstep_IMU,expdata.pJoi_MOC(:,i+6),'r--');
    
    plot(t_IMU,pJoi_track(:,i),'b--','LineWidth',wid_b);
    plot(t_IMU+Tstep_IMU,pJoi_track(:,i+6),'b--','LineWidth',wid_b);
    
    subplot(nr,nc,i+6*3);hold on; title([tit{i},'dot'])
    plot(t_mkr,pdotJoi_MOC(:,i),'k-');
    plot(expdata.t_grid_step,expdata.pdotJoi_MOC(:,i),'r--');
    plot(t_mkr+Tstep_IMU,pdotJoi_MOC(:,i+6),'k-.');
    plot(expdata.t_grid_step+Tstep_IMU,expdata.pdotJoi_MOC(:,i+6),'r--');
    
    subplot(nr,nc,i+6*4);hold on; title([tit{i},'ddot'])
    plot(t_mkr,pddotJoi_MOC(:,i),'k-');
    plot(expdata.t_grid_step,expdata.pddotJoi_MOC(:,i),'r--');
    plot(t_mkr+Tstep_IMU,pddotJoi_MOC(:,i+6),'k-.');
    plot(expdata.t_grid_step+Tstep_IMU,expdata.pddotJoi_MOC(:,i+6),'r--');
    
end
end
% return;
%%
auxdata.expdata=expdata;

auxdata.epsl = 0.001;

TJmax=[500 500 500 ]; % hip knee ankle
auxdata.TJmax=[TJmax,TJmax];

auxdata.c_exc=[3.3,16.7]; % c1, c2 in the muscle model

auxdata.CON_Mass=[auxdata.mb,auxdata.mt,auxdata.ms,auxdata.mf,auxdata.Ib,auxdata.It,auxdata.Is,auxdata.If,auxdata.g];
auxdata.CON_Len=[auxdata.Lb, auxdata.Lbm, auxdata.Lt, auxdata.Ltm, auxdata.Ls, auxdata.Lsm,...
                 auxdata.dhex,auxdata.dhey,auxdata.dtox,auxdata.dtoy,auxdata.dfmx,auxdata.dfmy];

% return;
%% create initial guess from exp mean

len_mn=size(PostRes.q_mn,1);

q_normmat=(ones(len_mn,1)*[L_Leg,L_Leg,ones(1,7)]);

q_mn=PostRes.q_mn.*q_normmat;

qdot_mn=PostRes.qdot_mn.*q_normmat;

qddot_mn=PostRes.qddot_mn.*q_normmat;

TJ_mn=PostRes.MomJoi_mn*BM*L_Leg;

q_mn_Nstep=interp1((1:len_mn).',q_mn,linspace(1,len_mn,N_step).','pchip');
qdot_mn_Nstep=interp1((1:len_mn).',qdot_mn,linspace(1,len_mn,N_step).','pchip');
qddot_mn_Nstep=interp1((1:len_mn).',qddot_mn,linspace(1,len_mn,N_step).','pchip');
a_mn_Nstep=interp1((1:len_mn).',TJ_mn./(ones(len_mn,1)*auxdata.TJmax),linspace(1,len_mn,N_step).','pchip');
u_mn_Nstep=interp1((1:len_mn).',TJ_mn./(ones(len_mn,1)*auxdata.TJmax),linspace(1,len_mn,N_step).','pchip');


OSxmn=PostRes.pJoi_mn(1,11)-PostRes.pJoi_mn(end,5);
OSymn=min(PostRes.pJoi_mn(end,5));

Nsd= 3 ;
auxdata.Nsd=Nsd;
q_min=(PostRes.q_mn-PostRes.q_sd*Nsd).*q_normmat;
qdot_min=(PostRes.qdot_mn-PostRes.qdot_sd*Nsd).*q_normmat;
qddot_min=(PostRes.qddot_mn-PostRes.qddot_sd*Nsd).*q_normmat;
TJ_min=(PostRes.MomJoi_mn-PostRes.MomJoi_sd*Nsd)*BM*L_Leg;

q_max=(PostRes.q_mn+PostRes.q_sd*Nsd).*q_normmat;
qdot_max=(PostRes.qdot_mn+PostRes.qdot_sd*Nsd).*q_normmat;
qddot_max=(PostRes.qddot_mn+PostRes.qddot_sd*Nsd).*q_normmat;
TJ_max=(PostRes.MomJoi_mn+PostRes.MomJoi_sd*Nsd)*BM*L_Leg;

qmax_RKne=q_max(:,5);
qmax_RKne(qmax_RKne>0)=0;
qmax_LKne=q_max(:,8);
qmax_LKne(qmax_LKne>0)=0;
q_max(:,5)=qmax_RKne;
q_max(:,8)=qmax_LKne;

q_min(:,3)=deg2rad(-5);
q_max(:,3)=deg2rad(5);

q_min_Nstep=interp1((1:len_mn).',q_min,linspace(1,len_mn,N_step).','pchip');
qdot_min_Nstep=interp1((1:len_mn).',qdot_min,linspace(1,len_mn,N_step).','pchip');
qddot_min_Nstep=interp1((1:len_mn).',qddot_min,linspace(1,len_mn,N_step).','pchip');
a_min_Nstep=interp1((1:len_mn).',TJ_min./(ones(len_mn,1)*auxdata.TJmax),linspace(1,len_mn,N_step).','pchip');
u_min_Nstep=interp1((1:len_mn).',TJ_min./(ones(len_mn,1)*auxdata.TJmax),linspace(1,len_mn,N_step).','pchip');

q_max_Nstep=interp1((1:len_mn).',q_max,linspace(1,len_mn,N_step).','pchip');
qdot_max_Nstep=interp1((1:len_mn).',qdot_max,linspace(1,len_mn,N_step).','pchip');
qddot_max_Nstep=interp1((1:len_mn).',qddot_max,linspace(1,len_mn,N_step).','pchip');
a_max_Nstep=interp1((1:len_mn).',TJ_max./(ones(len_mn,1)*auxdata.TJmax),linspace(1,len_mn,N_step).','pchip');
u_max_Nstep=interp1((1:len_mn).',TJ_max./(ones(len_mn,1)*auxdata.TJmax),linspace(1,len_mn,N_step).','pchip');
if auxdata.plotMain==1
    figure;hold on;
    set(gcf,'Position',[100 100 1400 600],'defaultLineLineWidth',1.5);
    for i=1:9
        subplot(5,9,i);hold on;
        plot(q_min_Nstep(:,i),'k-.');    plot(q_max_Nstep(:,i),'k-');    plot(q_mn_Nstep(:,i),'r-');
        subplot(5,9,i+9);hold on;
        plot(qdot_min_Nstep(:,i),'k-.');    plot(qdot_max_Nstep(:,i),'k-');    plot(qdot_mn_Nstep(:,i),'r-');
        subplot(5,9,i+18);hold on;
        plot(qddot_min_Nstep(:,i),'k-.');    plot(qddot_max_Nstep(:,i),'k-');    plot(qddot_mn_Nstep(:,i),'r-');
        subplot(5,9,i+27);hold on;
        if i>3
        plot(a_min_Nstep(:,i-3),'k-.');    plot(a_max_Nstep(:,i-3),'k-');    plot(a_mn_Nstep(:,i-3),'r-');
        end
        subplot(5,9,i+36);hold on;
        if i>3
        plot(u_min_Nstep(:,i-3),'k-.');    plot(u_max_Nstep(:,i-3),'k-');    plot(u_mn_Nstep(:,i-3),'r-');
        end
    end
end
qdddot_mn=TLdiff(qddot_mn,linspace(0,Tstep_IMU,51).',0);
qddot_rg=range(qddot_mn_Nstep).';
qdddot_rg=range(qdddot_mn).';

pJoi_rg=range([PostRes.pJoi_mn]);
qSeg_rg=range([PostRes.qSeg_mn]);

%%
qShaE_diff=qSeg_track(end,5)-qSeg_track(1,5+3);
qShaS_diff=qSeg_track(end,5+3)-qSeg_track(1,5);

auxdata.RThiE=predint(PostRes.RThiEfit.f,qShaE_diff);
auxdata.RFooE=predint(PostRes.RFooEfit.f,qShaE_diff);

auxdata.RThiS=predint(PostRes.RThiSfit.f,qShaS_diff);
auxdata.RFooS=predint(PostRes.RFooSfit.f,qShaS_diff);

%% weight values for objective and constraints
% Objective
switch auxdata.setting
    case {0,1}
        auxdata.objw_work        =0 * [1;1;1;1;1;1]; % simple setting
        auxdata.objw_u2          =0 * ([1;1;1;1;1;1]).^2;
        auxdata.objw_udot        =0 * [1;1;1;1;1;1]*1e-6;
        auxdata.objw_Fgdot       =0 * [1; 1; 1; 1]*1e-6;
        auxdata.objw_qdddot      =0 * ([1; 1; 1; 1; 1; 1; 1; 1; 1]./qdddot_rg).^2;
    case {10}
        auxdata.objw_work        =1e-3 * [1;1;1;1;1;1] / (BM*g*auxdata.pAy_rg) ;
        auxdata.objw_u2          =0 * ([1;1;1;1;1;1]).^2;
        auxdata.objw_udot        =0 * [1;1;1;1;1;1]*1e-6;
        auxdata.objw_Fgdot       =0 * [1; 1; 1; 1]*1e-6;
        auxdata.objw_qdddot      =1e-4 * ([1; 1; 1; 1; 1; 1; 1; 1; 1]./qdddot_rg).^2;
end

auxdata.objw_qSegErr     =1 * ([0; 0; 0;  0; 1/auxdata.qS_rg; 0; 0; 1/auxdata.qS_rg; 0]).^2; % shank angle
auxdata.objw_pJoiErr     =1 * ([0; 0; 0; 0; 1/auxdata.pAy_rg; 1/auxdata.pAy_rg;
                                0; 0; 0; 0; 1/auxdata.pAy_rg; 1/auxdata.pAy_rg]).^2; % ankle posi

% auxdata.objw_qSegErr     =1 * ([0; 0; 0;  0; 1; 0; 0; 1; 0]).^2; % shank angle
% auxdata.objw_pJoiErr     =1 * ([0; 0; 0; 0; 1; 1;
%                                 0; 0; 0; 0; 1; 1]).^2; % ankle posi

auxdata.objw_qErr        =0 * ([1; 1; 1; 1; 1; 1; 1; 1; 1]).^2;
auxdata.objw_qdotErr     =0 * ([1; 1; 1; 1; 1; 1; 1; 1; 1]).^2;
auxdata.objw_qddotErr    =0 *  [1; 1; 0; 0; 0; 0; 0; 0; 0;] ;
auxdata.objw_qdotSegErr  =0 * ([1; 1; 1; 1; 1; 1; 1; 1; 1]).^2;
auxdata.objw_pdotJoiErr  =0 * ([1; 1; 1; 1; 1; 1; 1; 1; 1;1; 1; 1;]).^2;
auxdata.objw_pddotJoiErr =0 * ([1; 1; 1; 1; 1; 1; 1; 1; 1;1; 1; 1;]).^2;

% Linear inequality  <=0
auxdata.lcneqw_peri        = 0 * [1;1;1;1;1]; 
auxdata.delta_q_perimin    = 0 * [1;1;1; 1;1;1; 1;1;1];
auxdata.delta_qdot_perimin = 0 * [1;1;1; 1;1;1; 1;1;1];
auxdata.delta_qddot_perimin= 0 * [1;1;1; 1;1;1; 1;1;1];
auxdata.delta_a_perimin    = 0 * [1;1;1; 1;1;1];
auxdata.delta_u_perimin    = 0 * [1;1;1; 1;1;1];

auxdata.delta_q_perimax    = 0 * [1;1;1; 1;1;1; 1;1;1];
auxdata.delta_qdot_perimax = 0 * [1;1;1; 1;1;1; 1;1;1];
auxdata.delta_qddot_perimax= 0 * [1;1;1; 1;1;1; 1;1;1];
auxdata.delta_a_perimax    = 0 * [1;1;1; 1;1;1];
auxdata.delta_u_perimax    = 0 * [1;1;1; 1;1;1];

% Non-Linear inequality  A*X<=b
% auxdata.nlcneqw_Fg=[1,1,  1,1,1,1,  1,1, 1]*1e-3; % min<Frgy1<max, Frgy 
auxdata.nlcneqw_Fg=[1,1,  1,1,1,1,  1,1, 0]*1e-3; % min<Frgy1<max, Frgy  no swing
auxdata.Fgy_HSmax=0.2*BM*g;
auxdata.Fgy_HSmin=0.01*BM*g;
auxdata.Fgyperimax=0.5*BM*g;
auxdata.Fgxperimax=0.25*BM*g;
% auxdata.Flgyswingmax=0.01*BM*g;
auxdata.Flgyswingmax=BM*g;

switch auxdata.setting
    case {0}
        auxdata.nlcneqw_qSeg=0;
    case {1,10}
        auxdata.nlcneqw_qSeg=1; % simple setting
end
auxdata.delta_qSeg_perimin = [-5;-1;-1; 
    auxdata.RThiE(1);-1;auxdata.RFooE(1); auxdata.RThiS(1);-1;auxdata.RFooS(1)]; % in rad
auxdata.delta_qSeg_perimax = [ 5; 1; 1; 
    auxdata.RThiE(2); 1;auxdata.RFooE(2); auxdata.RThiS(2); 1;auxdata.RFooS(2)];

% Non-Linear equality Aeq*X==beq
auxdata.nlceqw_qdot =1;
auxdata.nlceqw_qddot=1;
auxdata.nlceqw_Eq   =1;
auxdata.nlceqw_adot =1;

% Linear equality    ==0
auxdata.lceqw_peri      = 0 * [0;1;1;1;  1;1;1;  1;1;1; 1;1]; 
auxdata.lceqw_Velo      = 0 ;

auxdata.lceqw_OSx       = use_expOS;
auxdata.lceqw_OSy       = use_expOS;
auxdata.OSx_exp         = 0 ;
auxdata.OSy_exp         = 0 ;

%-------------------------------
lcneqAnz=Fun_lcneqA(auxdata.lcneqw_peri,auxdata.delta_q_perimin,auxdata.delta_qdot_perimin,...
    auxdata.delta_qddot_perimin,auxdata.delta_a_perimin,auxdata.delta_u_perimin,...
    auxdata.delta_q_perimax,auxdata.delta_qdot_perimax,...
    auxdata.delta_qddot_perimax,auxdata.delta_a_perimax,auxdata.delta_u_perimax);
lcneqA_size=auxdata.Grad_CONS.lcneqA_size;
auxdata.lcneqA=sparse(auxdata.Grad_CONS.lcneqAnz_row,auxdata.Grad_CONS.lcneqAnz_col,...
              lcneqAnz,lcneqA_size(1),lcneqA_size(2));
          
auxdata.lcneqb=Fun_lcneqb(auxdata.lcneqw_peri,auxdata.delta_q_perimin,auxdata.delta_qdot_perimin,...
    auxdata.delta_qddot_perimin,auxdata.delta_a_perimin,auxdata.delta_u_perimin,...
    auxdata.delta_q_perimax,auxdata.delta_qdot_perimax,...
    auxdata.delta_qddot_perimax,auxdata.delta_a_perimax,auxdata.delta_u_perimax);

%-------------------------------
lceqAnz=Fun_lceqA(auxdata.lceqw_peri,auxdata.lceqw_OSx,auxdata.lceqw_OSy);
lceqA_size=auxdata.Grad_CONS.lceqA_size;
auxdata.lceqA=sparse(auxdata.Grad_CONS.lceqAnz_row,auxdata.Grad_CONS.lceqAnz_col,...
              lceqAnz,lceqA_size(1),lceqA_size(2));
          
auxdata.lceqb=Fun_lceqb(auxdata.lceqw_peri,...
                    auxdata.lceqw_OSx,auxdata.lceqw_OSy,auxdata.OSx_exp,auxdata.OSy_exp);
% return;

%% whether to provide gradient and jacobian
auxdata.ObjGradFlag=true;
auxdata.ConGradFlag=true;
auxdata.ChkGradFlag=false;

%% Set-up and solve the nonlinear programming problem using fmincon

auxdata.boundFlag= 0 ;
switch auxdata.boundFlag
    case 0 % mean+- 3*SD
        a_step_LB=reshape(ones(N_step,6)*(-1),[],1);
        u_step_LB=reshape(ones(N_step,6)*(-1),[],1);
        
        OSx_LB       = -1;
        OSy_LB       = 0;
        
        a_step_UB=reshape(ones(N_step,6)*1,[],1);
        u_step_UB=reshape(ones(N_step,6)*1,[],1);
        OSx_UB       = 1;
        OSy_UB       = 0.2;
        
        
        lb=[reshape([q_min_Nstep,qdot_min_Nstep,...
            qddot_min_Nstep,a_min_Nstep],[],1);u_step_LB;
            OSx_LB;OSy_LB];
        ub=[reshape([q_max_Nstep,qdot_max_Nstep,...
            qddot_max_Nstep,a_max_Nstep],[],1);u_step_UB;
            OSx_UB;OSy_UB];
    case 1 % manual bound
        %-------------LLLLLLBBBBBB
        xb_step_LB      = ones(N_step,1)*(-5);
        xbdot_step_LB   = ones(N_step,1)*(0.5); % 0.8->0.5
        xbddot_step_LB  = ones(N_step,1)*(-5);  % 2->5
        yb_step_LB      = ones(N_step,1)*(0.5);
        ybdot_step_LB   = ones(N_step,1)*(-0.5); % 0.1->0.5
        ybddot_step_LB  = ones(N_step,1)*(-5);   % 2->5
        
        qb_step_LB      = ones(N_step,1)*(deg2rad(-5));
        qbdot_step_LB   = ones(N_step,1)*(deg2rad(-100));
        qbddot_step_LB  = ones(N_step,1)*(deg2rad(-1000));
        
        qrh_step_LB     = ones(N_step,1)*(deg2rad( -50));
        qrhdot_step_LB  = ones(N_step,1)*(deg2rad( -800));
        qrhddot_step_LB = ones(N_step,1)*(deg2rad( -8000));
        qlh_step_LB     = ones(N_step,1)*(deg2rad( -50));
        qlhdot_step_LB  = ones(N_step,1)*(deg2rad( -800));
        qlhddot_step_LB = ones(N_step,1)*(deg2rad( -8000));
        
        qrk_step_LB     = ones(N_step,1)*(deg2rad( -100));
        qrkdot_step_LB  = ones(N_step,1)*(deg2rad( -800));
        qrkddot_step_LB = ones(N_step,1)*(deg2rad( -8000));
        qlk_step_LB     = ones(N_step,1)*(deg2rad( -100));
        qlkdot_step_LB  = ones(N_step,1)*(deg2rad( -800));
        qlkddot_step_LB = ones(N_step,1)*(deg2rad( -8000));
        
        qra_step_LB     = ones(N_step,1)*(deg2rad( -30));
        qradot_step_LB  = ones(N_step,1)*(deg2rad( -800));
        qraddot_step_LB = ones(N_step,1)*(deg2rad( -8000));
        qla_step_LB     = ones(N_step,1)*(deg2rad( -30));
        qladot_step_LB  = ones(N_step,1)*(deg2rad( -800));
        qladdot_step_LB = ones(N_step,1)*(deg2rad( -8000));
        
        a_step_LB=reshape(ones(N_step,6)*(-1),[],1);
        u_step_LB=reshape(ones(N_step,6)*(-1),[],1);
        
        OSx_LB       = -1;
        OSy_LB       = -0.2;
        
        %-------------UUUUUUBBBBBB
        xb_step_UB      = ones(N_step,1)*(5);
        xbdot_step_UB   = ones(N_step,1)*(2); % 1.5->2.0
        xbddot_step_UB  = ones(N_step,1)*(5); % 2->5
        yb_step_UB      = ones(N_step,1)*(1.5);
        ybdot_step_UB   = ones(N_step,1)*(0.5); % 0.1->0.5
        ybddot_step_UB  = ones(N_step,1)*(5); % 2->5
        
        qb_step_UB      = ones(N_step,1)*(deg2rad( 5));
        qbdot_step_UB   = ones(N_step,1)*(deg2rad( 100));
        qbddot_step_UB  = ones(N_step,1)*(deg2rad( 1000));
        
        qrh_step_UB     = ones(N_step,1)*(deg2rad( 50));
        qrhdot_step_UB  = ones(N_step,1)*(deg2rad( 800));
        qrhddot_step_UB = ones(N_step,1)*(deg2rad( 8000));
        qlh_step_UB     = ones(N_step,1)*(deg2rad( 50));
        qlhdot_step_UB  = ones(N_step,1)*(deg2rad( 800));
        qlhddot_step_UB = ones(N_step,1)*(deg2rad( 8000));
        
        qrk_step_UB     = ones(N_step,1)*(deg2rad( 5)); % 10
        qrkdot_step_UB  = ones(N_step,1)*(deg2rad( 800));
        qrkddot_step_UB = ones(N_step,1)*(deg2rad( 8000));
        qlk_step_UB     = ones(N_step,1)*(deg2rad( 5));
        qlkdot_step_UB  = ones(N_step,1)*(deg2rad( 800));
        qlkddot_step_UB = ones(N_step,1)*(deg2rad( 8000));
        
        qra_step_UB     = ones(N_step,1)*(deg2rad( 30));  % 30
        qradot_step_UB  = ones(N_step,1)*(deg2rad( 800));
        qraddot_step_UB = ones(N_step,1)*(deg2rad( 8000));
        qla_step_UB     = ones(N_step,1)*(deg2rad( 30));
        qladot_step_UB  = ones(N_step,1)*(deg2rad( 800));
        qladdot_step_UB = ones(N_step,1)*(deg2rad( 8000));
        
        a_step_UB=reshape(ones(N_step,6)*1,[],1);
        u_step_UB=reshape(ones(N_step,6)*1,[],1);
        
        OSx_UB       = 1;
        OSy_UB       = 0.2;
        
        
        lb= [xb_step_LB;yb_step_LB;qb_step_LB;
            qrh_step_LB;qrk_step_LB;qra_step_LB;
            qlh_step_LB;qlk_step_LB;qla_step_LB;
            xbdot_step_LB;ybdot_step_LB;qbdot_step_LB;
            qrhdot_step_LB;qrkdot_step_LB;qradot_step_LB;
            qlhdot_step_LB;qlkdot_step_LB;qladot_step_LB;
            xbddot_step_LB;ybddot_step_LB;qbddot_step_LB;
            qrhddot_step_LB;qrkddot_step_LB;qraddot_step_LB;
            qlhddot_step_LB;qlkddot_step_LB;qladdot_step_LB;
            a_step_LB;u_step_LB;OSx_LB;OSy_LB];
        
        ub= [xb_step_UB;yb_step_UB;qb_step_UB;
            qrh_step_UB;qrk_step_UB;qra_step_UB;
            qlh_step_UB;qlk_step_UB;qla_step_UB;
            xbdot_step_UB;ybdot_step_UB;qbdot_step_UB;
            qrhdot_step_UB;qrkdot_step_UB;qradot_step_UB;
            qlhdot_step_UB;qlkdot_step_UB;qladot_step_UB;
            xbddot_step_UB;ybddot_step_UB;qbddot_step_UB;
            qrhddot_step_UB;qrkddot_step_UB;qraddot_step_UB;
            qlhddot_step_UB;qlkddot_step_UB;qladdot_step_UB;
            a_step_UB;u_step_UB;OSx_UB;OSy_UB];%
end

%% initial guess-------------------------------

switch auxdata.guessFlag
    case 0 % mean exp 
        X0=[reshape([q_mn_Nstep,qdot_mn_Nstep,...
            qddot_mn_Nstep,a_mn_Nstep,u_mn_Nstep],[],1);
            OSxmn;OSymn]; 
    case 10 % mean exp 
        X0=[reshape([q_mn_Nstep,qdot_mn_Nstep,...
            qddot_mn_Nstep,a_mn_Nstep,u_mn_Nstep],[],1);
            OSxmn;OSymn]+(ub-lb).*(rand(size(lb))-0.5)*2*0.3;     
    case 1 % fake motion
    xb_step_0     = linspace(0,0.5,N_step).';
    xbdot_step_0  = linspace(1.2,1.2,N_step).';
    xbddot_step_0  = linspace(0,0,N_step).';

    yb_step_0     = linspace(0.8,0.8,N_step).';
    ybdot_step_0  = linspace(0,0,N_step).';
    ybddot_step_0 = linspace(0,0,N_step).';

    qb_step_0     = linspace(deg2rad(0),deg2rad(0),N_step).';
    qbdot_step_0  = linspace(deg2rad( 0),deg2rad( 0),N_step).';
    qbddot_step_0  = linspace(deg2rad( 0),deg2rad( 0),N_step).';

    qrh_step_0    = linspace(deg2rad(20),deg2rad( -20),N_step).';
    qrhdot_step_0 = linspace(deg2rad(-50),deg2rad(-50),N_step).';
    qrhddot_step_0 = linspace(deg2rad(0),deg2rad(0),N_step).';
    qlh_step_0    = linspace(deg2rad(-20),deg2rad( 20),N_step).';
    qlhdot_step_0 = linspace(deg2rad(50),deg2rad( 50),N_step).';
    qlhddot_step_0 = linspace(deg2rad(0),deg2rad( 0),N_step).';

    qrk_step_0    = linspace(deg2rad(0),deg2rad( 0),N_step).';
    qrkdot_step_0 = linspace(deg2rad(0),deg2rad(0),N_step).';
    qrkddot_step_0 = linspace(deg2rad(0),deg2rad(0),N_step).';
    qlk_step_0    = linspace(deg2rad(0),deg2rad( 0),N_step).';
    qlkdot_step_0 = linspace(deg2rad(0),deg2rad( 0),N_step).';
    qlkddot_step_0 = linspace(deg2rad(0),deg2rad( 0),N_step).';

    qra_step_0    = linspace(deg2rad( 0),deg2rad( 0),N_step).';
    qradot_step_0 = linspace(deg2rad(0),deg2rad( 0),N_step).';
    qraddot_step_0 = linspace(deg2rad(0),deg2rad( 0),N_step).';
    qla_step_0    = linspace(deg2rad( 0),deg2rad( 0),N_step).';
    qladot_step_0 = linspace(deg2rad(0),deg2rad(0),N_step).';
    qladdot_step_0 = linspace(deg2rad(0),deg2rad(0),N_step).';

    a_step_0= reshape(ones(N_step,6)*0.05,[],1);
    u_step_0= reshape(ones(N_step,6)*0.05,[],1);

    OSx_0=-0.5;
    OSy_0=0.1;
    
    X0=[xb_step_0;yb_step_0;qb_step_0;
        qrh_step_0;qrk_step_0;qra_step_0;
        qlh_step_0;qlk_step_0;qla_step_0;
        xbdot_step_0;ybdot_step_0;qbdot_step_0;
        qrhdot_step_0;qrkdot_step_0;qradot_step_0;
        qlhdot_step_0;qlkdot_step_0;qladot_step_0;
        xbddot_step_0;ybddot_step_0;qbddot_step_0;
        qrhddot_step_0;qrkddot_step_0;qraddot_step_0;
        qlhddot_step_0;qlkddot_step_0;qladdot_step_0;
        a_step_0;u_step_0;OSx_0;OSy_0];
    case 2 % real MOC data
        a_step_0= expdata.TJ_MOC./(ones(N_step,1)*auxdata.TJmax);
        u_step_0= expdata.TJ_MOC./(ones(N_step,1)*auxdata.TJmax);
        X0=[reshape([expdata.q_MOC,expdata.qdot_MOC,expdata.qddot_MOC,a_step_0,u_step_0],[],1);
            auxdata.OSx_exp;auxdata.OSy_exp];
    case 3 % previous solution
        Res=load('*****.mat');
        X0=Res.Xopt;
end

%%
Fac_Xnorm=max(abs(lb),abs(ub));
lb=lb./Fac_Xnorm;
ub=ub./Fac_Xnorm;

X0=X0./Fac_Xnorm;

N_X=auxdata.Grad_CONS.N_X;
Jac_Xnorm=sparse(1:N_X,1:N_X,Fac_Xnorm,N_X,N_X);
auxdata.Jac_Xnorm = Jac_Xnorm;
auxdata.Fac_Xnorm = Fac_Xnorm;
auxdata.lb=lb;
auxdata.ub=ub;

% return;

%% test
auxdata.solver='fmincon';
plot_flag=10;
test_count=test_divide-1;  %#ok<NASGU>
% for i=1 :1000
X=X0;
[Result,myGrad] = V15_ObjFun(X0,auxdata);  %#ok<ASGLU>
% return;
%%
[myc,myceq,mycJac,myceqJac] = V15_ConFun(X0,auxdata);  %#ok<ASGLU>
mycJac=full(mycJac);
myceqJac=full(myceqJac);
% end
% return;
%%
if 0
%% manually check gradient/jacobian
    plot_flag=0;
    auxdata.ObjGradFlag = false;
    auxdata.ConGradFlag = false;
    
    [g1]=mklJac(@(x) V15_ObjFun(x,auxdata),X0); % mklJac can only compute jac of the first output
    ccc=[myGrad,g1.'];
    err1=(myGrad-g1.');
    err1max=(max(err1)).'
    
    [g2]=mklJac(@(x) V15_ConFunTest(x,auxdata,1),X0);
    err2=(mycJac-g2.');
    g2=g2.';
    num2str(reshape(err2(err2~=0),[],1));
    err2max=(max(err2)).';
    err2maxmax=(max(err2max)).'
    
    
    [g3]=mklJac(@(x) V15_ConFunTest(x,auxdata,2),X0);
    err3=(myceqJac-g3.');
    g3=g3.';
    num2str(reshape(err3(err3~=0),[],1));
    err3max=(max(err3));
    err3maxmax=(max(err3max)).'
    return;
end

%% DO Optimization sqp fimincon
auxdata.solver='ipopt';
plot_flag=0;
switch auxdata.solver
    case 'ipopt'
        if auxdata.ChkGradFlag==0
            test_divide=showcount;
        end
        [Xopt,fval,exitflag,output]=V15_Optimize_IPOPT(X0,lb,ub,auxdata);
end

if ~exist('Xopt','var')
    Xopt=X0;
end

%%
test_count=test_divide-1;
plot_flag=10;
[Result,myGrad] = V15_ObjFun(Xopt,auxdata);
Result.exitflag=exitflag;
Result.fval=fval;
Result.output=output;
[myc,myceq,mycJac,myceqJac] = V15_ConFun(Xopt,auxdata);
mycJac=full(mycJac).';
myceqJac=full(myceqJac).';
lcneq=auxdata.lcneqA*Xopt-auxdata.lcneqb;
lceq=auxdata.lceqA*Xopt-auxdata.lceqb;

disp(['nnz lcneq=',num2str(nnz(lcneq>1e-6))]);
disp(['nnz lceq=',num2str(nnz((abs(lceq)>1e-3)))]);
disp(['nnz nlcneq=',num2str(nnz(myc>1e-6))]);
disp(['nnz nlceq=',num2str(nnz((abs(myceq)>1e-3)))]);

X=Xopt;
%%
BW=Result.auxdata.BM*Result.auxdata.g;
BH=Result.auxdata.BH;
% disp('rms angle:')
RMSE=[rad2deg(rms([Result.Err.Err_q(:,4:6);Result.Err.Err_q(:,7:9)])),...
                rad2deg(rms([Result.Err.Err_qdot(:,4:6);Result.Err.Err_qdot(:,7:9)])),...
                rms([Result.Err.Err_TJ(:,1:3);Result.Err.Err_TJ(:,4:6)]/BW/BH*100),...
                rms([Result.Err.Err_GRF(:,1:2);Result.Err.Err_GRF(:,3:4)]/BW*100)]
% disp('StepL:')
StepL_Err=Result.StepL_Err
return;

%%

path(pathdef);
addpath('./TLFuns');
addpath(auxdata.EOM_Folder);
addpath(auxdata.Grad_Folder);

test_count=test_divide-1;
if ~exist('Xopt','var')
    Xopt=X./diag(auxdata.Jac_Xnorm);
end
plot_flag=10;

[Result,myGrad] = V15_ObjFun(Xopt,auxdata);
[myc,myceq,mycJac,myceqJac] = V15_ConFun(Xopt,auxdata);
lcneq=auxdata.lcneqA*Xopt-auxdata.lcneqb;
lceq=auxdata.lceqA*Xopt-auxdata.lceqb;

%% 
X=Xopt;
%%
Xopt=X;

%%
test_count=test_divide-1;
plot_flag=10;

[Result,myGrad] = V15_ObjFun(X0,auxdata);
[myc,myceq,mycJac,myceqJac] = V15_ConFun(X0,auxdata);













