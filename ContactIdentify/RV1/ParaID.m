% this is the main program to identify contact parameters

clear;clc;
set(0,'DefaultFigureVisible', 'off')
% set(0,'DefaultFigureVisible', 'on')  % !!! comment this , if run all the trials.

path(pathdef);
addpath('./TLFuns');
addpath('./AutoGeneFuns');

load('../../EXPData_Process/RV1/IMUVICONData20201124_175000.mat');

idx=0;
GaitTypes={'Norm','Slow','Fast','Rmin','Lmin'};

SubjectNum=1;
TypeNum=1;
TrialNum=1;

%%
for SubjectNum=  1 :12
    eval(['Subj=IMUVICONData.Subject',num2str(SubjectNum),';']);
%%
for TypeNum= 1 
    eval(['Gait=Subj.',GaitTypes{TypeNum},';']);
%%
for TrialNum=  1 :6
    close all;
    eval(['trial=Subj.',GaitTypes{TypeNum},'.Trial',num2str(TrialNum),';']);
    
    %%
    BH=Subj.aux.BH/100;
    BM=Subj.aux.BM;
    FootLen=(Subj.aux.FootL_HEEL+Subj.aux.FootL_TOE)/100;

    PY_AK=Subj.aux.FootH_Len/100;
    PX_HE=-Subj.aux.FootL_HEEL/100;
    PX_TO=Subj.aux.FootL_TOE/100;
    
    
    t_mkr=trial.Traj.t_mkr;

    FrameCount=trial.Traj.FrameCount;
    FrameCountDS=trial.Traj.FrameCountDS;

    Time_step=trial.Traj.Time_step;

    xra_s=trial.Traj.Traj_RAJC(:,2);
    yra_s=trial.Traj.Traj_RAJC(:,3);
    xla_s=trial.Traj.Traj_LAJC(:,2);
    yla_s=trial.Traj.Traj_LAJC(:,3);

    xradot_s =trial.Traj.TrajV_RAJC(:,2);
    yradot_s =trial.Traj.TrajV_RAJC(:,3);
    xladot_s =trial.Traj.TrajV_LAJC(:,2);
    yladot_s =trial.Traj.TrajV_LAJC(:,3);

    qrf_s = trial.Traj.AngP_RFoo-Gait.Res_Sta.AngP_RFoo_Sta;
    qlf_s = trial.Traj.AngP_LFoo-Gait.Res_Sta.AngP_LFoo_Sta;

    qrfdot_s =trial.Traj.AngV_RFoo;
    qlfdot_s =trial.Traj.AngV_LFoo;

    pAJ=[xra_s,yra_s,xla_s,yla_s];
    pAJdot=[xradot_s,yradot_s,xladot_s,yladot_s];
    qFO=[qrf_s,qlf_s];
    qFOdot=[qrfdot_s,qlfdot_s];

    COP_dyn=[trial.GRFMCOP.P2_g(:,2:3),trial.GRFMCOP.P1_g(:,2:3)];
    GRF_dyn=[trial.GRFMCOP.F2_g(:,2:3),trial.GRFMCOP.F1_g(:,2:3)];
    
    

%% 
figure;hold on;
set(gcf,'Position',[100 100 1100 600],'defaultLineLineWidth',1.5)
%---------------
subplot(3,3,1);hold on;title('pAJx');
plot(t_mkr,pAJ(:,1),'r');
plot(t_mkr+Time_step,pAJ(:,3),'r-.');

subplot(3,3,2);hold on;title('pAJy');
plot(t_mkr,pAJ(:,2),'r');
plot(t_mkr+Time_step,pAJ(:,4),'r-.');

subplot(3,3,3);hold on;
plot(t_mkr,qFO(:,1),'r');title('qFO');
plot(t_mkr+Time_step,qFO(:,2),'r-.');
legend({'Exp-R','Exp-L'});
%---------------
subplot(3,3,4);hold on;title('pAJxdot');
plot(t_mkr,pAJdot(:,1),'r');
plot(t_mkr+Time_step,pAJdot(:,3),'r-.');

subplot(3,3,5);hold on;title('pAJydot');
plot(t_mkr,pAJdot(:,2),'r');
plot(t_mkr+Time_step,pAJdot(:,4),'r-.');

subplot(3,3,6);hold on;title('qFOdot');
plot(t_mkr,qFOdot(:,1),'r');
plot(t_mkr+Time_step,qFOdot(:,2),'r-.');
%---------------
subplot(3,3,7);hold on;title('COP_dyn');
plot(t_mkr,COP_dyn(:,1));
plot(t_mkr,COP_dyn(:,3));
line(t_mkr(FrameCountDS)*ones(1,2),ylim(),'Color',[0.3,0.3,0.3],'LineStyle','--');
legend({'COPRx','COPLx'})

subplot(3,3,8);hold on;title('GRF_dyn');
plot(t_mkr,GRF_dyn(:,1));
plot(t_mkr,GRF_dyn(:,2));
plot(t_mkr,GRF_dyn(:,3));
plot(t_mkr,GRF_dyn(:,4));
line(t_mkr(FrameCountDS)*ones(1,2),ylim(),'Color',[0.3,0.3,0.3],'LineStyle','--');
legend({'GRFRx','GRFRy','GRFLx','GRFLy'})

%% Use SimBody Model

N_cts= 5 ;
R1_0=0.05;
R5_0=0.02;
yc_0=0;
ctsk_0=ones(N_cts,1)*2;
ctsb_0=ones(N_cts,1)*1.2;
ctsv0_0=0.1;

R1_Bd   =[ 0.01  ,  0.05];
R5_Bd   =[ 0.01  ,  0.05];
yc_Bd   =[ -0.02, 0.02  ];
ctsk_Bd=ones(N_cts,1)*[0.1 , 10000];
ctsb_Bd=ones(N_cts,1)*[0.1 , 5];
ctsv0_Bd=[0.001 ,  1 ];

X0=[ R1_0; R5_0;yc_0;ctsk_0;ctsb_0 ;ctsv0_0];
Bd=[R1_Bd;R5_Bd; yc_Bd;ctsk_Bd;ctsb_Bd;ctsv0_Bd];

lb=Bd(:,1);
ub=Bd(:,2);

linA0=[];
linb=[];
% X0=(rand(length(X0),1)-0.5)*10;

Aeq=[];
beq=[];
option=optimoptions('fmincon','Algorithm','sqp',...
    'Display','iter-detailed','MaxFunctionEvaluations',1e6);

% return;
%%
auxdata.FrameCount=FrameCount;
auxdata.FrameCountDS=FrameCountDS;

auxdata.BM=BM;
auxdata.BH=BH;
auxdata.FootLen=FootLen;
auxdata.N_cts=N_cts;

auxdata.PY_AK=PY_AK;
auxdata.PX_HE=PX_HE;
auxdata.PX_TO=PX_TO;

auxdata.GRF=GRF_dyn;
auxdata.COP=COP_dyn;

auxdata.pAJ=pAJ;
auxdata.qFO=qFO;
auxdata.pAJdot=pAJdot;
auxdata.qFOdot=qFOdot;

auxdata.t_mkr=t_mkr;

auxdata.plotflag=0;

path(pathdef);
addpath('./AutoGeneFuns');
if 0
%% test functions
    X=X0;
    auxdata.plotflag=1;
    [myf] = FootID_ObjFun(X0,auxdata);
    [myc,myceq] = FootID_ConFun(X0,auxdata);
    return;
end

%% Run optimization
auxdata.plotflag=0;
[Xopt,fval,exitflag,output]=fmincon(...
    @(x) FootID_ObjFun(x,auxdata),...
    X0,linA0,linb,Aeq,beq,lb,ub,...
    @(x) FootID_ConFun(x,auxdata),option);

%% Plot Results
auxdata.plotflag=1;
[myf] = FootID_ObjFun(Xopt,auxdata);
[myc,myceq] = FootID_ConFun(Xopt,auxdata);
X=Xopt;

%%
R1       = X(1);
R5       = X(2);
yc       = X(3);
k       = X(4:(4+N_cts-1))*1e4;
b       = X((4+N_cts):(4+N_cts+N_cts-1));
v0      = X(4+N_cts+N_cts);

dhex=auxdata.PX_HE+R1;
dhey=-auxdata.PY_AK+R1;
dtox=auxdata.PX_TO-R5;
dtoy=-auxdata.PY_AK+R5;

ctsx =linspace(dhex, dtox,N_cts);
ctsy =linspace(dhey, dtoy,N_cts);
ctsR      = ctsy-(ctsy(1)-R1);
ctsk      = k.';
ctsc      = b.';
ctsud     = ones(1,N_cts)*0.8;
ctsV      = ones(1,N_cts)*v0;

% idRes=TrialNum;
idx=idx+1;
Res_Info(idx,:)=[SubjectNum,TypeNum,TrialNum];
Results_collect{idx,1}=idx;
Results_collect{idx,2}=SubjectNum;
Results_collect{idx,3}=TypeNum;
Results_collect{idx,4}=TrialNum;
Results_collect{idx,5}=Xopt;
Results_collect{idx,6}=auxdata;
Results_collect{idx,7}=exitflag;

            
Res_ctsx(idx,:)=ctsx/FootLen;
Res_ctsy(idx,:)=ctsy/FootLen;
Res_ctsR(idx,:)=ctsR/FootLen;
Res_ctsk(idx,:)=ctsk/BM/9.8;
Res_ctsc(idx,:)=ctsc/BM/9.8;
Res_ctsud(idx,:)=ctsud;
Res_ctsV(idx,:)=ctsV;

disp(['SubjectNum= ',num2str(SubjectNum),',; TypeNum= ',num2str(TypeNum),...
    '; TrialNum= ',num2str(TrialNum),'; exitflag=',num2str(exitflag),';----------------']);

end
end
Res_ctsx_subj(SubjectNum,:)=mean(Res_ctsx(Res_Info(:,1)==SubjectNum,:));
Res_ctsy_subj(SubjectNum,:)=mean(Res_ctsy(Res_Info(:,1)==SubjectNum,:));
Res_ctsR_subj(SubjectNum,:)=mean(Res_ctsR(Res_Info(:,1)==SubjectNum,:));
Res_ctsk_subj(SubjectNum,:)=mean(Res_ctsk(Res_Info(:,1)==SubjectNum,:));
Res_ctsc_subj(SubjectNum,:)=mean(Res_ctsc(Res_Info(:,1)==SubjectNum,:));
Res_ctsud_subj(SubjectNum,:)=mean(Res_ctsud(Res_Info(:,1)==SubjectNum,:));
Res_ctsV_subj(SubjectNum,:)=mean(Res_ctsV(Res_Info(:,1)==SubjectNum,:));

end

Res_ctsx_mn=mean(Res_ctsx);
Res_ctsy_mn=mean(Res_ctsy);
Res_ctsR_mn=mean(Res_ctsR);
Res_ctsk_mn=mean(Res_ctsk);
Res_ctsc_mn=mean(Res_ctsc);
Res_ctsud_mn=mean(Res_ctsud);
Res_ctsV_mn=mean(Res_ctsV);

Results_table=cell2table(Results_collect,'VariableNames',{'idx','SubjectNum',...
    'TypeNum','TrialNum','Xopt','auxdata','exitflag'});

return;
%%
%%

save(['ParaIDres',char(datetime('now','format','yyyyMMdd_hhmmss')),'.mat'],...
    'Results_table',...
    'Res_ctsx','Res_ctsy','Res_ctsR','Res_ctsk','Res_ctsc','Res_ctsud','Res_ctsV',...
    'Res_ctsx_subj','Res_ctsy_subj','Res_ctsR_subj','Res_ctsk_subj','Res_ctsc_subj','Res_ctsud_subj','Res_ctsV_subj',...
    'Res_ctsx_mn','Res_ctsy_mn','Res_ctsR_mn','Res_ctsk_mn','Res_ctsc_mn','Res_ctsud_mn','Res_ctsV_mn');





%% for paper fig
load('ParaIDres20201124_060903.mat');
path(pathdef);
addpath('./TLFuns');
addpath('./AutoGeneFuns');


set(0,'DefaultFigureVisible', 'on') 
auxdata.plotflag=10;
idx=1; % paper fig use 1
[myf] = FootID_ObjFun(Results_table.Xopt{idx},Results_table{idx,6});




%%
% Res_cts=load('ParaIDres20201106_105533.mat');
cts=[Res_ctsR_mn.', Res_ctsk_mn.', Res_ctsc_mn.',...
    Res_ctsud_mn.', Res_ctsV_mn.'];

% cts_tab=









