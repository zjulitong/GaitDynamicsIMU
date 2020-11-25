clear;clc;

path(pathdef);
addpath('./TLFuns');
loadnum= 3;
%%

for loadnum= 3 % :3

switch loadnum
    case 1
        load('*****.mat'); %
        Ntri_T= 6 ;
    case 2
        load('*****.mat'); %
        Ntri_T= 1 ;
    case 3
        load('ResAll_BatchAll2_20201125_115435.mat'); %
        Ntri_T= 6 ;
end

%%
N_Res=size(Results_table,1);

Obj=zeros(N_Res,5);
objw_qSegErr=zeros(N_Res,9);
objw_pJoiErr=zeros(N_Res,12);

RMSE_q=zeros(N_Res,6);
RMSE_qdot=zeros(N_Res,6);
Err_StepL=zeros(N_Res,1);
StepL_Exp=zeros(N_Res,1);
StepL_Mod=zeros(N_Res,1);

ErrPst_StepL=zeros(N_Res,1);

Durations=zeros(N_Res,1);
IterCounts=zeros(N_Res,1);

Err_GRF=zeros(N_Res,2);
Err_TJ=zeros(N_Res,3);
Fg_cons=zeros(N_Res,5);

Exp_All=[];
Mod_All=[];
corr_All=[];
Res_All=[];

for i=1:N_Res
    ResName=Results_table.Result{i};
    load(ResName);
    Res_All{i,1}=Result;
    Res=Result;
    
    aux=Res.auxdata;
    BW=Res.auxdata.BM*Res.auxdata.g;
    BH=Res.auxdata.BH;
    
    Durations(i,1)=seconds(Res.auxdata.t_use);
    IterCounts(i,1)=Res.output.Iterations;
    Obj(i,:)=[Res.obj,Res.obj_qSegErr,Res.obj_pJoiErr,Res.obj_work,Res.obj_qdddot];
    Work(i,:)=sum(Res.work_sum);
    objw_qSegErr(i,:)=Res.auxdata.objw_qSegErr.';
    objw_pJoiErr(i,:)=Res.auxdata.objw_pJoiErr.';
    Fg_cons(i,:)=[Res.auxdata.Fgy_HSmax,Res.auxdata.Fgy_HSmin,Res.auxdata.Fgyperimax,Res.auxdata.Fgxperimax,Res.auxdata.Flgyswingmax];
    
    RMSE_q(i,:)=[rms(Res.Err.Err_q(:,1:3)),rms([Res.Err.Err_q(:,4:6);Res.Err.Err_q(:,7:9)])];
    RMSE_qdot(i,:)=[rms(Res.Err.Err_qdot(:,1:3)),rms([Res.Err.Err_qdot(:,4:6);Res.Err.Err_qdot(:,7:9)])];
    Err_StepL(i,1)=Res.StepL_Err;
    StepL_Exp(i,1)=Res.StepL_MOC;
    StepL_Mod(i,1)=Res.StepL_Model;

    ErrPst_StepL(i,1)=Res.StepL_Err/Res.StepL_MOC;
    
    Err_GRF(i,:)=rms([Res.Err.Err_GRF(:,1:2);Res.Err.Err_GRF(:,3:4)]/BW*100);
    Err_TJ(i,:)=rms([Res.Err.Err_TJ(:,1:3);Res.Err.Err_TJ(:,4:6)]/BW/BH*100);
    
    id_All=(i-1)*30*2;
    Exp_All(id_All+(1:30),:)=[rad2deg(aux.expdata.q_MOC(:,4:6)),rad2deg(aux.expdata.qdot_MOC(:,4:6)),aux.expdata.TJ_MOC(:,1:3)/BW/BH*100,aux.expdata.Fg_MOC(:,1:2)/BW*100];
    Exp_All(id_All+(31:60),:)=[rad2deg(aux.expdata.q_MOC(:,7:9)),rad2deg(aux.expdata.qdot_MOC(:,7:9)),aux.expdata.TJ_MOC(:,4:6)/BW/BH*100,aux.expdata.Fg_MOC(:,3:4)/BW*100];

    Mod_All(id_All+(1:30),:)=[rad2deg(Res.q_s(:,4:6)),rad2deg(Res.qdot_s(:,4:6)),Res.TJ_s(:,1:3)/BW/BH*100,Res.Fg_s(:,1:2)/BW*100];
    Mod_All(id_All+(31:60),:)=[rad2deg(Res.q_s(:,7:9)),rad2deg(Res.qdot_s(:,7:9)),Res.TJ_s(:,4:6)/BW/BH*100,Res.Fg_s(:,3:4)/BW*100];
    
    RMSE_All(i,:)=[rad2deg(rms([Res.Err.Err_q(:,4:6);Res.Err.Err_q(:,7:9)])),...
                rms([Res.Err.Err_TJ(:,1:3);Res.Err.Err_TJ(:,4:6)]/BW/BH*100),...
                rms([Res.Err.Err_GRF(:,1:2);Res.Err.Err_GRF(:,3:4)]/BW*100)];
        
    
    corr_All(i,:)=[TLcorrcoef([Res.q_s(:,4:6);Res.q_s(:,7:9)] , [aux.expdata.q_MOC(:,4:6);aux.expdata.q_MOC(:,7:9)]),...
        TLcorrcoef([Res.TJ_s(:,1:3);Res.TJ_s(:,4:6)] , [aux.expdata.TJ_MOC(:,1:3);aux.expdata.TJ_MOC(:,4:6)]),...
        TLcorrcoef([Res.Fg_s(:,1:2);Res.Fg_s(:,3:4)] , [aux.expdata.Fg_MOC(:,1:2);aux.expdata.Fg_MOC(:,3:4)])];

    Exp_range_All(i,:)=[range(rad2deg([aux.expdata.q_MOC(:,4:6);aux.expdata.q_MOC(:,7:9)])),...
        range([aux.expdata.TJ_MOC(:,1:3);aux.expdata.TJ_MOC(:,4:6)]/BW/BH*100),...
        range([aux.expdata.Fg_MOC(:,1:2);aux.expdata.Fg_MOC(:,3:4)]/BW*100)];
    
    Mod_range_All(i,:)=[range(rad2deg([Res.q_s(:,4:6);Res.q_s(:,7:9)])),...
        range([Res.TJ_s(:,1:3);Res.TJ_s(:,4:6)]/BW/BH*100),...
        range([Res.Fg_s(:,1:2);Res.Fg_s(:,3:4)]/BW*100)];
    
end
%%
eval(['Res',num2str(loadnum),'_RMSE=[RMSE_All,abs(Err_StepL*100)];']);
end

%%
Nuse=round(N_Res/5/Ntri_T);
Ntri=6;

id_Norm=reshape((((1:Nuse).'-1)*5*Ntri_T+( 1: Ntri)).',[],1);
id_Slow=reshape((((1:Nuse).'-1)*5*Ntri_T+(Ntri_T*1+(1:Ntri))).',[],1);
id_Fast=reshape((((1:Nuse).'-1)*5*Ntri_T+(Ntri_T*2+(1:Ntri))).',[],1);
id_Rmin=reshape((((1:Nuse).'-1)*5*Ntri_T+(Ntri_T*3+(1:Ntri))).',[],1);
id_Lmin=reshape((((1:Nuse).'-1)*5*Ntri_T+(Ntri_T*4+(1:Ntri))).',[],1);

id_normal=reshape((((1:Nuse).'-1)*5*Ntri_T+[(1:Ntri),Ntri_T*1+(1:Ntri),Ntri_T*2+(1:Ntri)]).',[],1);
id_abnormal=reshape((((1:Nuse).'-1)*5*Ntri_T+[Ntri_T*3+(1:Ntri),Ntri_T*4+(1:Ntri)]).',[],1);

id_all=reshape((((1:Nuse).'-1)*5*Ntri_T+[(1:Ntri),Ntri_T*1+(1:Ntri),Ntri_T*2+(1:Ntri),Ntri_T*3+(1:Ntri),Ntri_T*4+(1:Ntri)]).',[],1);

Types=cell(N_Res,1);
for i=1:N_Res/5/Ntri_T*Ntri
Types{id_Norm(i)}='Norm';
Types{id_Slow(i)}='Slow';
Types{id_Fast(i)}='Fast';
Types{id_Rmin(i)}='Rmin';
Types{id_Lmin(i)}='Lmin';
end
Err_range=Exp_range_All-Mod_range_All;
%%
Err_All=[RMSE_All,abs(Err_StepL)*100];
Err_All_mn=[mean(Err_All(id_Slow,:));mean(Err_All(id_Norm,:));mean(Err_All(id_Fast,:));mean(Err_All(id_Rmin,:));mean(Err_All(id_Lmin,:));
            mean(Err_All(id_all,:));mean(Err_All(id_normal,:));mean(Err_All(id_abnormal,:))].';
Err_All_sd=[std(Err_All(id_Slow,:));std(Err_All(id_Norm,:));std(Err_All(id_Fast,:));std(Err_All(id_Rmin,:));std(Err_All(id_Lmin,:));
            std(Err_All(id_all,:));std(Err_All(id_normal,:));std(Err_All(id_abnormal,:))].';
ErrPst_All=[RMSE_All./Exp_range_All*100,abs(ErrPst_StepL)*100];
ErrPst_All_mn=[mean(ErrPst_All(id_Slow,:));mean(ErrPst_All(id_Norm,:));mean(ErrPst_All(id_Fast,:));mean(ErrPst_All(id_Rmin,:));mean(ErrPst_All(id_Lmin,:));
            mean(ErrPst_All(id_all,:));mean(ErrPst_All(id_normal,:));mean(ErrPst_All(id_abnormal,:))].';
ErrPst_All_sd=[std(ErrPst_All(id_Slow,:));std(ErrPst_All(id_Norm,:));std(ErrPst_All(id_Fast,:));std(ErrPst_All(id_Rmin,:));std(ErrPst_All(id_Lmin,:));
            std(ErrPst_All(id_all,:));std(ErrPst_All(id_normal,:));std(ErrPst_All(id_abnormal,:))].';


%%

Err_All_SubjAve=zeros(Nuse,size(Err_All,2));
ErrPst_All_SubjAve=zeros(Nuse,size(Err_All,2));
Exp_range_SubjAve=zeros(Nuse,size(Exp_range_All,2));

Err_All_SubjTypeAve=zeros(Nuse*5,size(Err_All,2));
ErrPst_All_SubjTypeAve=zeros(Nuse*5,size(Err_All,2));
for subj=1:Nuse
    Err_All_SubjAve(subj,:)=mean(Err_All(id_all(5*Ntri*(subj-1)+1:5*Ntri*subj),:));
    ErrPst_All_SubjAve(subj,:)=mean(ErrPst_All(id_all(5*Ntri*(subj-1)+1:5*Ntri*subj),:));
    Exp_range_SubjAve(subj,:)=mean(Exp_range_All(id_all(5*Ntri*(subj-1)+1:5*Ntri*subj),:));
    
    Err_All_SubjTypeAve(5*(subj-1)+1:5*subj,:)=[mean(Err_All(id_all(5*Ntri*(subj-1)+0*Ntri+1:5*Ntri*(subj-1)+1*Ntri),:));...
                                  mean(Err_All(id_all(5*Ntri*(subj-1)+1*Ntri+1:5*Ntri*(subj-1)+2*Ntri),:));...
                                  mean(Err_All(id_all(5*Ntri*(subj-1)+2*Ntri+1:5*Ntri*(subj-1)+3*Ntri),:));...
                                  mean(Err_All(id_all(5*Ntri*(subj-1)+3*Ntri+1:5*Ntri*(subj-1)+4*Ntri),:));...
                                  mean(Err_All(id_all(5*Ntri*(subj-1)+4*Ntri+1:5*Ntri*(subj-1)+5*Ntri),:))];
    ErrPst_All_SubjTypeAve(5*(subj-1)+1:5*subj,:)=[mean(ErrPst_All(id_all(5*Ntri*(subj-1)+0*Ntri+1:5*Ntri*(subj-1)+1*Ntri),:));...
                                  mean(ErrPst_All(id_all(5*Ntri*(subj-1)+1*Ntri+1:5*Ntri*(subj-1)+2*Ntri),:));...
                                  mean(ErrPst_All(id_all(5*Ntri*(subj-1)+2*Ntri+1:5*Ntri*(subj-1)+3*Ntri),:));...
                                  mean(ErrPst_All(id_all(5*Ntri*(subj-1)+3*Ntri+1:5*Ntri*(subj-1)+4*Ntri),:));...
                                  mean(ErrPst_All(id_all(5*Ntri*(subj-1)+4*Ntri+1:5*Ntri*(subj-1)+5*Ntri),:))];
    Id_SubjTypeAve(5*(subj-1)+1:5*subj,:)=[1;1;1;1;1]*subj;
end

[Err_All_Subj_max,Err_All_Subj_maxid]=max(Err_All_SubjAve);
Err_All_Subj_maxid

for i=1:5
    [Err_All_TypeSubj_max(i,:),Err_All_TypeSubj_maxid(i,:)]=max(Err_All_SubjTypeAve(i:5:end,:));
    [ErrPst_All_TypeSubj_max(i,:),ErrPst_All_TypeSubj_maxid(i,:)]=max(ErrPst_All_SubjTypeAve(i:5:end,:));
end
    
%%
BHeights=[182;182;179;177;160;170;161;157;158;180;173;170;]/100;%[cm]
BMasses=[66.0;73.5;78.3;76.8;46.8;59.0;54.5;58.7;57.3;78.0;71.5;68.9;];% [kg]
x_BH=linspace(min(BHeights),max(BHeights),10).';
x_BM=linspace(min(BMasses),max(BMasses),10).';
gof_BHBM=[];
titles={'Hip angle','Knee angle','Ankle angle'};
figure;
set(gcf,'Position',[100 100 800 350],'defaultLineLineWidth',1.5);
for i=1:3
    [f,gof,~] = fit(BHeights,Err_All_SubjAve(:,i),'poly1');
    gof_BHBM(1,i)=gof.adjrsquare;
    gof_BHBM(3,i)=gof.rsquare;
    
    disp(['BH1: y=',num2str(f.p1,'%0.2f'),'x',num2str(f.p2,'%+0.2f')]); % (a)
disp(['R^2=',num2str(gof.rsquare,'%0.2f')]);

    subplot(2,3,i);hold on;title(titles{i});
    plot(BHeights,Err_All_SubjAve(:,i),'ro');
    plot(x_BH,f.p1*x_BH+f.p2,'k');
    xlabel(['Body height (m)']);
    if i==1
        ylabel('RMSE (\circ)')
    end

    [f,gof,~] = fit(BMasses,Err_All_SubjAve(:,i),'poly1');
    gof_BHBM(2,i)=gof.adjrsquare;
    gof_BHBM(4,i)=gof.rsquare;
    disp(['BM1: y=',num2str(f.p1,'%0.2f'),'x',num2str(f.p2,'%+0.2f')]); % (a)
disp(['R^2=',num2str(gof.rsquare,'%0.2f')]);
    
    subplot(2,3,i+3);hold on;
    plot(BMasses,Err_All_SubjAve(:,i),'ro');
    plot(x_BM,f.p1*x_BM+f.p2,'k');
    xlabel(['Body mass (Kg)']);
    if i==1
        ylabel('RMSE (\circ)')
    end
    if i==3
        legend({'subjects','linear fit'},'Box','off')
    end
end

%% Table IV for paper
RMSE_All_use=RMSE_All./Exp_range_All*100;
ErrPst_StepL_use=abs(ErrPst_StepL)*100;
RMSE_All_mn=[mean(RMSE_All_use(id_Slow,:));mean(RMSE_All_use(id_Norm,:));mean(RMSE_All_use(id_Fast,:));
    mean(RMSE_All_use(id_Rmin,:));mean(RMSE_All_use(id_Lmin,:));mean(RMSE_All_use(id_all,:));mean(RMSE_All_use(id_normal,:));mean(RMSE_All_use(id_abnormal,:))].';
RMSE_All_sd=[std(RMSE_All_use(id_Slow,:));std(RMSE_All_use(id_Norm,:));std(RMSE_All_use(id_Fast,:));
    std(RMSE_All_use(id_Rmin,:));std(RMSE_All_use(id_Lmin,:));std(RMSE_All_use(id_all,:));std(RMSE_All_use(id_normal,:));std(RMSE_All_use(id_abnormal,:))].';
ErrPst_StepL_mn=[mean(ErrPst_StepL_use(id_Slow,:));mean(ErrPst_StepL_use(id_Norm,:));mean(ErrPst_StepL_use(id_Fast,:));
    mean(ErrPst_StepL_use(id_Rmin,:));mean(ErrPst_StepL_use(id_Lmin,:));mean(ErrPst_StepL_use(id_all,:));mean(ErrPst_StepL_use(id_normal,:));mean(ErrPst_StepL_use(id_abnormal,:))].';
ErrPst_StepL_sd=[std(ErrPst_StepL_use(id_Slow,:));std(ErrPst_StepL_use(id_Norm,:));std(ErrPst_StepL_use(id_Fast,:));
    std(ErrPst_StepL_use(id_Rmin,:));std(ErrPst_StepL_use(id_Lmin,:));std(ErrPst_StepL_use(id_all,:));std(ErrPst_StepL_use(id_normal,:));std(ErrPst_StepL_use(id_abnormal,:))].';

allmn=[RMSE_All_mn;ErrPst_StepL_mn];
allsd=[RMSE_All_sd;ErrPst_StepL_sd];

lens=[1 1 1 1 1 1 1 1,   1];
for i=1:size(allmn,1)
    for j=1:size(allmn,2)
        form=['%.',num2str(lens(i)),'f'];
        table_pst{i,j}=[num2str(allmn(i,j),form),' (',num2str(allsd(i,j),form),')'];
        table_err{i,j}=[num2str(Err_All_mn(i,j),form),' (',num2str(Err_All_sd(i,j),form),')'];
    end
end

%%
RMSE_StepL_normal=rms(Err_StepL(id_normal,:))*100;
RMSE_StepLPst_normal=rms(ErrPst_StepL(id_normal,:))*100;

RMSE_StepL=rms(Err_StepL(:,:))*100;
RMSE_StepLPst=rms(ErrPst_StepL(:,:))*100;


return;

%%
load('../../EXPData_Process_V2\V1.2.1/IMUVICONData20201118_151329.mat'); 

for idx=1:size(Results_table,1)
    ResName=Results_table.Result{idx};
    load(ResName);
    Res_idx=Result;

eval(['Subj=IMUVICONData.Subject',num2str(Results_table.subi(idx)),';']);
eval(['Gait=Subj.',Res_idx.auxdata.GaitTypes{Results_table.typei(idx)},';']);
eval(['trial=Subj.',Res_idx.auxdata.GaitTypes{Results_table.typei(idx)},'.Trial',num2str(Results_table.trii(idx)),';']);

Traj=trial.Traj;
Res_Sta=Gait.Res_Sta;

MOCOffsetx=Traj.Traj_RAJC(round(end/2),2);
OPy=Traj.Traj_OPGT(:,2)-MOCOffsetx;
OPz=Traj.Traj_OPGT(:,3);


% q_MOC=[OPy,OPz,Traj.AngP_Pelv-Res_Sta.AngP_Pelv_Sta,...
%     Traj.AngP_RThi-Res_Sta.AngP_RThi_Sta,...
%     Traj.AngP_RKne-Res_Sta.AngP_RKne_Sta,Traj.AngP_RAnk-Res_Sta.AngP_RAnk_Sta,...
%     Traj.AngP_LThi-Res_Sta.AngP_LThi_Sta,...
%     Traj.AngP_LKne-Res_Sta.AngP_LKne_Sta,Traj.AngP_LAnk-Res_Sta.AngP_LAnk_Sta];

% q_MOC=[OPy,OPz,Traj.AngP_Pelv-mean(Traj.AngP_Pelv),... % similar, seems worse
%     Traj.AngP_RThi-Res_Sta.AngP_RThi_Sta-(Traj.AngP_Pelv-mean(Traj.AngP_Pelv)),...
%     Traj.AngP_RKne-Res_Sta.AngP_RKne_Sta,Traj.AngP_RAnk-Res_Sta.AngP_RAnk_Sta,...
%     Traj.AngP_LThi-Res_Sta.AngP_LThi_Sta-(Traj.AngP_Pelv-mean(Traj.AngP_Pelv)),...
%     Traj.AngP_LKne-Res_Sta.AngP_LKne_Sta,Traj.AngP_LAnk-Res_Sta.AngP_LAnk_Sta];

q_MOC=[OPy,OPz,Traj.AngP_Pelv-Res_Sta.AngP_Pelv_Sta,...
    Traj.AngP_RThi-Res_Sta.AngP_RThi_Sta,...
    Traj.AngP_RKne,Traj.AngP_RAnk,...
    Traj.AngP_LThi-Res_Sta.AngP_LThi_Sta,...
    Traj.AngP_LKne,Traj.AngP_LAnk];

t_mkr_IMU=linspace(0,trial.IMURes.Tstep,length(trial.Traj.t_mkr)).';
expdata=Res_idx.auxdata.expdata;
t_mkr=trial.Traj.t_mkr;
Fg_MOC=[trial.GRFMCOP.F2_g(:,2),trial.GRFMCOP.F2_g(:,3),...
        trial.GRFMCOP.F1_g(:,2),trial.GRFMCOP.F1_g(:,3)];
TJ_MOC=[trial.JM2D.JM2D_RHip,trial.JM2D.JM2D_RKne,trial.JM2D.JM2D_RAnk,...
        trial.JM2D.JM2D_LHip,trial.JM2D.JM2D_LKne,trial.JM2D.JM2D_LAnk];

Tstep_MOC=trial.Traj.Time_step;
Tstep_IMU=trial.IMURes.Tstep;

q_MOC_new        =TLinterp1_nan(t_mkr_IMU,q_MOC     ,expdata.t_grid_step,'pchip');
% Err_TJ=TJ_s-TJ_MOC;
% Err_GRF=Fg_s-Fg_MOC;

Err_q=(Res_idx.q_s-q_MOC_new);
RMSE_q_new(idx,:)=[rad2deg(rms([Err_q(:,4:6);Err_q(:,7:9)]))];
end

RMSE_q_old_mn=mean(RMSE_All(:,1:3))
RMSE_q_new_mn=mean(RMSE_q_new(:,:))


%%
figure;hold on;
set(gcf,'Position',[100 100 1400 800],'defaultLineLineWidth',1.5);
for i=1:6
    subplot(4,3,i);hold on;title(['q',num2str(i)]);
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
end

for i=1:3 % TJ
    subplot(4,3,i+6);hold on;title(['TJ',num2str(i)]);
    plot(t_mkr,TJ_MOC(:,i),'k-');
    plot(expdata.t_grid_step,expdata.TJ_MOC(:,i),'r--');
    plot(t_mkr+Tstep_IMU,TJ_MOC(:,i+3),'k-.');
    plot(expdata.t_grid_step+Tstep_IMU,expdata.TJ_MOC(:,i+3),'r--');
end
for i=1:2 % GRF
    subplot(4,3,i+9);hold on;title(['GRF',num2str(i)]);
    plot(t_mkr,Fg_MOC(:,i),'k-');
    plot(expdata.t_grid_step,expdata.Fg_MOC(:,i),'r--');
    plot(t_mkr+Tstep_IMU,Fg_MOC(:,i+2),'k-.');
    plot(expdata.t_grid_step+Tstep_IMU,expdata.Fg_MOC(:,i+2),'r--');
end








%%
ErrTable=cell(9,8);
Err_mn=[[RMSE_All_mn*100,corr_All_mn];[Err_StepL_use_mn*100,zeros(1,4)]];
Err_sd=[[RMSE_All_sd*100,corr_All_sd];[Err_StepL_use_sd*100,zeros(1,4)]];

Err_anova=[[RMSE_Shank_anova,corr_anova];[ErrPst_StepL_anova,zeros(1,4)*nan]];
Err_Len=[1 1 1 1 2 2 2 2];
for i=1:9
    for j=1:8
        form=['%.',num2str(Err_Len(j)),'f'];
        ErrTable{i,j}=[num2str(Err_mn(i,j),form),...
            ' (',num2str(Err_sd(i,j),form),')'];
        if i==12&&j>4
            ErrTable{i,j}=[];
        end
        if j==2
           if  Err_anova(i,1)<0.05 && Err_anova(i,2)<0.05 
            ErrTable{i,j}=[ErrTable{i,j},'*'];
           end
        end
        if j==3
           if  Err_anova(i,1)<0.05 && Err_anova(i,3)<0.05 
            ErrTable{i,j}=[ErrTable{i,j},'#'];
           end
           if  Err_anova(i,1)<0.05 && Err_anova(i,4)<0.05 
            ErrTable{i,j}=[ErrTable{i,j},'&'];
           end
        end
    end
end


%% step length
% [SLreg.bx,SLreg.bintx,SLreg.rx,SLreg.rintx,SLreg.statsx] = ...
%     regress(StepL_Mod,[ones(size(StepL_Exp)),StepL_Exp]);

C1= [110 110 110]/255;
C2= [79 129 189]/255;
C3= [255 48 48]/255;

figure;hold on;
% set(gcf,'Position',[100 100 500 250]);
set(gcf,'defaultLineLineWidth',1,'defaultLineMarkerSize',4);
axis equal
plot(StepL_Exp(id_Norm),StepL_Mod(id_Norm),'o','MarkerFaceColor',C1,'MarkerEdgeColor','none');
plot(StepL_Exp(id_Slow),StepL_Mod(id_Slow),'o','MarkerFaceColor',C2,'MarkerEdgeColor','none');
plot(StepL_Exp(id_Fast),StepL_Mod(id_Fast),'o','MarkerFaceColor',C2,'MarkerEdgeColor','none');
plot(StepL_Exp(id_Rmin),StepL_Mod(id_Rmin),'o','MarkerFaceColor',C3,'MarkerEdgeColor','none');
plot(StepL_Exp(id_Lmin),StepL_Mod(id_Lmin),'o','MarkerFaceColor',C3,'MarkerEdgeColor','none');

x_reg=0.4:0.001:0.8;
% y_reg=SLreg.bx(2)*x_reg+SLreg.bx(1);

[fit1,fit2,fit3]=fit(StepL_Exp,StepL_Mod,'poly1')
line(x_reg,fit1(x_reg),'Color','k','LineWidth',2);
    
ylim(xlim())
legend({'Lmin','Norm','Rmin','Regression'})
xlabel('Experimental Step Length(m)');
ylabel('Estimated Step Length (m)');

%% StepL errpst Plot

% C1= [110 110 110]/255;
% C2= [79 129 189]/255;
% C3= [255 48 48]/255;
% x_reg=0.4:0.001:0.8;
% % y_reg=SLreg.bx(2)*x_reg+SLreg.bx(1);
% 
% yuse=Err_StepL;
% yuse=ErrPst_StepL*100;
% yuse=abs(ErrPst_StepL*100);
% 
% [fit1,fit2,fit3]=fit(StepL_Exp,yuse,'poly1');
% 
% figure;hold on;
% set(gcf,'Position',[100 100 500 250]);
% line([0 1], [0 0],'Color',[1 1 1]*0.3,'HandleVisibility','off')
% % axis equal
% plot(StepL_Exp(id_Norm),yuse(id_Norm),'o','MarkerFaceColor',C1,'MarkerEdgeColor','none');
% plot(StepL_Exp(id_Slow),yuse(id_Slow),'o','MarkerFaceColor',C2,'MarkerEdgeColor','none');
% plot(StepL_Exp(id_Fast),yuse(id_Fast),'o','MarkerFaceColor',C2,'MarkerEdgeColor','none');
% plot(StepL_Exp(id_Rmin),yuse(id_Rmin),'o','MarkerFaceColor',C3,'MarkerEdgeColor','none');
% plot(StepL_Exp(id_Lmin),yuse(id_Lmin),'o','MarkerFaceColor',C3,'MarkerEdgeColor','none');
% line(x_reg,fit1(x_reg),'Color','k','LineWidth',2);
% 
% legend({'Norm','Slow','Fast','Rmin','Lmin','Regression'})
% xlim([0.35 0.8]);
% % ylim([-40 40])
% xlabel('Step Length(m)');
% ylabel('Error of Step Length (m)');

%% angle\torque, point compare.
% idpool=1:60*5; % 
% idpool=1:length(Exp_All); % 
% 
% idpool_nor=reshape(((id_Norm.'-1)*60+[1:60]).',[],1);
% idpool_abn=reshape(((i_abn.'-1)*60+[1:60]).',[],1);
% 
% figure;hold on;
% set(gcf,'Position',[100 100 1000 600]);
% 
% C1= [110 110 110]/2/255;
% C2= [79 59 209]/255;
% C3= [205 48 48]/255;
% 
% colpool=[1 5 9 2 6 10 3 7 11 4 8];
% titles={'Hip Angle','Knee Angle','Ankle Angle','Hip Angle Velo','Knee Angle Velo','Ankle Angle Velo','Hip Torque','Knee Torque','Ankle Torque','GRF-x','GRF-y'};
% for i=1:11
%     subplot(3,4,colpool(i));hold on;
%     title(titles{i},'FontSize',9);
%     
%     axis equal
%     
%     plot(Exp_All(idpool_nor,i),Mod_All(idpool_nor,i),'.','Color',C2,'MarkerSize',3);
%     plot(Exp_All(idpool_abn,i),Mod_All(idpool_abn,i),'.','Color',C3,'MarkerSize',3);
% 
% %     plot(Exp_All(idpool,i),'k.');
% %     plot(Mod_All(idpool,i),'r.');
%     
% %     plot(idpool*0+1,Exp_All(idpool,i),'k.');
% %     plot(idpool*0+1,Mod_All(idpool,i),'r.');
% 
%     xfit=linspace(min(Exp_All(idpool,i)),max(Exp_All(idpool,i)),50);
%     
%     [fit1,fit2,fit3]=fit(Exp_All(idpool,i),Mod_All(idpool,i),'poly1');
%     disp(['col=',num2str(i),' p1=',num2str(fit1.p1),' p2=',num2str(fit1.p2),';']);
%     
%     plot(xfit,fit1(xfit),'Color',C1,'LineWidth',1.5);
%     
% end






% return;
%% show a single trial
global test_count test_divide plot_flag;
test_count=0; % count to show temperary resutls.
test_divide=100000;
plot_flag=0;

idx= 26 ; % 26 is ok, 
idx= 185 ; % 26 is ok, 

load(Results_table.Result{idx});
Res_idx=Result;
addpath('TLFuns');
addpath(Res_idx.auxdata.EOM_Folder);
addpath(Res_idx.auxdata.Grad_Folder);

test_count=test_divide-1;

plot_flag=10;

[Result,myGrad] = V15_ObjFun(Res_idx.X_res, Res_idx.auxdata);
[myc,myceq,mycJac,myceqJac] = V15_ConFun(Res_idx.X_res,Res_idx.auxdata);
Nceq=sum(abs(myceq)>1e-4)






