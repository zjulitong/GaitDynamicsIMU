% this file is used to generate figures for paper.
clear;clc;
%%
path(pathdef);
addpath('./TLFuns');
addpath('./AutoGeneFuns');

%%
load('ResAll_BatchAll2_20201125_115435.mat'); % 8 each type
idx= 1 ;

load(Results_table.Result{idx});
Res_idx=Result;

addpath(Res_idx.auxdata.Grad_Folder);
NOS=1;

auxdata=Res_idx.auxdata;

%%
Jac_Xnorm = auxdata.Jac_Xnorm;
Grad_CONS = auxdata.Grad_CONS;
EOM_CONS  = auxdata.EOM_CONS;

N_step    = auxdata.N_step;
cts       = auxdata.cts;
N_cts     = EOM_CONS.N_cts;

TSP_exp   = auxdata.TSP_exp;
epsl      = auxdata.epsl;

N_q       = Grad_CONS.N_q;
N_g       = Grad_CONS.N_g;
N_u       = Grad_CONS.N_u;

N_X       = Grad_CONS.N_X;
N_var     = Grad_CONS.N_var;
expdata=auxdata.expdata;

CON_Len=auxdata.CON_Len;

%% Extract initial guess to optimize

X=Jac_Xnorm*Res_idx.X_res;
ccnow=0;

q_s       = reshape(X(ccnow+1:ccnow+N_step*N_q),[],N_q); ccnow=ccnow+N_step*N_q;
qdot_s    = reshape(X(ccnow+1:ccnow+N_step*N_q),[],N_q); ccnow=ccnow+N_step*N_q;
qddot_s   = reshape(X(ccnow+1:ccnow+N_step*N_q),[],N_q); ccnow=ccnow+N_step*N_q;
a_s       = reshape(X((ccnow+1):(ccnow+N_step*N_u)),[],N_u); ccnow=ccnow+N_step*N_u;
u_s       = reshape(X((ccnow+1):(ccnow+N_step*N_u)),[],N_u); ccnow=ccnow+N_step*N_u;

OSx =X(ccnow+1); ccnow=ccnow+1;
OSy =X(ccnow+1); ccnow=ccnow+1;

Time_step=TSP_exp;

%%

ctsx=cts.ctsx;
ctsy=cts.ctsy;

yrcts   =Fun_yrcts(   q_s,CON_Len,ctsx,ctsy);
xrctsdot=Fun_xrctsdot(q_s,qdot_s,CON_Len,ctsx,ctsy);
yrctsdot=Fun_yrctsdot(q_s,qdot_s,CON_Len,ctsx,ctsy);

ylcts   =Fun_ylcts(   q_s,CON_Len,ctsx,ctsy);
xlctsdot=Fun_xlctsdot(q_s,qdot_s,CON_Len,ctsx,ctsy);
ylctsdot=Fun_ylctsdot(q_s,qdot_s,CON_Len,ctsx,ctsy);

ctsR      = cts.ctsR;
ctsk      = cts.ctsk;
ctsc      = cts.ctsc;
ctsud     = cts.ctsud;
ctsV      = cts.ctsV;
[cts_FRx]=Fun_Contact_Sim_Fx(ctsR,ctsk,ctsc,ctsud,ctsV,yrcts,yrctsdot,xrctsdot);
[cts_FRy]=Fun_Contact_Sim_Fy(ctsR,ctsk,ctsc,           yrcts,yrctsdot);
[cts_FLx]=Fun_Contact_Sim_Fx(ctsR,ctsk,ctsc,ctsud,ctsV,ylcts,ylctsdot,xlctsdot);
[cts_FLy]=Fun_Contact_Sim_Fy(ctsR,ctsk,ctsc,           ylcts,ylctsdot);

Frgx_s=sum(cts_FRx,2);
Frgy_s=sum(cts_FRy,2);
Flgx_s=sum(cts_FLx,2);
Flgy_s=sum(cts_FLy,2);

Fg_s=[Frgx_s,Frgy_s,Flgx_s,Flgy_s];
%%

q_MOC    =expdata.q_MOC;
qdot_MOC =expdata.qdot_MOC;
qddot_MOC=expdata.qddot_MOC;
Fg_MOC   =expdata.Fg_MOC;
TJ_MOC   =expdata.TJ_MOC;

qSeg_MOC    =expdata.qSeg_MOC;
qdotSeg_MOC =expdata.qdotSeg_MOC;
pJoi_MOC    =expdata.pJoi_MOC;
pdotJoi_MOC =expdata.pdotJoi_MOC;
pddotJoi_MOC =expdata.pddotJoi_MOC;

qSeg_track=expdata.qSeg_track;
qdotSeg_track =expdata.qdotSeg_track;
pJoi_track    =expdata.pJoi_track;
pdotJoi_track =expdata.pdotJoi_track;
pddotJoi_track =expdata.pddotJoi_track;


TJmax     = auxdata.TJmax;
TJ_s=Fun_T_Joi(a_s,TJmax);
power_s=Fun_P_Joi(qdot_s,TJ_s);

qSeg=[q_s(:,1:3), Fun_qrt(q_s),Fun_qrs(q_s),Fun_qrf(q_s),...
                  Fun_qlt(q_s),Fun_qls(q_s),Fun_qlf(q_s)];
qdotSeg=[qdot_s(:,1:3), Fun_qrtdot(qdot_s),Fun_qrsdot(qdot_s),Fun_qrfdot(qdot_s),...
                        Fun_qltdot(qdot_s),Fun_qlsdot(qdot_s),Fun_qlfdot(qdot_s)];
pJoi=Fun_posiJoi(q_s,CON_Len);
pdotJoi=Fun_posidotJoi(q_s,qdot_s,CON_Len);
pddotJoi=Fun_posiddotJoi(q_s,qdot_s,qddot_s,CON_Len);


%% All Info -------------
t_pst=linspace(0,50,length(expdata.t_grid_step)).';

close all;
figure;hold on;
set(gcf,'Position',[100 100 1400 600]);
set(gcf,'defaultLineMarkerSize',2,'defaultLineLineWidth',2);
nr=3;nc=6;

C1= [110 110 110]/3/255;
C2= [192 0 0]/255;
% C3= [79 129 189]/255;
C3= [51 153 255]/255;
C4= [255 102 0]/255;
C1S=[230 230 230]*0.8/255;
C2S=[220 230 255]/255;
C3S=[255 230 230]/255;

plot_bd= 1 ;

lb=Jac_Xnorm*auxdata.lb;
ub=Jac_Xnorm*auxdata.ub;

q_min=reshape(lb(1:N_step*N_q),N_step,[]);
qdot_min=reshape(lb((1:N_step*N_q)+N_step*N_q),N_step,[]);
qddot_min=reshape(lb((1:N_step*N_q)+N_step*N_q*2),N_step,[]);
a_min=reshape(lb((1:N_step*N_u)+N_step*N_q*3),N_step,[]);
u_min=reshape(lb((1:N_step*N_u)+N_step*N_q*3+N_step*N_u),N_step,[]);

q_max=reshape(ub(1:N_step*N_q),N_step,[]);
qdot_max=reshape(ub((1:N_step*N_q)+N_step*N_q),N_step,[]);
qddot_max=reshape(ub((1:N_step*N_q)+N_step*N_q*2),N_step,[]);
a_max=reshape(ub((1:N_step*N_u)+N_step*N_q*3),N_step,[]);
u_max=reshape(ub((1:N_step*N_u)+N_step*N_q*3+N_step*N_u),N_step,[]);

tftsize=14; % title fontsize
%--------------------
subplot(nr,nc,1);hold on;
title('xb');
title('$x_b$','Interpreter','latex','FontSize',tftsize)
if plot_bd
    plot_sd(t_pst,q_min(:,1),q_max(:,1),C1S); end
plot(t_pst,q_MOC(:,1),'Color',C1);
plot(t_pst,q_s(:,1),'-','Color',C2,'DisplayName','rHip');
ylabel('Displacement (m)');
xlim([0 100])
grid minor

%--------------------
subplot(nr,nc,2);hold on;
title('$y_b$','Interpreter','latex','FontSize',tftsize)
if plot_bd
    plot_sd(t_pst,q_min(:,2),q_max(:,2),C1S); end
plot(t_pst,q_MOC(:,2),'Color',C1);
plot(t_pst,q_s(:,2),'-','Color',C2,'DisplayName','d1');
ylabel('Displacement (m)');
xlim([0 100])
ylim([0.7 1.2])
grid minor

%--------------------
subplot(nr,nc,3);hold on;
title('$q_b$','Interpreter','latex','FontSize',tftsize)
if plot_bd
plot_sd(t_pst,rad2deg(q_min(:,3)),rad2deg(q_max(:,3)),C1S); end
plot(t_pst,rad2deg(q_MOC(:,3)),'Color',C1);
plot(t_pst,rad2deg(q_s(:,3)),'-','Color',C2,'DisplayName','q1')
ylabel('Angle (deg.)');
xlim([0 100])
grid minor


%--------------------
subplot(nr,nc,4);hold on;
title('$q_{Hip}$','Interpreter','latex','FontSize',tftsize)
if plot_bd
    plot_sd(t_pst,rad2deg(q_min(:,4)),rad2deg(q_max(:,4)),C1S);
    plot_sd(t_pst+50,rad2deg(q_min(:,7)),rad2deg(q_max(:,7)),C1S); end
plot(t_pst,rad2deg(q_MOC(:,4)),'Color',C1);
plot(t_pst+50,rad2deg(q_MOC(:,7)),'Color',C1);
plot(t_pst,rad2deg(q_s(:,4)),'Color',C3,'DisplayName','rHip');
plot(t_pst+50,rad2deg(q_s(:,7)),'Color',C4,'DisplayName','lHip');
ylabel('Angle (deg.)');
grid minor

%--------------------
subplot(nr,nc,5);hold on;
title('$q_{Knee}$','Interpreter','latex','FontSize',tftsize)
if plot_bd
    plot_sd(t_pst,rad2deg(q_min(:,5)),rad2deg(q_max(:,5)),C1S);
    plot_sd(t_pst+50,rad2deg(q_min(:,8)),rad2deg(q_max(:,8)),C1S); end
plot(t_pst,rad2deg(q_MOC(:,5)),'Color',C1);
plot(t_pst+50,rad2deg(q_MOC(:,8)),'Color',C1);
plot(t_pst,rad2deg(q_s(:,5)),'Color',C3,'DisplayName','d1');
plot(t_pst+50,rad2deg(q_s(:,8)),'Color',C4,'DisplayName','d2');
ylabel('Angle (deg.)');
grid minor

%--------------------
subplot(nr,nc,6);hold on;
title('$q_{Ankle}$','Interpreter','latex','FontSize',tftsize)
if plot_bd
    plot_sd(t_pst,rad2deg(q_min(:,6)),rad2deg(q_max(:,6)),C1S);
    plot_sd(t_pst+50,rad2deg(q_min(:,9)),rad2deg(q_max(:,9)),C1S); end
plot(t_pst,rad2deg(q_MOC(:,6)),'Color',C1);
plot(t_pst+50,rad2deg(q_MOC(:,9)),'Color',C1);
plot(t_pst,rad2deg(q_s(:,6)),'Color',C3,'DisplayName','q1')
plot(t_pst+50,rad2deg(q_s(:,9)),'Color',C4,'DisplayName','q2')
ylabel('Angle (deg.)');
grid minor


%---------------------------------------Velo

subplot(nr,nc,7);hold on;
title('$\dot{x}_b$','Interpreter','latex','FontSize',tftsize)

if plot_bd
    plot_sd(t_pst,qdot_min(:,1),qdot_max(:,1),C1S); end
plot(t_pst,qdot_MOC(:,1),'Color',C1);
plot(t_pst,qdot_s(:,1),'Color',C2,'DisplayName','rHip');
ylabel('Velocity (m)');
xlim([0 100])
grid minor

%--------------------
subplot(nr,nc,8);hold on;
title('$\dot{y}_b$','Interpreter','latex','FontSize',tftsize)
if plot_bd
    plot_sd(t_pst,qdot_min(:,2),qdot_max(:,2),C1S); end
plot(t_pst,qdot_MOC(:,2),'Color',C1);
plot(t_pst,qdot_s(:,2),'Color',C2,'DisplayName','d1');
ylabel('Velocity (m)');
xlim([0 100])
grid minor

%--------------------
subplot(nr,nc,9);hold on;
title('$\dot{q}_b$','Interpreter','latex','FontSize',tftsize)
if plot_bd
    plot_sd(t_pst,rad2deg(qdot_min(:,3)),rad2deg(qdot_max(:,3)),C1S); end
plot(t_pst,rad2deg(qdot_MOC(:,3)),'Color',C1);
plot(t_pst,rad2deg(qdot_s(:,3)),'Color',C2,'DisplayName','q1')
ylabel('Angular Velocity (deg.)');
xlim([0 100])
grid minor


%--------------------
subplot(nr,nc,10);hold on;
title('$\dot{q}_{Hip}$','Interpreter','latex','FontSize',tftsize)
if plot_bd
    plot_sd(t_pst,rad2deg(qdot_min(:,4)),rad2deg(qdot_max(:,4)),C1S);
    plot_sd(t_pst+50,rad2deg(qdot_min(:,7)),rad2deg(qdot_max(:,7)),C1S); end
plot(t_pst,rad2deg(qdot_MOC(:,4)),'Color',C1);
plot(t_pst+50,rad2deg(qdot_MOC(:,7)),'Color',C1);
plot(t_pst,rad2deg(qdot_s(:,4)),'Color',C3,'DisplayName','rHip');
plot(t_pst+50,rad2deg(qdot_s(:,7)),'Color',C4,'DisplayName','lHip');
ylabel('Angular Velocity (deg.)');
grid minor

%--------------------
subplot(nr,nc,11);hold on;
title('$\dot{q}_{Knee}$','Interpreter','latex','FontSize',tftsize)
if plot_bd
    plot_sd(t_pst,rad2deg(qdot_min(:,5)),rad2deg(qdot_max(:,5)),C1S);
    plot_sd(t_pst+50,rad2deg(qdot_min(:,8)),rad2deg(qdot_max(:,8)),C1S); end
plot(t_pst,rad2deg(qdot_MOC(:,5)),'Color',C1);
plot(t_pst+50,rad2deg(qdot_MOC(:,8)),'Color',C1);
plot(t_pst,rad2deg(qdot_s(:,5)),'Color',C3,'DisplayName','d1');
plot(t_pst+50,rad2deg(qdot_s(:,8)),'Color',C4,'DisplayName','d2');
ylabel('Angular Velocity (deg.)');
grid minor

%--------------------
subplot(nr,nc,12);hold on;
title('$\dot{q}_{Ankle}$','Interpreter','latex','FontSize',tftsize)
if plot_bd
    plot_sd(t_pst,rad2deg(qdot_min(:,6)),rad2deg(qdot_max(:,6)),C1S);
    plot_sd(t_pst+50,rad2deg(qdot_min(:,9)),rad2deg(qdot_max(:,9)),C1S); end
plot(t_pst,rad2deg(qdot_MOC(:,6)),'Color',C1);
plot(t_pst+50,rad2deg(qdot_MOC(:,9)),'Color',C1);
plot(t_pst,rad2deg(qdot_s(:,6)),'Color',C3,'DisplayName','q1')
plot(t_pst+50,rad2deg(qdot_s(:,9)),'Color',C4,'DisplayName','q2')
ylabel('Angular Velocity (deg.)');
grid minor



%--------------------
subplot(nr,nc,13);hold on;
title('$F_{gx}$','Interpreter','latex','FontSize',tftsize)
plot(t_pst,Fg_MOC(:,1),'Color',C1);
plot(t_pst+50,Fg_MOC(:,3),'Color',C1);
plot(t_pst,Frgx_s,'Color',C3,'DisplayName','F1')
plot(t_pst+50,Flgx_s,'Color',C4,'DisplayName','F2');
ylabel('Force (N)');
grid minor
xlabel('Gait (%)');

%--------------------
subplot(nr,nc,14);hold on;
title('$F_{gy}$','Interpreter','latex','FontSize',tftsize)

plot(t_pst,Fg_MOC(:,2),'Color',C1,'HandleVisibility','off');
plot(t_pst+50,Fg_MOC(:,4),'Color',C1,'HandleVisibility','off');
plot(t_pst,Frgy_s,'Color',C3,'DisplayName','F1');
plot(t_pst+50,Flgy_s,'Color',C4,'DisplayName','F2');
ylim([-100 max(ylim())]);
xlabel('Gait (%)');
ylabel('Force (N)');
grid minor


subplot(nr,nc,15);hold on;
plot(1,1,'Color',C1);
plot(1,1,'Color',C2);
plot(1,1,'Color',C3);
plot(1,1,'Color',C4);
axis off
legend({'MCS','Trunk','Leg 1','Leg 2'},'FontSize',10,'Box','off')

%--------------------
subplot(nr,nc,16);hold on;
title('$T_{Hip}$','Interpreter','latex','FontSize',tftsize)
if plot_bd
    plot_sd(t_pst,a_min(:,1)*TJmax(1),a_max(:,1)*TJmax(1),C1S);
    plot_sd(t_pst+50,a_min(:,4)*TJmax(4),a_max(:,4)*TJmax(4),C1S); end
plot(t_pst,TJ_MOC(:,1),'Color',C1);
plot(t_pst+50,TJ_MOC(:,4),'Color',C1);
plot(t_pst,TJ_s(:,1),'Color',C3,'DisplayName','T1');
plot(t_pst+50,TJ_s(:,4),'Color',C4,'DisplayName','T3');
ylabel('Torque(N*m)');
grid minor
xlabel('Gait (%)');
%--------------------
subplot(nr,nc,17);hold on;
title('$T_{Knee}$','Interpreter','latex','FontSize',tftsize)
if plot_bd
    plot_sd(t_pst,a_min(:,2)*TJmax(2),a_max(:,2)*TJmax(2),C1S);
    plot_sd(t_pst+50,a_min(:,5)*TJmax(5),a_max(:,5)*TJmax(5),C1S); end
plot(t_pst,TJ_MOC(:,2),'Color',C1);
plot(t_pst+50,TJ_MOC(:,5),'Color',C1);
plot(t_pst,TJ_s(:,2),'Color',C3,'DisplayName','T1');
plot(t_pst+50,TJ_s(:,5),'Color',C4,'DisplayName','T3');
ylabel('Torque(N*m)');
grid minor
xlabel('Gait (%)');
%--------------------
subplot(nr,nc,18);hold on;
title('$T_{Ankle}$','Interpreter','latex','FontSize',tftsize)
if plot_bd
    plot_sd(t_pst,a_min(:,3)*TJmax(3),a_max(:,3)*TJmax(3),C1S);
    plot_sd(t_pst+50,a_min(:,6)*TJmax(6),a_max(:,6)*TJmax(6),C1S); end
plot(t_pst,TJ_MOC(:,3),'Color',C1);
plot(t_pst+50,TJ_MOC(:,6),'Color',C1);
plot(t_pst,TJ_s(:,3),'Color',C3,'DisplayName','F2');
plot(t_pst+50,TJ_s(:,6),'Color',C4,'DisplayName','F4');
ylabel('Torque(N*m)');
grid minor
xlabel('Gait (%)');










return;

%%
xn_s = Fun_xn (q_s,CON_Len);
yn_s = Fun_yn (q_s,CON_Len);
xbm_s = Fun_xbm (q_s,CON_Len);
ybm_s = Fun_ybm (q_s,CON_Len);

xrk_s = Fun_xrk (q_s,CON_Len);
yrk_s = Fun_yrk (q_s,CON_Len);
xra_s = Fun_xra (q_s,CON_Len);
yra_s = Fun_yra (q_s,CON_Len);

xlk_s = Fun_xlk (q_s,CON_Len);
ylk_s = Fun_ylk (q_s,CON_Len);
xla_s = Fun_xla (q_s,CON_Len);
yla_s = Fun_yla (q_s,CON_Len);

xrtm_s = Fun_xrtm (q_s,CON_Len);
yrtm_s = Fun_yrtm (q_s,CON_Len);
xrsm_s = Fun_xrsm (q_s,CON_Len);
yrsm_s = Fun_yrsm (q_s,CON_Len);
xrfm_s = Fun_xrfm (q_s,CON_Len);
yrfm_s = Fun_yrfm (q_s,CON_Len);

xltm_s = Fun_xltm (q_s,CON_Len);
yltm_s = Fun_yltm (q_s,CON_Len);
xlsm_s = Fun_xlsm (q_s,CON_Len);
ylsm_s = Fun_ylsm (q_s,CON_Len);
xlfm_s = Fun_xlfm (q_s,CON_Len);
ylfm_s = Fun_ylfm (q_s,CON_Len);

xrhe_s = Fun_xrhe(q_s,CON_Len);
yrhe_s = Fun_yrhe(q_s,CON_Len);
xrto_s = Fun_xrto(q_s,CON_Len);
yrto_s = Fun_yrto(q_s,CON_Len);

xlhe_s = Fun_xlhe(q_s,CON_Len);
ylhe_s = Fun_ylhe(q_s,CON_Len);
xlto_s = Fun_xlto(q_s,CON_Len);
ylto_s = Fun_ylto(q_s,CON_Len);

xrcts_s = Fun_xrcts(q_s,CON_Len,ctsx,ctsy);
yrcts_s = Fun_yrcts(q_s,CON_Len,ctsx,ctsy);
xlcts_s = Fun_xlcts(q_s,CON_Len,ctsx,ctsy);
ylcts_s = Fun_ylcts(q_s,CON_Len,ctsx,ctsy);

%% for check
figure;hold on
plot(rad2deg(q_s(:,3)));
figure;hold on
plot(q_s(:,1));


%% SnapShot

figure;
set(gcf,'defaultLineMarkerSize',3,'defaultLineLineWidth',3);
set(gcf,'Position',[100 100 1000 350]);
frpool=round(linspace(1,30,10));
Nss=length(frpool);
for i=1:10
    subplot('Position',[(1/(Nss+1)*i-1/15) 0.25 1/(Nss+1) 0.73]);hold on;
    gridi=frpool(i);
    
    xlabel(['t=',num2str(t_grid_step(gridi),'%.02f'),'s'],'FontSize',17)
    
    plot(q_s(gridi,1),q_s(gridi,2),'ko-','MarkerFaceColor','k');
    
    plot(xn_s(gridi),yn_s(gridi)  ,'ko-','MarkerFaceColor','none');
    plot(xrk_s(gridi),yrk_s(gridi),'bo-','MarkerFaceColor','b');
    plot(xra_s(gridi),yra_s(gridi),'bo-','MarkerFaceColor','b');
    
    plot(xlk_s(gridi),ylk_s(gridi),'go-','MarkerFaceColor','g');
    plot(xla_s(gridi),yla_s(gridi),'go-','MarkerFaceColor','g');
    
    
    plot(xbm_s(gridi),ybm_s(gridi),'k*');
    plot(xrtm_s(gridi),yrtm_s(gridi),'b*');
    plot(xrsm_s(gridi),yrsm_s(gridi),'b*');
    plot(xrfm_s(gridi),yrfm_s(gridi),'b*');
    plot(xltm_s(gridi),yltm_s(gridi),'g*');
    plot(xlsm_s(gridi),ylsm_s(gridi),'g*');
    plot(xlfm_s(gridi),ylfm_s(gridi),'g*');
    
    line([q_s(gridi,1),xn_s(gridi)],[q_s(gridi,2),yn_s(gridi)],...
        'Color','r','LineStyle','-','LineWidth',1.5);
    
    line([q_s(gridi,1),xrk_s(gridi)],[q_s(gridi,2),yrk_s(gridi)],...
        'Color','b','LineStyle','-','LineWidth',1.5);
    line([xrk_s(gridi),xra_s(gridi)],[yrk_s(gridi),yra_s(gridi)],...
        'Color','b','LineStyle','--','LineWidth',1.5);
    
    line([xra_s(gridi),xrhe_s(gridi)] ,[yra_s(gridi),yrhe_s(gridi)],...
        'Color','b','LineStyle','-');
    line([xra_s(gridi),xrto_s(gridi)] ,[yra_s(gridi),yrto_s(gridi)],...
        'Color','b','LineStyle','-');
    line([xrhe_s(gridi),xrto_s(gridi)] ,[yrhe_s(gridi),yrto_s(gridi)],...
        'Color','b','LineStyle','-');
    
    
    line([q_s(gridi,1),xlk_s(gridi)],[q_s(gridi,2),ylk_s(gridi)],...
        'Color','g','LineStyle','-','LineWidth',1.5);
    line([xlk_s(gridi),xla_s(gridi)],[ylk_s(gridi),yla_s(gridi)],...
        'Color','g','LineStyle','--','LineWidth',1.5);
    line([xla_s(gridi),xlhe_s(gridi)] ,[yla_s(gridi),ylhe_s(gridi)],...
        'Color','g','LineStyle','--');
    line([xla_s(gridi),xlto_s(gridi)] ,[yla_s(gridi),ylto_s(gridi)],...
        'Color','g','LineStyle','--');
    line([xlhe_s(gridi),xlto_s(gridi)] ,[ylhe_s(gridi),ylto_s(gridi)],...
        'Color','g','LineStyle','--');
    
    
    switch auxdata.contactFlag
        case 1 % 1=vdB
            for cti=1:auxdata.EOM_CONS.N_cts
                plot(xlcts_s(gridi,cti),ylcts_s(gridi,cti),'ko','MarkerFaceColor',[0.3 0.3 0.3])
                plot(xrcts_s(gridi,cti),yrcts_s(gridi,cti),'ko','MarkerFaceColor',[0.3 0.3 0.3])
            end
        case 2  % 2=Sim
            for cti=1:auxdata.EOM_CONS.N_cts
                rectangle('Position',[xlcts_s(gridi,cti)-cts.ctsR(cti),ylcts_s(gridi,cti)-cts.ctsR(cti),2*cts.ctsR(cti),2*cts.ctsR(cti)],'Curvature',[1,1],'FaceColor',[0.1 0.1 0.1 0.3]);
                rectangle('Position',[xrcts_s(gridi,cti)-cts.ctsR(cti),yrcts_s(gridi,cti)-cts.ctsR(cti),2*cts.ctsR(cti),2*cts.ctsR(cti)],'Curvature',[1,1],'FaceColor',[0.1 0.1 0.1 0.3]);
            end
    end
    
    yticklabels({});
    xticklabels({});
    set(gca,'YColor','w');
    
    
    axis equal;
    ylim([-0.2 1.8]);
    %         grid minor
    xlim([q_s(gridi,1)-0.8, q_s(gridi,1)+0.8]);
    line(xlim(),[0 0],'Color','k','LineStyle','--','LineWidth',1.2);
    
end

%%
return;













































