function [obj,grad] = V15_ObjFun(X,auxdata)

%% input parameters
global test_count test_divide plot_flag

%% Extract the constants 
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
X_res=X;
X=Jac_Xnorm*X;
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

%% objective

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

qSeg_s=[q_s(:,1:3), Fun_qrt(q_s),Fun_qrs(q_s),Fun_qrf(q_s),...
                    Fun_qlt(q_s),Fun_qls(q_s),Fun_qlf(q_s)];
qdotSeg_s=[qdot_s(:,1:3), Fun_qrtdot(qdot_s),Fun_qrsdot(qdot_s),Fun_qrfdot(qdot_s),...
                          Fun_qltdot(qdot_s),Fun_qlsdot(qdot_s),Fun_qlfdot(qdot_s)];
pJoi_s=Fun_posiJoi(q_s,CON_Len);
pdotJoi_s=Fun_posidotJoi(q_s,qdot_s,CON_Len);
pddotJoi_s=Fun_posiddotJoi(q_s,qdot_s,qddot_s,CON_Len);

                    
obj=Fun_obj(q_s,qdot_s,qddot_s,a_s,u_s,Fg_s,power_s,qSeg_s,qdotSeg_s,pJoi_s,pdotJoi_s,pddotJoi_s,...
    auxdata.objw_work,auxdata.objw_u2,auxdata.objw_udot,auxdata.objw_Fgdot,auxdata.objw_qdddot,...
    auxdata.objw_qErr,auxdata.objw_qdotErr,auxdata.objw_qddotErr,...
    auxdata.objw_qSegErr,auxdata.objw_qdotSegErr,auxdata.objw_pJoiErr,auxdata.objw_pdotJoiErr,auxdata.objw_pddotJoiErr,...
    q_MOC,qdot_MOC,qddot_MOC,qSeg_track,qdotSeg_track,pJoi_track,pdotJoi_track,pddotJoi_track,...
    Time_step,OSx,OSy,epsl);

%%
if auxdata.ObjGradFlag
    % -------Jac obj w.r.t. var
    
    objJv=Fun_objJv(q_s,qdot_s,qddot_s,a_s,u_s,Fg_s,power_s,qSeg_s,qdotSeg_s,pJoi_s,pdotJoi_s,pddotJoi_s,...
        auxdata.objw_work,auxdata.objw_u2,auxdata.objw_udot,auxdata.objw_Fgdot,auxdata.objw_qdddot,...
        auxdata.objw_qErr,auxdata.objw_qdotErr,auxdata.objw_qddotErr,...
        auxdata.objw_qSegErr,auxdata.objw_qdotSegErr,auxdata.objw_pJoiErr,auxdata.objw_pdotJoiErr,auxdata.objw_pddotJoiErr,...
        q_MOC,qdot_MOC,qddot_MOC,qSeg_track,qdotSeg_track,pJoi_track,pdotJoi_track,pddotJoi_track,...
        Time_step,OSx,OSy,epsl);

    % -------Jac var1 w.r.t. X
    varobj1Jac_X=eye(N_X);

    % -------Jac var2 w.r.t. X
    J_cts_FRx=zeros(N_cts,N_var);
    J_cts_FRy=zeros(N_cts,N_var);
    J_cts_FLx=zeros(N_cts,N_var);
    J_cts_FLy=zeros(N_cts,N_var);
    
    Jac_Frgx=zeros(N_step,N_var);
    Jac_Frgy=zeros(N_step,N_var);
    Jac_Flgx=zeros(N_step,N_var);
    Jac_Flgy=zeros(N_step,N_var);
    
    qJac_X=Grad_CONS.qJac_X;
    qdotJac_X=Grad_CONS.qdotJac_X;
    qddotJac_X=Grad_CONS.qddotJac_X;
    qqdotJac_X=Grad_CONS.qqdotJac_X;
    uJac_X=Grad_CONS.uJac_X;

    Jac_power=zeros(N_step,N_u,N_var);
    Jac_qSeg=zeros(N_step,9,N_var);
    Jac_qdotSeg=zeros(N_step,9,N_var);
    Jac_pJoi=zeros(N_step,12,N_var);
    Jac_pdotJoi=zeros(N_step,12,N_var);
    Jac_pddotJoi=zeros(N_step,12,N_var);
    
    aJac_X=Grad_CONS.aJac_X;
    
    for ni=1:N_step
        
        
        J_cts_FRy_p=Fun_Contact_Sim_JT_Fy(ctsR,ctsk,ctsc,           yrcts(ni,:),yrctsdot(ni,:)).';
        J_cts_FRx_p=Fun_Contact_Sim_JT_Fx(ctsR,ctsk,ctsc,ctsud,ctsV,yrcts(ni,:),yrctsdot(ni,:),xrctsdot(ni,:)).';
        J_cts_FLy_p=Fun_Contact_Sim_JT_Fy(ctsR,ctsk,ctsc,           ylcts(ni,:),ylctsdot(ni,:)).';
        J_cts_FLx_p=Fun_Contact_Sim_JT_Fx(ctsR,ctsk,ctsc,ctsud,ctsV,ylcts(ni,:),ylctsdot(ni,:),xlctsdot(ni,:)).';
        
        J_ylcts   =Fun_J_ylcts(q_s(ni,:),CON_Len,ctsx,ctsy);
        J_xlctsdot=Fun_J_xlctsdot(q_s(ni,:),qdot_s(ni,:),CON_Len,ctsx,ctsy);
        J_ylctsdot=Fun_J_ylctsdot(q_s(ni,:),qdot_s(ni,:),CON_Len,ctsx,ctsy);
        
        J_yrcts   =Fun_J_yrcts(q_s(ni,:),CON_Len,ctsx,ctsy);
        J_xrctsdot=Fun_J_xrctsdot(q_s(ni,:),qdot_s(ni,:),CON_Len,ctsx,ctsy);
        J_yrctsdot=Fun_J_yrctsdot(q_s(ni,:),qdot_s(ni,:),CON_Len,ctsx,ctsy);
        
        for i=1:N_cts
            J_cts_FRx(i,:) =J_cts_FRx_p(i,:)*[J_yrcts(i,:)*qJac_X;J_yrctsdot(i,:)*qqdotJac_X;J_xrctsdot(i,:)*qqdotJac_X];
            J_cts_FRy(i,:) =J_cts_FRy_p(i,:)*[J_yrcts(i,:)*qJac_X;J_yrctsdot(i,:)*qqdotJac_X];
            J_cts_FLx(i,:) =J_cts_FLx_p(i,:)*[J_ylcts(i,:)*qJac_X;J_ylctsdot(i,:)*qqdotJac_X;J_xlctsdot(i,:)*qqdotJac_X];
            J_cts_FLy(i,:) =J_cts_FLy_p(i,:)*[J_ylcts(i,:)*qJac_X;J_ylctsdot(i,:)*qqdotJac_X];
        end
        Jac_Frgx(ni,:)=ones(1,N_cts)*J_cts_FRx;
        Jac_Frgy(ni,:)=ones(1,N_cts)*J_cts_FRy;
        Jac_Flgx(ni,:)=ones(1,N_cts)*J_cts_FLx;
        Jac_Flgy(ni,:)=ones(1,N_cts)*J_cts_FLy;
        
        J_T_Joi=Fun_J_T_Joi(TJmax);
        
        Jac_power(ni,:,:)=Fun_J_PJoi(qdot_s(ni,:),TJ_s(ni,:))*[qdotJac_X;J_T_Joi*aJac_X];
        
        Jac_qSeg(ni,:,:)=[qJac_X(1:3,:);Fun_J_qSeg(q_s(ni,:))*[qJac_X]];
        Jac_qdotSeg(ni,:,:)=[qdotJac_X(1:3,:);Fun_J_qdotSeg(qdot_s(ni,:))*[qJac_X;qdotJac_X]];
        
        Jac_pJoi(ni,:,:)=Fun_J_posiJoi(q_s(ni,:),CON_Len)*[qJac_X];
        Jac_pdotJoi(ni,:,:)=Fun_J_posidotJoi(q_s(ni,:),qdot_s(ni,:),CON_Len)*[qJac_X;qdotJac_X];
        Jac_pddotJoi(ni,:,:)=Fun_J_posiddotJoi(q_s(ni,:),qdot_s(ni,:),qddot_s(ni,:),CON_Len)*[qJac_X;qdotJac_X;qddotJac_X];
        
    end
    
    Jac_power_step=reshape(Jac_power,[],N_var);
    Jac_qSeg_step=reshape(Jac_qSeg,[],N_var);
    Jac_pJoi_step=reshape(Jac_pJoi,[],N_var);
    
    varobj2Jac_s=[Jac_power_step;Jac_qSeg_step;Jac_pJoi_step];
    [vobj2J_nzrow,vobj2J_nzcol,vobj2J_nzval]=find(varobj2Jac_s);
    vobj2J_nzcol_X=(vobj2J_nzcol-1)*(N_step)+mod(vobj2J_nzrow-1,N_step)+1;
    varobj2Jac_X=sparse(vobj2J_nzrow,vobj2J_nzcol_X,vobj2J_nzval,size(varobj2Jac_s,1),N_X);
    
    % -------Jac obj w.r.t. X
    varobjJac_X=[varobj1Jac_X;varobj2Jac_X ];
    
    grad=objJv*varobjJac_X;
    grad=grad*Jac_Xnorm;
    
    if strcmp(auxdata.solver,'fmincon')
        grad=grad.';
    end
else
    grad=0;
end

%% Plot to see the temp results--------------------------
test_count=test_count+1;

if mod(test_count,test_divide)==0 || plot_flag==10
    
    %% ------------------------
    % Timing
    t_grid_step = linspace(0,Time_step,N_step).';
    ccnow=0;
    xb_s  = X(ccnow+1:ccnow+N_step); ccnow=ccnow+N_step;
    yb_s  = X(ccnow+1:ccnow+N_step); ccnow=ccnow+N_step;
    
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
    
    power_s_abs=sqrt(power_s.^2+epsl^2)-epsl;
    work_sum=TLtrapz(t_grid_step,power_s_abs);
    obj_work=work_sum * auxdata.objw_work;
    
    u2_sum=TLtrapz(t_grid_step,u_s.^2);
    obj_u2=u2_sum*auxdata.objw_u2;
    
    tdot_s=t_grid_step(2:end)-t_grid_step(1:end-1);
    
    udot_s=TLdiff(u_s,t_grid_step,0);
    udot_sum=sum(udot_s.^2.*(tdot_s*ones(1,N_u)));
    obj_udot=udot_sum*auxdata.objw_udot;
    
    qdddot_s=TLdiff(qddot_s,t_grid_step,0);
    qdddot_sum=sum(qdddot_s.^2.*(tdot_s*ones(1,N_q)));
    obj_qdddot=qdddot_sum*auxdata.objw_qdddot;
    
    Fgdot_s=TLdiff(Fg_s,t_grid_step,0);
    Fgdot_sum=sum(Fgdot_s.^2.*(tdot_s*ones(1,N_g)));
    obj_Fgdot=Fgdot_sum*auxdata.objw_Fgdot;


    qErr_sum=TLtrapz(t_grid_step,(q_s-q_MOC).^2);
    qdotErr_sum=TLtrapz(t_grid_step,(qdot_s-qdot_MOC).^2);
    qddotErr_sum=TLtrapz(t_grid_step,(qddot_s-qddot_MOC).^2);
    
    pJoi_exp_calobj=pJoi_track;
    pJoi_exp_calobj(:,11)=pJoi_exp_calobj(:,11)+OSx; % lax
    pJoi_exp_calobj(:,12)=pJoi_exp_calobj(:,12)+OSy; % lay
    pJoi_exp_calobj(:,6)=pJoi_exp_calobj(:,6)+OSy;   % ray

    qSegErr_sum    =TLtrapz(t_grid_step,(qSeg_s    -qSeg_track  ).^2);
    qdotSegErr_sum =TLtrapz(t_grid_step,(qdotSeg_s -qdotSeg_track ).^2);
    pJoiErr_sum    =TLtrapz(t_grid_step,(pJoi_s    -pJoi_exp_calobj    ).^2);
    pdotJoiErr_sum =TLtrapz(t_grid_step,(pdotJoi_s -pdotJoi_track ).^2);
    pddotJoiErr_sum =TLtrapz(t_grid_step,(pddotJoi_s -pddotJoi_track ).^2);
    
    obj_qErr    = qErr_sum    * auxdata.objw_qErr;
    obj_qdotErr = qdotErr_sum * auxdata.objw_qdotErr;
    obj_qddotErr= qddotErr_sum* auxdata.objw_qddotErr;
    obj_qSegErr    = qSegErr_sum    * auxdata.objw_qSegErr;
    obj_qdotSegErr = qdotSegErr_sum * auxdata.objw_qdotSegErr;
    obj_pJoiErr    = pJoiErr_sum    * auxdata.objw_pJoiErr;
    obj_pdotJoiErr = pdotJoiErr_sum * auxdata.objw_pdotJoiErr;
    obj_pddotJoiErr = pddotJoiErr_sum * auxdata.objw_pddotJoiErr;
    
    obj=obj_qErr+obj_qdotErr+obj_qddotErr+...
        obj_qSegErr + obj_qdotSegErr + ...
        obj_pJoiErr + obj_pdotJoiErr + obj_pddotJoiErr + ...
        (obj_work + obj_u2 + obj_udot + obj_qdddot + obj_Fgdot);

    Err.Err_q=q_s-q_MOC;
    Err.Err_qdot=qdot_s-qdot_MOC;
    Err.Err_qddot=qddot_s-qddot_MOC;
    Err.Err_TJ=TJ_s-TJ_MOC; 
    Err.Err_GRF=Fg_s-Fg_MOC; 
    Err.Err_qSeg_track=qSeg_s-qSeg_track;
    Err.Err_qdotSeg_track=qdotSeg_s -qdotSeg_track;
    Err.Err_pJoi_track=pJoi_s-pJoi_exp_calobj;
    Err.Err_pdotJoi_track=pdotJoi_s-pdotJoi_track;
    Err.Err_pddotJoi_track=pddotJoi_s-pddotJoi_track;
    
    Err.Err_qSeg=qSeg_s-qSeg_MOC;
    Err.Err_qdotSeg=qdotSeg_s-qdotSeg_MOC;
    Err.Err_pJoi=pJoi_s-pJoi_MOC;
    Err.Err_pdotJoi=pdotJoi_s-pdotJoi_MOC;
    Err.Err_pddotJoi=pddotJoi_s-pddotJoi_MOC;
    
    
    Err.RMSE_q=rms(Err.Err_q);
    Err.RMSE_qdot=rms(Err.Err_qdot);
    Err.RMSE_qddot=rms(Err.Err_qddot);
    Err.RMSE_TJ=rms(Err.Err_TJ);
    Err.RMSE_GRF=rms(Err.Err_GRF);
    
    Err.RMSE_qSeg=rms(Err.Err_qSeg);
    Err.RMSE_qdotSeg=rms(Err.Err_qdotSeg);
    Err.RMSE_pJoi=rms(Err.Err_pJoi);
    Err.RMSE_pdotJoi=rms(Err.Err_pdotJoi);
    Err.RMSE_pddotJoi=rms(Err.Err_pddotJoi);
    
    
    StepL_MOC=pJoi_MOC(end,11)-pJoi_MOC(1,5);
    StepL_Model=pJoi_s(end,11)-pJoi_s(1,5);
    StepL_Err=StepL_Model-StepL_MOC;
    StepL_ErrPst=StepL_Err/StepL_MOC;
    
end

if mod(test_count,test_divide)==0
    %% extract bounds
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
    
    %% Main Fig, ALL info-------------
    close all;
    figure;hold on;

    set(gcf,'Position',[100 100 1300 700]);
    set(gcf,'defaultLineMarkerSize',1,'defaultLineLineWidth',1.5);
    plot_bd= 1 ;
    
    C1= [110 110 110]/255;
    C2= [79 129 189]/255;
    C3= [255 48 48]/255;
    C1S=[230 230 230]*0.8/255;
    C2S=[220 230 255]/255;
    C3S=[255 230 230]/255;
    %--------- 1 --------
    subplot(5,5,1);hold on;
    title('xb,yb');
    if plot_bd
    plot_sd(t_grid_step,q_min(:,1),q_max(:,1),C1S);
    plot_sd(t_grid_step,q_min(:,2),q_max(:,2),C1S); end
    plot(t_grid_step,q_MOC(:,1),'k-','HandleVisibility','off');
    plot(t_grid_step,q_MOC(:,2),'k-','HandleVisibility','off');
    plot(t_grid_step,q_s(:,1),'co-','DisplayName','x');
    plot(t_grid_step,q_s(:,2),'mo-','DisplayName','y');
    legend({'x','y'},'FontSize',6)
    xlim([0 1])
    grid minor
    
    %--------- 2 --------
    subplot(5,5,2);hold on;
    title('xb,yb Velo');
    if plot_bd
    plot_sd(t_grid_step,qdot_min(:,1),qdot_max(:,1),C1S);
    plot_sd(t_grid_step,qdot_min(:,2),qdot_max(:,2),C1S); end
    plot(t_grid_step,qdot_MOC(:,1),'k-','HandleVisibility','off');
    plot(t_grid_step,qdot_MOC(:,2),'k-','HandleVisibility','off');
    plot(t_grid_step,qdot_s(:,1),'co-','DisplayName','x');
    plot(t_grid_step,qdot_s(:,2),'mo-','DisplayName','y');
    xlim([0 1])
    grid minor

    %--------- 3 --------
    subplot(5,5,3);hold on;
    title('xb,yb Acce');
    if plot_bd
    plot_sd(t_grid_step,qddot_min(:,1),qddot_max(:,1),C1S);
    plot_sd(t_grid_step,qddot_min(:,2),qddot_max(:,2),C1S); end
    plot(t_grid_step,qddot_MOC(:,1),'k-');
    plot(t_grid_step,qddot_MOC(:,2),'k-');
    plot(t_grid_step,qddot_s(:,1),'co-','DisplayName','x');
    plot(t_grid_step,qddot_s(:,2),'mo-','DisplayName','y');
    xlim([0 1])
    grid minor
    
    %--------- 4 --------
    subplot(5,5,4);hold on;
    title('Fgx');
    plot(t_grid_step,Fg_MOC(:,1),'k-');
    plot(t_grid_step+Time_step,Fg_MOC(:,3),'k-');
    plot(t_grid_step,Frgx_s,'bo-','DisplayName','F1')
    plot(t_grid_step+Time_step,Flgx_s,'g*-','DisplayName','F2');
    grid minor
    
    %--------- 5 --------
    subplot(5,5,5);hold on;
    title('Fgy');
    plot(t_grid_step,Fg_MOC(:,2),'k-','HandleVisibility','off');
    plot(t_grid_step+Time_step,Fg_MOC(:,4),'k-','HandleVisibility','off');
    plot(t_grid_step,Frgy_s,'bo-','DisplayName','F1');
    plot(t_grid_step+Time_step,Flgy_s,'g*-','DisplayName','F2');
    
    ylim([-100 max(ylim())]);
    
    grid minor
    
    %--------- 6 --------
    subplot(5,5,6);hold on;
    title('Hip');
    if plot_bd
    plot_sd(t_grid_step,rad2deg(q_min(:,4)),rad2deg(q_max(:,4)),C1S);
    plot_sd(t_grid_step+Time_step,rad2deg(q_min(:,7)),rad2deg(q_max(:,7)),C1S); end
    plot(t_grid_step,rad2deg(q_MOC(:,4)),'k-');
    plot(t_grid_step+Time_step,rad2deg(q_MOC(:,7)),'k-');
    plot(t_grid_step,rad2deg(q_s(:,4)),'bo-','DisplayName','rHip');
    plot(t_grid_step+Time_step,rad2deg(q_s(:,7)),'g*-','DisplayName','lHip');
    
    grid minor
    
    %--------- 7 --------
    subplot(5,5,7);hold on;
    title('Hip Velo');
    if plot_bd
    plot_sd(t_grid_step,qdot_min(:,4),qdot_max(:,4),C1S);
    plot_sd(t_grid_step+Time_step,qdot_min(:,7),qdot_max(:,7),C1S); end
    plot(t_grid_step,qdot_MOC(:,4),'k-');
    plot(t_grid_step+Time_step,qdot_MOC(:,7),'k-');
    plot(t_grid_step,qdot_s(:,4),'bo-','DisplayName','d1');
    plot(t_grid_step+Time_step,qdot_s(:,7),'g*-','DisplayName','d2');
    grid minor    
    
    %--------- 8 --------
    subplot(5,5,8);hold on;
    title('Hip Acce');
    if plot_bd
    plot_sd(t_grid_step,qddot_min(:,4),qddot_max(:,4),C1S);
    plot_sd(t_grid_step+Time_step,qddot_min(:,7),qddot_max(:,7),C1S); end
    plot(t_grid_step,qddot_MOC(:,4),'k-');
    plot(t_grid_step+Time_step,qddot_MOC(:,7),'k-');
    plot(t_grid_step,qddot_s(:,4),'bo-','DisplayName','d1');
    plot(t_grid_step+Time_step,qddot_s(:,7),'g*-','DisplayName','d2');
    grid minor 
    
    %--------- 9 --------
    subplot(5,5,9);hold on;  
    title('Hip Torqe');
    if plot_bd
    plot_sd(t_grid_step,a_min(:,1)*TJmax(1),a_max(:,1)*TJmax(1),C1S);
    plot_sd(t_grid_step+Time_step,a_min(:,4)*TJmax(4),a_max(:,4)*TJmax(4),C1S); end
    plot(t_grid_step,TJ_MOC(:,1),'k-','DisplayName','T1');
    plot(t_grid_step+Time_step,TJ_MOC(:,4),'k-','DisplayName','T3');
    plot(t_grid_step,TJ_s(:,1),'bo-','DisplayName','T1');
    plot(t_grid_step+Time_step,TJ_s(:,4),'g*-','DisplayName','T3');
    grid minor
    
    %--------- 10 --------  
    subplot(5,5,10);hold on;  
    title('Hip Power');
    plot(t_grid_step,power_s(:,1),'bo-','DisplayName','Trh');
    plot(t_grid_step+Time_step,power_s(:,4),'g*-','DisplayName','Tlh');
    grid minor

    %--------- 11 --------
    subplot(5,5,11);hold on;
    title('Knee');
    if plot_bd
    plot_sd(t_grid_step,rad2deg(q_min(:,5)),rad2deg(q_max(:,5)),C1S);
    plot_sd(t_grid_step+Time_step,rad2deg(q_min(:,8)),rad2deg(q_max(:,8)),C1S); end
    plot(t_grid_step,rad2deg(q_MOC(:,5)),'k-');
    plot(t_grid_step+Time_step,rad2deg(q_MOC(:,8)),'k-');
    plot(t_grid_step,rad2deg(q_s(:,5)),'bo-','DisplayName','d1');
    plot(t_grid_step+Time_step,rad2deg(q_s(:,8)),'g*-','DisplayName','d2');
    grid minor
    
    %--------- 12 --------
    subplot(5,5,12);hold on;
    
    title('Knee Velo');
    if plot_bd
    plot_sd(t_grid_step,qdot_min(:,5),qdot_max(:,5),C1S);
    plot_sd(t_grid_step+Time_step,qdot_min(:,8),qdot_max(:,8),C1S); end
    plot(t_grid_step,qdot_MOC(:,5),'k-');
    plot(t_grid_step+Time_step,qdot_MOC(:,8),'k-');
    plot(t_grid_step,qdot_s(:,5),'bo-','DisplayName','d1');
    plot(t_grid_step+Time_step,qdot_s(:,8),'g*-','DisplayName','d2');
    grid minor    
    %--------- 13 --------
    subplot(5,5,13);hold on;
    
    title('Knee Acce');
    if plot_bd
    plot_sd(t_grid_step,qddot_min(:,5),qddot_max(:,5),C1S);
    plot_sd(t_grid_step+Time_step,qddot_min(:,8),qddot_max(:,8),C1S); end
    plot(t_grid_step,qddot_MOC(:,5),'k-');
    plot(t_grid_step+Time_step,qddot_MOC(:,8),'k-');
    plot(t_grid_step,qddot_s(:,5),'bo-','DisplayName','d1');
    plot(t_grid_step+Time_step,qddot_s(:,8),'g*-','DisplayName','d2');
    grid minor  
    
    %--------- 14 --------
    subplot(5,5,14);hold on;
    title('Knee Torqe');
    if plot_bd
    plot_sd(t_grid_step,a_min(:,2)*TJmax(2),a_max(:,2)*TJmax(2),C1S);
    plot_sd(t_grid_step+Time_step,a_min(:,5)*TJmax(5),a_max(:,5)*TJmax(5),C1S); end
    plot(t_grid_step,TJ_MOC(:,2),'k-','DisplayName','T1');
    plot(t_grid_step+Time_step,TJ_MOC(:,5),'k-','DisplayName','T3');
    plot(t_grid_step,TJ_s(:,2),'bo-','DisplayName','T1');
    plot(t_grid_step+Time_step,TJ_s(:,5),'g*-','DisplayName','T3');

    grid minor
	 
    %--------- 15 --------
    subplot(5,5,15);hold on;
    title('Knee Power');
    plot(t_grid_step,power_s(:,2),'bo-','DisplayName','Trh');
    plot(t_grid_step+Time_step,power_s(:,5),'g*-','DisplayName','Tlh');
    grid minor
    
    %--------- 16 --------   
    subplot(5,5,16);hold on;    
    
    title('Ankle');
    if plot_bd
    plot_sd(t_grid_step,rad2deg(q_min(:,6)),rad2deg(q_max(:,6)),C1S);
    plot_sd(t_grid_step+Time_step,rad2deg(q_min(:,9)),rad2deg(q_max(:,9)),C1S); end
    plot(t_grid_step,rad2deg(q_MOC(:,6)),'k-');
    plot(t_grid_step+Time_step,rad2deg(q_MOC(:,9)),'k-');
    plot(t_grid_step,rad2deg(q_s(:,6)),'bo-','DisplayName','q1')
    plot(t_grid_step+Time_step,rad2deg(q_s(:,9)),'g*-','DisplayName','q2')
    grid minor
    
    
    %--------- 17 --------
    subplot(5,5,17);hold on;  
    
    title('Ankle Velo');
    if plot_bd
    plot_sd(t_grid_step,qdot_min(:,6),qdot_max(:,6),C1S);
    plot_sd(t_grid_step+Time_step,qdot_min(:,9),qdot_max(:,9),C1S); end
    plot(t_grid_step,qdot_MOC(:,6),'k-');
    plot(t_grid_step+Time_step,qdot_MOC(:,9),'k-');
    plot(t_grid_step,qdot_s(:,6),'bo-','DisplayName','q1');
    plot(t_grid_step+Time_step,qdot_s(:,9),'g*-','DisplayName','q2');
    grid minor    
    
    %--------- 18 --------
    subplot(5,5,18);hold on;
    title('Ankle Acce');
    if plot_bd
    plot_sd(t_grid_step,qddot_min(:,6),qddot_max(:,6),C1S);
    plot_sd(t_grid_step+Time_step,qddot_min(:,9),qddot_max(:,9),C1S); end
    plot(t_grid_step,qddot_MOC(:,6),'k-');
    plot(t_grid_step+Time_step,qddot_MOC(:,9),'k-');
    plot(t_grid_step,qddot_s(:,6),'bo-','DisplayName','d1');
    plot(t_grid_step+Time_step,qddot_s(:,9),'g*-','DisplayName','d2');
    grid minor    
	
    %--------- 19 --------
    subplot(5,5,19);hold on;
    title('Ankle Torqe');
    if plot_bd
    plot_sd(t_grid_step,a_min(:,3)*TJmax(3),a_max(:,3)*TJmax(3),C1S);
    plot_sd(t_grid_step+Time_step,a_min(:,6)*TJmax(6),a_max(:,6)*TJmax(6),C1S); end
    plot(t_grid_step,TJ_MOC(:,3),'k-','DisplayName','T1');
    plot(t_grid_step+Time_step,TJ_MOC(:,6),'k-','DisplayName','T3');
    plot(t_grid_step,TJ_s(:,3),'bo-','DisplayName','F2');    
    plot(t_grid_step+Time_step,TJ_s(:,6),'g*-','DisplayName','F4');
    
    grid minor
    
    %--------- 20 --------
    subplot(5,5,20);hold on;
    title('Ankle Power');
    plot(t_grid_step,power_s(:,3),'bo-','DisplayName','Trh');
    plot(t_grid_step+Time_step,power_s(:,6),'g*-','DisplayName','Tlh');
    grid minor
    
    
    %--------- 21 --------   
    subplot(5,5,21);hold on;  
    title('Trunk');
    if plot_bd
    plot_sd(t_grid_step,rad2deg(q_min(:,3)),rad2deg(q_max(:,3)),C1S); end
    plot(t_grid_step,rad2deg(q_MOC(:,3)),'k-');
    plot(t_grid_step,rad2deg(q_s(:,3)),'ro-','DisplayName','q1')
    xlim([0 1])
    grid minor
    %--------- 22 --------
    subplot(5,5,22);hold on;
    
    title('Trunk Velo');
    if plot_bd
    plot_sd(t_grid_step,qdot_min(:,3),qdot_max(:,3),C1S); end
    plot(t_grid_step,qdot_MOC(:,3),'k-');
    plot(t_grid_step,qdot_s(:,3),'ro-','DisplayName','q1');
    xlim([0 1])
    %--------- 23 --------
    subplot(5,5,23);hold on;
	
    title('Trunk Acce');
    if plot_bd
    plot_sd(t_grid_step,qddot_min(:,3),qddot_max(:,3),C1S); end
    plot(t_grid_step,qddot_MOC(:,3),'k-');
    plot(t_grid_step,qddot_s(:,3),'ro-','DisplayName','q1');
    xlim([0 1])
    %--------- 24 --------
    subplot(5,5,24);hold on;
    
    %--------- 25 --------
    subplot(5,5,25);hold on;
    plot(1,1,'b-');
    plot(1,1,'g-');
    legend({'Right','Left'},'FontSize',9)
    
    %% --------- Snapshot all -------
    figure;hold on;
    set(gcf,'Position',[1100 100 300 300]);
    set(gcf,'defaultLineMarkerSize',2,'defaultLineLineWidth',1.5);
    title('x-y');
    plot(xb_s,yb_s,'ko-','MarkerFaceColor','k');
    
    plot(xn_s,yn_s  ,'ko-','MarkerFaceColor','k');
    plot(xrk_s,yrk_s,'bo-','MarkerFaceColor','b');
    plot(xra_s,yra_s,'bo-','MarkerFaceColor','b');
	
    plot(xlk_s,ylk_s,'go-','MarkerFaceColor','g');
    plot(xla_s,yla_s,'go-','MarkerFaceColor','g');

    plot(xbm_s,ybm_s,'k*');
    plot(xrtm_s,yrtm_s,'b*');
    plot(xrsm_s,yrsm_s,'b*');
    plot(xrfm_s,yrfm_s,'b*');
    plot(xltm_s,yltm_s,'g*');
    plot(xlsm_s,ylsm_s,'g*');
    plot(xlfm_s,ylfm_s,'g*');
    
    for i=[1,N_step]
%    for i=1:N_s
        line([xb_s(i),xn_s(i)],[yb_s(i),yn_s(i)],...
            'Color','r','LineStyle','-','LineWidth',1.5);
        
        line([xb_s(i),xrk_s(i)],[yb_s(i),yrk_s(i)],...
            'Color','b','LineStyle','-','LineWidth',1.5);
        line([xrk_s(i),xra_s(i)],[yrk_s(i),yra_s(i)],...
            'Color','b','LineStyle','--','LineWidth',1.5);
        
        line([xra_s(i),xrhe_s(i)] ,[yra_s(i),yrhe_s(i)],...
            'Color','b','LineStyle','-');
        line([xra_s(i),xrto_s(i)] ,[yra_s(i),yrto_s(i)],...
            'Color','b','LineStyle','-');
        line([xrhe_s(i),xrto_s(i)] ,[yrhe_s(i),yrto_s(i)],...
            'Color','b','LineStyle','-');
        
        
        line([xb_s(i),xlk_s(i)],[yb_s(i),ylk_s(i)],...
            'Color','g','LineStyle','-','LineWidth',1.5);
        line([xlk_s(i),xla_s(i)],[ylk_s(i),yla_s(i)],...
            'Color','g','LineStyle','--','LineWidth',1.5);
        line([xla_s(i),xlhe_s(i)] ,[yla_s(i),ylhe_s(i)],...
            'Color','g','LineStyle','--');
        line([xla_s(i),xlto_s(i)] ,[yla_s(i),ylto_s(i)],...
            'Color','g','LineStyle','--');
        line([xlhe_s(i),xlto_s(i)] ,[ylhe_s(i),ylto_s(i)],...
            'Color','g','LineStyle','--');

        for f1cti=1:N_cts
            rectangle('Position',[xlcts_s(i,f1cti)-cts.ctsR(f1cti),ylcts_s(i,f1cti)-cts.ctsR(f1cti),2*cts.ctsR(f1cti),2*cts.ctsR(f1cti)],'Curvature',[1,1],'FaceColor',[0.1 0.1 0.1 0.3]);
            rectangle('Position',[xrcts_s(i,f1cti)-cts.ctsR(f1cti),yrcts_s(i,f1cti)-cts.ctsR(f1cti),2*cts.ctsR(f1cti),2*cts.ctsR(f1cti)],'Curvature',[1,1],'FaceColor',[0.1 0.1 0.1 0.3]);
        end

    end
    line(xlim(),[0 0],'Color','k','LineStyle','--','LineWidth',1.2);
    axis equal;grid minor
    
    %% Save Temp results----------------
    pause(0.1);
    
    if(test_count>test_divide)
        save([char(auxdata.t_start),'x.mat'],'X','auxdata','obj',...
            'test_count','test_divide','plot_flag');
    end

end

%% visual resutls in animation 

if plot_flag==10
    
    Result.auxdata=auxdata;
    Result.X_res=X_res;

    Result.Time_step=Time_step;
    Result.q_s=q_s;
    Result.qdot_s=qdot_s;
    Result.qddot_s=qddot_s;
    Result.u_s=u_s;
    Result.a_s=a_s;
    Result.TJ_s=TJ_s;
    Result.Fg_s=Fg_s;
    Result.power_s=power_s;
    Result.qSeg=qSeg_s;    
    
    Result.work_sum=work_sum;
    Result.cost_Effort=u2_sum;
    Result.udot_sum=udot_sum;
    Result.Fgdot_sum=Fgdot_sum;
    Result.qdddot_sum=qdddot_sum;
    
    Result.qErr_sum=qErr_sum;
    Result.qdotErr_sum=qdotErr_sum;
    Result.qddotErr_sum=qddotErr_sum;
    Result.qSegErr_sum=qSegErr_sum;
    Result.qdotSegErr_sum=qdotSegErr_sum;
    Result.pJoiErr_sum=pJoiErr_sum;
    Result.pdotJoiErr_sum=pdotJoiErr_sum;
    Result.pddotJoiErr_sum=pddotJoiErr_sum;
    
    
    Result.obj_work=obj_work;
    Result.obj_Effort=obj_u2;
    Result.obj_qdddot=obj_qdddot;
    Result.obj_udot=obj_udot;
    Result.obj_Fgdot=obj_Fgdot;
    
    Result.obj_qErr=obj_qErr;
    Result.obj_qdotErr=obj_qdotErr;
    Result.obj_qddotErr=obj_qddotErr;
    Result.obj_qSegErr=obj_qSegErr;
    Result.obj_qdotSegErr=obj_qdotSegErr;
    Result.obj_pJoiErr=obj_pJoiErr;
    Result.obj_pdotJoiErr=obj_pdotJoiErr;
    Result.obj_pddotJoiErr=obj_pddotJoiErr;
    
    Result.obj=obj;
    
    Result.Err=Err;
    Result.StepL_MOC=StepL_MOC;
    Result.StepL_Model=StepL_Model;
    Result.StepL_Err=StepL_Err;
    Result.StepL_ErrPst=StepL_ErrPst;
    
    obj=Result;
    
end

end
















