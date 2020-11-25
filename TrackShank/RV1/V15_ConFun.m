function [nlcneq,nlceq,nlcneqJX,nlceqJX] = V15_ConFun(X,auxdata)

%% input parameters
if 0
    %%
    X=X0;
end

%% Extract the constants 
N_step    = auxdata.N_step;

cts       = auxdata.cts;

TSP_exp      = auxdata.TSP_exp;

Fac_Xnorm    = auxdata.Fac_Xnorm;

Jac_Xnorm = auxdata.Jac_Xnorm;
EOM_CONS  = auxdata.EOM_CONS;
Grad_CONS = auxdata.Grad_CONS;

N_X       = Grad_CONS.N_X;
N_var     = Grad_CONS.N_var;

N_q       = Grad_CONS.N_q;
N_g       = Grad_CONS.N_g;
N_u       = Grad_CONS.N_u;
N_cts     = EOM_CONS.N_cts;

CON_Mass=auxdata.CON_Mass;
CON_Len=auxdata.CON_Len;

%% Extract initial guess to optimize
X=Jac_Xnorm*X;
ccnow=0;
q_s        = reshape(X(ccnow+1:ccnow+N_step*N_q),[],N_q); ccnow=ccnow+N_step*N_q;
qdot_s     = reshape(X(ccnow+1:ccnow+N_step*N_q),[],N_q); ccnow=ccnow+N_step*N_q;
qddot_s     = reshape(X(ccnow+1:ccnow+N_step*N_q),[],N_q); ccnow=ccnow+N_step*N_q;
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

TJmax     = auxdata.TJmax;
TJ_s=Fun_T_Joi(a_s,TJmax);
c_exc=auxdata.c_exc;
adot_s=Fun_adot_Joi(a_s,u_s,c_exc);

qSeg=[q_s(:,1:3), Fun_qrt(q_s),Fun_qrs(q_s),Fun_qrf(q_s),...
                  Fun_qlt(q_s),Fun_qls(q_s),Fun_qlf(q_s)];

%% Constraints

Eq_s=zeros(N_step-1,N_q);

for ni=1:N_step-1
   
    Eq_j=Fun_Eq(CON_Mass,CON_Len,ctsx,ctsy,q_s(ni,:),qdot_s(ni,:),qddot_s(ni,:),TJ_s(ni,:),...
                cts_FRx(ni,:),cts_FRy(ni,:),cts_FLx(ni,:),cts_FLy(ni,:));
    
    Eq_s(ni,:)=Eq_j;
end

nlceq = Fun_nlceq(q_s,qdot_s,qddot_s,a_s,u_s,Eq_s,adot_s,Time_step,Fac_Xnorm,...
    auxdata.nlceqw_qdot,auxdata.nlceqw_qddot,auxdata.nlceqw_Eq,auxdata.nlceqw_adot);

nlcneq = Fun_nlcneq(Fg_s,auxdata.nlcneqw_Fg,auxdata.Fgy_HSmax,auxdata.Fgy_HSmin,...
    auxdata.Fgyperimax,auxdata.Fgxperimax,auxdata.Flgyswingmax,...
    auxdata.nlcneqw_qSeg,auxdata.delta_qSeg_perimin,auxdata.delta_qSeg_perimax,qSeg);


%% Jacobian
if auxdata.ConGradFlag
    
    JacEq=zeros(N_step-1,N_q*  1,N_var);
    
    J_cts_FRx=zeros(N_cts,N_var);
    J_cts_FRy=zeros(N_cts,N_var);
    J_cts_FLx=zeros(N_cts,N_var);
    J_cts_FLy=zeros(N_cts,N_var);
    
    qJac_X=Grad_CONS.qJac_X;
    qdotJac_X=Grad_CONS.qdotJac_X;
    qddotJac_X=Grad_CONS.qddotJac_X;
    qqdotJac_X=Grad_CONS.qqdotJac_X;
    aJac_X=Grad_CONS.aJac_X;
    
    Jac_Fg_var=zeros(N_step,N_g  ,N_var);
    Jac_qSeg=zeros(N_step,9,N_var);
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

        J_T_Joi=Fun_J_T_Joi(TJmax);
        Jac_VarsEq=[qJac_X;qdotJac_X;qddotJac_X;
            J_T_Joi*aJac_X;
            J_cts_FRx;J_cts_FRy;J_cts_FLx;J_cts_FLy];
        
        Jac_Eq_nz=Fun_J_Eq_nzval(CON_Mass,CON_Len,ctsx,ctsy,q_s(ni,:),qdot_s(ni,:),qddot_s(ni,:),TJ_s(ni,:),...
                cts_FRx(ni,:),cts_FRy(ni,:),cts_FLx(ni,:),cts_FLy(ni,:));
        
        Jac_Eq_var=sparse(EOM_CONS.J_Eq_nzrow  ,EOM_CONS.J_Eq_nzcol  ,Jac_Eq_nz,...
                        EOM_CONS.J_Eq_size(1),EOM_CONS.J_Eq_size(2));
        
        if ni<N_step
            JacEq(ni,:,:)=Jac_Eq_var*Jac_VarsEq;
        end
        Jac_Fg_var(ni,:,:)=[sum(J_cts_FRx,1);sum(J_cts_FRy,1);sum(J_cts_FLx,1);sum(J_cts_FLy,1)];
    
        Jac_qSeg(ni,:,:)=[qJac_X(1:3,:);Fun_J_qSeg(q_s(ni,:))*[qJac_X]];
    end
    
    Jac_Eq_step=reshape(JacEq,[],N_var);
    [Jac_Eq_nzrow,Jac_Eq_nzcol,Eq_nzval]=find(Jac_Eq_step);
    Jac_Eq_nzcol_X=(Jac_Eq_nzcol-1)*N_step+mod(Jac_Eq_nzrow-1,N_step-1)+1;
    Jac_Eq_X=sparse(Jac_Eq_nzrow,Jac_Eq_nzcol_X,Eq_nzval,size(Jac_Eq_step,1),N_X);
    
    
    Jac_Fg_s=reshape(Jac_Fg_var,[],N_var);
    [Jac_Fg_nzrow,Jac_Fg_nzcol,Jac_Fg_nzval]=find(Jac_Fg_s);
    Jac_Fg_nzcol_X=(Jac_Fg_nzcol-1)*N_step+mod(Jac_Fg_nzrow-1,N_step)+1;
    Jac_Fg_X=sparse(Jac_Fg_nzrow,Jac_Fg_nzcol_X,Jac_Fg_nzval,size(Jac_Fg_s,1),N_X);

    Jac_qSeg_s=reshape(Jac_qSeg,[],N_var);
    [Jac_qSeg_nzrow,Jac_qSeg_nzcol,Jac_qSeg_nzval]=find(Jac_qSeg_s);
    Jac_qSeg_nzcol_X=(Jac_qSeg_nzcol-1)*N_step+mod(Jac_qSeg_nzrow-1,N_step)+1;
    Jac_qSeg_X=sparse(Jac_qSeg_nzrow,Jac_qSeg_nzcol_X,Jac_qSeg_nzval,size(Jac_qSeg_s,1),N_X);
    %%
    cJv_nzval  = Fun_nlcneqJvnz(Fg_s,auxdata.nlcneqw_Fg,auxdata.Fgy_HSmax,auxdata.Fgy_HSmin,...
    auxdata.Fgyperimax,auxdata.Fgxperimax,auxdata.Flgyswingmax,...
    auxdata.nlcneqw_qSeg,auxdata.delta_qSeg_perimin,auxdata.delta_qSeg_perimax,qSeg);

    nlcneqJv=sparse(Grad_CONS.nlcneqJvnz_row,Grad_CONS.nlcneqJvnz_col,cJv_nzval,...
        Grad_CONS.nlcneqJv_size(1),Grad_CONS.nlcneqJv_size(2));
        
    varnlcneqJX = [eye(N_X);Jac_Fg_X;Jac_qSeg_X];
    
    %%
    J_adot_var=zeros(N_step,N_u  ,N_var);
    auJac_X=Grad_CONS.auJac_X;
    for ni=1:N_step
    
        J_adot_Joi=Fun_J_adot_Joi(a_s(ni,:),u_s(ni,:),c_exc);
        J_adot_var(ni,:,:)=J_adot_Joi*auJac_X;
    
    end
     Jac_adot_s=reshape(J_adot_var,[],N_var);
    [Jac_adot_nzrow,Jac_adot_nzcol,Jac_adot_nzval]=find(Jac_adot_s);
    Jac_adot_nzcol_X=(Jac_adot_nzcol-1)*N_step+mod(Jac_adot_nzrow-1,N_step)+1;
    Jac_adot_X=sparse(Jac_adot_nzrow,Jac_adot_nzcol_X,Jac_adot_nzval,size(Jac_adot_s,1),N_X);
    
    %% Jac_ceq_X
    nlceqJvnz_val= Fun_nlceqJvnz(q_s,qdot_s,qddot_s,a_s,u_s,Eq_s,adot_s,Time_step,Fac_Xnorm,...
    auxdata.nlceqw_qdot,auxdata.nlceqw_qddot,auxdata.nlceqw_Eq,auxdata.nlceqw_adot);

    nlceqJv=sparse(Grad_CONS.nlceqJvnz_row,Grad_CONS.nlceqJvnz_col,nlceqJvnz_val,...
                         Grad_CONS.nlceqJv_size(1),Grad_CONS.nlceqJv_size(2));
    
    varnlceq1JX=eye(N_X);
    varnlceqJX=[varnlceq1JX;Jac_adot_X;Jac_Eq_X;];
    
    %%
    nlcneqJX=(nlcneqJv*varnlcneqJX);
    nlceqJX=(nlceqJv*varnlceqJX);
    
    nlcneqJX=nlcneqJX*Jac_Xnorm;
    nlceqJX=nlceqJX*Jac_Xnorm;
    
    %% depend on solver type
    if strcmp(auxdata.solver,'fmincon')
        nlcneqJX=nlcneqJX.';
        nlceqJX=nlceqJX.';
    end
else % NO jacobian info
    nlcneqJX=0;
    nlceqJX=0;
end

end
















