
% compute the symbolic expression of gradient and jacobian matrix.
% using velocity as design variable rather than step time.

clear;clc;
%#ok<*NBRAK>  % unnecessary backets ok 

t_start = datetime('now','format','yyyyMMdd_HHmmss');
N_step = 3;
N_step = 30;

%%
path(pathdef);
addpath('./TLFuns')
EOM_FolderName='AutoGeneFuns';
addpath(EOM_FolderName);

EOM_CONS=load([EOM_FolderName,'/','EOM_CONS.mat']);
Torque_CONS=load([EOM_FolderName,'/','Torque_CONS.mat']);

%%
objFlag='PD'; % predict
Grad_FolderName=['AutoGeneGrad_',objFlag,'_N',num2str(N_step),''];
if ~exist(Grad_FolderName,'dir')
    mkdir(Grad_FolderName)
end

%% Timing
T_SP=sym('T_SP'); % step time
t_grid_step   = linspace(0,T_SP,N_step).';
tdot_s=t_grid_step(2:end)-t_grid_step(1:end-1);

%%
OSx=sym('OSx');
OSy=sym('OSy');

%%
N_q_ske=EOM_CONS.N_q_ske;
N_T_Joi=EOM_CONS.N_T_Joi;

N_q=N_q_ske;
q_s=sym('q_s',[N_step,N_q]);
qdot_s=sym('qdot_s',[N_step,N_q]);
qddot_s=sym('qddot_s',[N_step,N_q]);

N_u=N_T_Joi;
a_s=sym('a_s',[N_step,N_u]);
u_s=sym('u_s',[N_step,N_u]);


var_s=[q_s,qdot_s,qddot_s,a_s,u_s];
N_var=N_q+N_q+N_q+N_u+N_u;

X=[reshape(var_s,[],1);OSx;OSy]; % Velo;
N_X    = length(X);

N_para = N_X-N_var*N_step;
N_g = 4;

%% Fac_Xnorm

Fac_Xnorm=sym('Fac_Xnorm',[N_X,1]);
Fac_Xnorm_mat=reshape(Fac_Xnorm(1:end-N_para),N_step,[]);

Fac_qnorm=Fac_Xnorm_mat(1,1:N_q);
Fac_qdotnorm=Fac_Xnorm_mat(1,(1+N_q):N_q*2);
Fac_qddotnorm=Fac_Xnorm_mat(1,(1+N_q*2):N_q*3);
Fac_anorm=Fac_Xnorm_mat(1,(1+N_q*3):(N_q*3+N_u));
Fac_unorm=Fac_Xnorm_mat(1,(1+N_q*3+N_u):N_var);

%% tracking cost

Fg_s=sym('Fg_s',[N_step,4]);

q_exp    =sym('q_exp',[N_step,N_q]);
qdot_exp =sym('qdot_exp',[N_step,N_q]);
qddot_exp=sym('qddot_exp',[N_step,N_q]);

qErr_sum    =TLtrapz(t_grid_step,(q_s    -q_exp    ).^2);
qdotErr_sum =TLtrapz(t_grid_step,(qdot_s -qdot_exp ).^2);
qddotErr_sum=TLtrapz(t_grid_step,(qddot_s-qddot_exp).^2);

objw_qErr    =sym('w_qErr'    ,[N_q,1]); % weight factors
objw_qdotErr =sym('w_qdotErr' ,[N_q,1]);
objw_qddotErr=sym('w_qddotErr',[N_q,1]);

obj_qErr    = qErr_sum    * objw_qErr;
obj_qdotErr = qdotErr_sum * objw_qdotErr;
obj_qddotErr= qddotErr_sum* objw_qddotErr;

%% angle of segments
qSeg=sym('qSeg',[N_step,N_q]);
qSeg_exp=sym('qSeg_exp',[N_step,N_q]);
qdotSeg=sym('qdotSeg',[N_step,N_q]);
qdotSeg_exp=sym('qdotSeg_exp',[N_step,N_q]);

qSegErr_sum    =TLtrapz(t_grid_step,(qSeg    -qSeg_exp    ).^2);
qdotSegErr_sum =TLtrapz(t_grid_step,(qdotSeg -qdotSeg_exp ).^2);

objw_qSegErr    =sym('w_qSegErr'    ,[N_q,1]); % weight factors
objw_qdotSegErr =sym('w_qdotSegErr' ,[N_q,1]);

obj_qSegErr    = qSegErr_sum    * objw_qSegErr;
obj_qdotSegErr = qdotSegErr_sum * objw_qdotSegErr;

%% position of joints 
pJoi=sym('pJoi',[N_step,12]); % rhx rhy rkx rky rax ray *2
pJoi_exp=sym('pJoi_exp',[N_step,12]); 
pJoi_exp(:,11)=pJoi_exp(:,11)+OSx; % lax
pJoi_exp(:,12)=pJoi_exp(:,12)+OSy; % lay
pJoi_exp(:,6)=pJoi_exp(:,6)+OSy;   % ray

pdotJoi=sym('pdotJoi',[N_step,12]);
pdotJoi_exp=sym('pdotJoi_exp',[N_step,12]);
pddotJoi=sym('pddotJoi',[N_step,12]);
pddotJoi_exp=sym('pddotJoi_exp',[N_step,12]);

pJoiErr_sum     =TLtrapz(t_grid_step,(pJoi     -pJoi_exp    ).^2);
pdotJoiErr_sum  =TLtrapz(t_grid_step,(pdotJoi  -pdotJoi_exp ).^2);
pddotJoiErr_sum =TLtrapz(t_grid_step,(pddotJoi -pddotJoi_exp ).^2);

objw_pJoiErr     =sym('w_pJoiErr'     ,[12,1]); % weight factors
objw_pdotJoiErr  =sym('w_pdotJoiErr'  ,[12,1]);
objw_pddotJoiErr =sym('w_pddotJoiErr' ,[12,1]);

obj_pJoiErr     = pJoiErr_sum     * objw_pJoiErr;
obj_pdotJoiErr  = pdotJoiErr_sum  * objw_pdotJoiErr;
obj_pddotJoiErr = pddotJoiErr_sum * objw_pddotJoiErr;


%% Mechanical work
epsl=sym('epsl');

power_s=sym('power_s',[N_step,N_u]);
power_s_abs=sqrt(power_s.^2+epsl^2)-epsl;
work_sum=TLtrapz(t_grid_step,power_s_abs);
objw_work=sym('w_work',[N_u,1]);
obj_work=work_sum * objw_work;

%% Force Cost
u2_sum=TLtrapz(t_grid_step,u_s.^2);
objw_u2=sym('w_u2',[N_u,1]);
obj_u2=u2_sum*objw_u2;

%% udot Cost
udot_s=TLdiff(u_s,t_grid_step,0);
udot_sum=sum(udot_s.^2.*(tdot_s*ones(1,N_u)));
objw_udot=sym('w_udot',[N_u,1]);
obj_udot=udot_sum*objw_udot;

%% qddot dot Cost
qdddot_s=TLdiff(qddot_s,t_grid_step,0);
qdddot_sum=sum(qdddot_s.^2.*(tdot_s*ones(1,N_q)));
objw_qdddot=sym('w_qdddot',[N_q,1]);
obj_qdddot=qdddot_sum  *objw_qdddot;

%% Fgdot Cost
Fgdot_s=TLdiff(Fg_s,t_grid_step,0);
Fgdot_sum=sum(Fgdot_s.^2.*(tdot_s*ones(1,N_g)));
objw_Fgdot=sym('w_Fgdot',[N_g,1]);
obj_Fgdot=Fgdot_sum*objw_Fgdot;

%%
obj=obj_qSegErr  + ...
    obj_pJoiErr   + ...
    (obj_work   + obj_qdddot );

varobj1=X;
varobj2=[reshape(power_s,[],1);
         reshape(qSeg,[],1);
         reshape(pJoi,[],1);];
varobj=[varobj1;varobj2];
objJv=jacobian(obj,varobj);

%% char for subs
[q_cOld    ,q_cNew    ]=TLcharsymmat(q_s);
[qdot_cOld ,qdot_cNew ]=TLcharsymmat(qdot_s);
[qddot_cOld,qddot_cNew]=TLcharsymmat(qddot_s);
[a_cOld    ,a_cNew    ]=TLcharsymmat(a_s);
[u_cOld    ,u_cNew    ]=TLcharsymmat(u_s);
[Fg_cOld   ,Fg_cNew   ]=TLcharsymmat(Fg_s);
[power_cOld   ,power_cNew   ]=TLcharsymmat(power_s);
[qSeg_cOld    ,qSeg_cNew    ]=TLcharsymmat(qSeg);
[qdotSeg_cOld ,qdotSeg_cNew ]=TLcharsymmat(qdotSeg);
[pJoi_cOld    ,pJoi_cNew    ]=TLcharsymmat(pJoi);
[pdotJoi_cOld ,pdotJoi_cNew ]=TLcharsymmat(pdotJoi);
[pddotJoi_cOld ,pddotJoi_cNew ]=TLcharsymmat(pddotJoi);

[objw_qErr_cOld     ,objw_qErr_cNew]     =TLcharsymmat(objw_qErr);
[objw_qdotErr_cOld  ,objw_qdotErr_cNew]  =TLcharsymmat(objw_qdotErr);
[objw_qddotErr_cOld ,objw_qddotErr_cNew] =TLcharsymmat(objw_qddotErr);
[objw_qSegErr_cOld     ,objw_qSegErr_cNew]     =TLcharsymmat(objw_qSegErr);
[objw_qdotSegErr_cOld  ,objw_qdotSegErr_cNew]  =TLcharsymmat(objw_qdotSegErr);
[objw_pJoiErr_cOld     ,objw_pJoiErr_cNew]     =TLcharsymmat(objw_pJoiErr);
[objw_pdotJoiErr_cOld  ,objw_pdotJoiErr_cNew]  =TLcharsymmat(objw_pdotJoiErr);
[objw_pddotJoiErr_cOld  ,objw_pddotJoiErr_cNew]  =TLcharsymmat(objw_pddotJoiErr);

[objw_work_cOld     ,objw_work_cNew]     =TLcharsymmat(objw_work);
[objw_u2_cOld   ,objw_u2_cNew]           =TLcharsymmat(objw_u2);
[objw_udot_cOld     ,objw_udot_cNew]     =TLcharsymmat(objw_udot);
[objw_Fgdot_cOld    ,objw_Fgdot_cNew]    =TLcharsymmat(objw_Fgdot);
[objw_qdddot_cOld   ,objw_qdddot_cNew]   =TLcharsymmat(objw_qdddot);

[q_exp_cOld    ,q_exp_cNew    ]=TLcharsymmat(q_exp);
[qdot_exp_cOld ,qdot_exp_cNew ]=TLcharsymmat(qdot_exp);
[qddot_exp_cOld,qddot_exp_cNew]=TLcharsymmat(qddot_exp);
[qSeg_exp_cOld    ,qSeg_exp_cNew    ]=TLcharsymmat(qSeg_exp);
[qdotSeg_exp_cOld ,qdotSeg_exp_cNew ]=TLcharsymmat(qdotSeg_exp);
[pJoi_exp_cOld    ,pJoi_exp_cNew    ]=TLcharsymmat(pJoi_exp);
[pdotJoi_exp_cOld ,pdotJoi_exp_cNew ]=TLcharsymmat(pdotJoi_exp);
[pddotJoi_exp_cOld ,pddotJoi_exp_cNew ]=TLcharsymmat(pddotJoi_exp);


varobj_cOld=[q_cOld;qdot_cOld;qddot_cOld;a_cOld;u_cOld;Fg_cOld;power_cOld;qSeg_cOld;qdotSeg_cOld;pJoi_cOld;pdotJoi_cOld;pddotJoi_cOld;
             objw_work_cOld;objw_u2_cOld;objw_udot_cOld;objw_Fgdot_cOld;objw_qdddot_cOld;
             objw_qErr_cOld;objw_qdotErr_cOld;objw_qddotErr_cOld;
             objw_qSegErr_cOld;objw_qdotSegErr_cOld;objw_pJoiErr_cOld;objw_pdotJoiErr_cOld;objw_pddotJoiErr_cOld;
             q_exp_cOld;qdot_exp_cOld;qddot_exp_cOld;
             qSeg_exp_cOld;qdotSeg_exp_cOld;pJoi_exp_cOld;pdotJoi_exp_cOld;pddotJoi_exp_cOld;];
            
varobj_cNew=[q_cNew;qdot_cNew;qddot_cNew;a_cNew;u_cNew;Fg_cNew;power_cNew;qSeg_cNew;qdotSeg_cNew;pJoi_cNew;pdotJoi_cNew;pddotJoi_cNew;
             objw_work_cNew;objw_u2_cNew;objw_udot_cNew;objw_Fgdot_cNew;objw_qdddot_cNew;
             objw_qErr_cNew;objw_qdotErr_cNew;objw_qddotErr_cNew;
             objw_qSegErr_cNew;objw_qdotSegErr_cNew;objw_pJoiErr_cNew;objw_pdotJoiErr_cNew;objw_pddotJoiErr_cNew;
             q_exp_cNew;qdot_exp_cNew;qddot_exp_cNew;
             qSeg_exp_cNew;qdotSeg_exp_cNew;pJoi_exp_cNew;pdotJoi_exp_cNew;pddotJoi_exp_cNew;];

%% replace
tic
obj_new=subs(obj,varobj_cOld,varobj_cNew);
objJv_new=subs(objJv,varobj_cOld,varobj_cNew);
substoc_obj=toc;
disp(['substoc_obj=',num2str(substoc_obj),'sec'])

%% print function
grad_arg=['q_s,qdot_s,qddot_s,a_s,u_s,Fg_s,power_s,qSeg,qdotSeg,pJoi,pdotJoi,pddotJoi,',...
          'w_work,w_u2,w_udot,w_Fgdot,w_qdddot,',...
          'w_qErr,w_qdotErr,w_qddotErr,w_qSegErr,w_qdotSegErr,w_pJoiErr,w_pdotJoiErr,w_pddotJoiErr,',... 
          'q_exp,qdot_exp,qddot_exp,qSeg_exp,qdotSeg_exp,pJoi_exp,pdotJoi_exp,pddotJoi_exp,',... 
          'T_SP,OSx,OSy,epsl'];
TLprintFunction(Grad_FolderName,'Fun_obj',grad_arg,obj_new)
TLprintFunction(Grad_FolderName,'Fun_objJv',grad_arg,objJv_new)

%% Constraint
qdot_s_FD=TLdiff(q_s,t_grid_step,0);
qddot_s_FD=TLdiff(qdot_s,t_grid_step,0);
adot_s_FD=TLdiff(a_s,t_grid_step,0);

Eq_s=sym('Eq_s',[N_step-1,N_q]);
adot_s=sym('adot_s',[N_step,N_u]);

nlceqw_qdot=sym('nlceqw_qdot');
nlceqw_qddot=sym('nlceqw_qddot');
nlceqw_Eq=sym('nlceqw_Eq');
nlceqw_adot=sym('nlceqw_adot');


qdot_defect=qdot_s_FD-(qdot_s(1:end-1,:) + qdot_s(2:end,:))/2; % error of velocity integration - trapzoid rule
qdot_defect=nlceqw_qdot*qdot_defect./(ones(size(qdot_defect,1),1)*Fac_qdotnorm);
nlceq_q_s   =reshape(qdot_defect,[],1);

qddot_defect=qddot_s_FD-(qddot_s(1:end-1,:) + qddot_s(2:end,:))/2; % error of velocity integration
qddot_defect=nlceqw_qddot*qddot_defect./(ones(size(qddot_defect,1),1)*Fac_qddotnorm);
nlceq_qdot_s=reshape(qddot_defect,[],1);

Eq_defect=nlceqw_Eq*Eq_s./(ones(size(Eq_s,1),1)*[200,800,300*ones(1,7)]);
nlceq_Eq_s  =reshape(Eq_defect,[],1);

adots_defect=adot_s_FD-(adot_s(1:end-1,:)+adot_s(2:end,:))/2;
adots_defect=nlceqw_adot*adots_defect./(ones(size(adots_defect,1),1)*Fac_anorm*10);
nlceq_adot_s=reshape(adots_defect,[],1);

%% 
lceqw_peri=sym('lceqw_peri',[12,1]);

lceq_q_peri=[q_s(1,1)*lceqw_peri(1);    
            (q_s(end,2:3)-q_s(1,2:3)).'*lceqw_peri(2);
            (q_s(end,4:6)-q_s(1,7:9)).'*lceqw_peri(3);
            (q_s(end,7:9)-q_s(1,4:6)).'*lceqw_peri(4)];

lceq_qdot_peri=[(qdot_s(end,1:3)-qdot_s(1,1:3)).'*lceqw_peri(5);
                (qdot_s(end,4:6)-qdot_s(1,7:9)).'*lceqw_peri(6);
                (qdot_s(end,7:9)-qdot_s(1,4:6)).'*lceqw_peri(7)];

lceq_qddot_peri=[(qddot_s(end,1:3)-qddot_s(1,1:3)).'*lceqw_peri(8);
                 (qddot_s(end,4:6)-qddot_s(1,7:9)).'*lceqw_peri(9);
                 (qddot_s(end,7:9)-qddot_s(1,4:6)).'*lceqw_peri(10)];

lceq_a_peri=[(a_s(end,1:3)-a_s(1,4:6)).';
             (a_s(end,4:6)-a_s(1,1:3)).']*lceqw_peri(11);
         
lceq_u_peri=[(u_s(end,1:3)-u_s(1,4:6)).';
             (u_s(end,4:6)-u_s(1,1:3)).']*lceqw_peri(12);
%%

lcneqw_peri=sym('lcneqw_peri',[5,1]);

delta_q_perimin=sym('delta_q_perimin',[9,1]);
delta_qdot_perimin=sym('delta_qdot_perimin',[9,1]);
delta_qddot_perimin=sym('delta_qddot_perimin',[9,1]);
delta_a_perimin=sym('delta_a_perimin',[6,1]);
delta_u_perimin=sym('delta_u_perimin',[6,1]);

delta_q_perimax=sym('delta_q_perimax',[9,1]);
delta_qdot_perimax=sym('delta_qdot_perimax',[9,1]);
delta_qddot_perimax=sym('delta_qddot_perimax',[9,1]);
delta_a_perimax=sym('delta_a_perimax',[6,1]);
delta_u_perimax=sym('delta_u_perimax',[6,1]);
% abs(q)<delta => -delta<q<delta => -delta-q<0 && -delta+q<0 % not using this anymore
% delta_min < q < delta_max => delta_min-q<0 && -delta_max+q < 0

lcneq_q_peri=[ delta_q_perimin(1:3)-(q_s(end,1:3)-q_s(1,1:3)).';
              -delta_q_perimax(1:3)+(q_s(end,1:3)-q_s(1,1:3)).';
               delta_q_perimin(4:6)-(q_s(end,4:6)-q_s(1,7:9)).';
              -delta_q_perimax(4:6)+(q_s(end,4:6)-q_s(1,7:9)).';
               delta_q_perimin(7:9)-(q_s(end,7:9)-q_s(1,4:6)).';
              -delta_q_perimax(7:9)+(q_s(end,7:9)-q_s(1,4:6)).';]*lcneqw_peri(1);

lcneq_qdot_peri=[ delta_qdot_perimin(1:3)-(qdot_s(end,1:3)-qdot_s(1,1:3)).';
                 -delta_qdot_perimax(1:3)+(qdot_s(end,1:3)-qdot_s(1,1:3)).';
                  delta_qdot_perimin(4:6)-(qdot_s(end,4:6)-qdot_s(1,7:9)).';
                 -delta_qdot_perimax(4:6)+(qdot_s(end,4:6)-qdot_s(1,7:9)).';
                  delta_qdot_perimin(7:9)-(qdot_s(end,7:9)-qdot_s(1,4:6)).';
                 -delta_qdot_perimax(7:9)+(qdot_s(end,7:9)-qdot_s(1,4:6)).';]*lcneqw_peri(2);

lcneq_qddot_peri=[ delta_qddot_perimin(1:3)-(qddot_s(end,1:3)-qddot_s(1,1:3)).';
                  -delta_qddot_perimax(1:3)+(qddot_s(end,1:3)-qddot_s(1,1:3)).';
                   delta_qddot_perimin(4:6)-(qddot_s(end,4:6)-qddot_s(1,7:9)).';
                  -delta_qddot_perimax(4:6)+(qddot_s(end,4:6)-qddot_s(1,7:9)).';
                   delta_qddot_perimin(7:9)-(qddot_s(end,7:9)-qddot_s(1,4:6)).';
                  -delta_qddot_perimax(7:9)+(qddot_s(end,7:9)-qddot_s(1,4:6)).';]*lcneqw_peri(3);

lcneq_a_peri=[ delta_a_perimin(1:3)-(a_s(end,1:3)-a_s(1,4:6)).';
              -delta_a_perimax(1:3)+(a_s(end,1:3)-a_s(1,4:6)).';
               delta_a_perimin(4:6)-(a_s(end,4:6)-a_s(1,1:3)).';
              -delta_a_perimax(4:6)+(a_s(end,4:6)-a_s(1,1:3)).';]*lcneqw_peri(4);
          
lcneq_u_peri=[ delta_u_perimin(1:3)-(u_s(end,1:3)-u_s(1,4:6)).';
              -delta_u_perimax(1:3)+(u_s(end,1:3)-u_s(1,4:6)).';
               delta_u_perimin(4:6)-(u_s(end,4:6)-u_s(1,1:3)).';
              -delta_u_perimax(4:6)+(u_s(end,4:6)-u_s(1,1:3)).';]*lcneqw_peri(5);

%% S time equality

lceqw_OSx=sym('lceqw_OSx');
OSx_exp=sym('OSx_exp');
lceq_OSx=(OSx-OSx_exp)*lceqw_OSx;

lceqw_OSy=sym('lceqw_OSy');
OSy_exp=sym('OSy_exp');
lceq_OSy=(OSy-OSy_exp)*lceqw_OSy;

%% force peri and force
nlcneqw_Fg=sym('nlcneqw_Fg',[9,1]);
Fgy_HSmax=sym('Fgy_HSmax');
Fgy_HSmin=sym('Fgy_HSmin');
Fgyperimax=sym('Fgyperimax');
Fgxperimax=sym('Fgxperimax');
Flgyswingmax=sym('Flgyswingmax');

nlceq_Fg=[(Fg_s(1,2)-Fgy_HSmax)*nlcneqw_Fg(1); % Frgy(1)<Fgy_HSmax
          (-Fg_s(1,2)+Fgy_HSmin)*nlcneqw_Fg(2); % Frgy(1)>Fgy_HSmin
          (Fg_s(end,2)-Fg_s(1,4)-Fgyperimax)*nlcneqw_Fg(3); % abs(Frgy(end)-Flgy(1)) <Fgyperimax
          (-Fg_s(end,2)+Fg_s(1,4)-Fgyperimax)*nlcneqw_Fg(4);
          (Fg_s(end,1)-Fg_s(1,3)-Fgxperimax)*nlcneqw_Fg(5); % abs(Frgx(end)-Flgx(1)) <Fgxperimax
          (-Fg_s(end,1)+Fg_s(1,3)-Fgxperimax)*nlcneqw_Fg(6);
          (Fg_s(end,4)-Fgy_HSmax)*nlcneqw_Fg(7); % Flgy(end)<=Fgy_HSmax
          (-Fg_s(end,4)+Fgy_HSmin)*nlcneqw_Fg(8); % Flgy(end)>=Fgy_HSmin
          (Fg_s(round(N_step/2):(end-1),4)-Flgyswingmax)*nlcneqw_Fg(9); % Flgy(swing)<Fgmax
          ];
%%
nlcneqw_qSeg=sym('nlcneqw_qSeg');
delta_qSeg_perimin=sym('delta_qSeg_perimin',[9,1]);
delta_qSeg_perimax=sym('delta_qSeg_perimax',[9,1]);
nlcneq_qSeg_peri=[delta_qSeg_perimin(1:3)-(qSeg(end,1:3)-qSeg(1,1:3)).'; % delta_perimin< q(end)-q(1) <delta_perimax
                 -delta_qSeg_perimax(1:3)+(qSeg(end,1:3)-qSeg(1,1:3)).';
                  delta_qSeg_perimin(4:6)-(qSeg(end,4:6)-qSeg(1,7:9)).';
                 -delta_qSeg_perimax(4:6)+(qSeg(end,4:6)-qSeg(1,7:9)).';
                  delta_qSeg_perimin(7:9)-(qSeg(end,7:9)-qSeg(1,4:6)).';
                 -delta_qSeg_perimax(7:9)+(qSeg(end,7:9)-qSeg(1,4:6)).';]*nlcneqw_qSeg;

%% -----------------Collection of Constraints----------------------

nlcneq   = [ nlceq_Fg;nlcneq_qSeg_peri ]; % nlcneq_udot;nlcneq_qdddot
nlceq = [nlceq_q_s;nlceq_qdot_s;nlceq_Eq_s;nlceq_adot_s]; % ;nlceq_LSL

lcneq=[lcneq_q_peri;lcneq_qdot_peri;lcneq_qddot_peri;lcneq_a_peri;lcneq_u_peri;];
lceq =[lceq_q_peri;lceq_qdot_peri;lceq_qddot_peri;lceq_a_peri;lceq_u_peri;lceq_OSx;lceq_OSy]; % lceq_Velo;

N_nlcneq=length(nlcneq);
N_nlceq=length(nlceq);
N_lcneq=length(lcneq);
N_lceq=length(lceq);

%% linear-constraint in-equality
lcneqA=jacobian(lcneq,X);
lcneqb=simplify(jacobian(lcneq,X)*X-lcneq);

%% linear-constraint equality
lceqA=jacobian(lceq,X);
lceqb=simplify(jacobian(lceq,X)*X-lceq);

%% Generate Functions linear non-equality Lc-neq
[lcneqAnz_row,lcneqAnz_col,lcneqAnz_val]=find(lcneqA);
lcneqA_size=size(lcneqA);
lcneqA_strc=TLmatstrc(lcneqA);

[lcneqw_peri_cOld,lcneqw_peri_cNew]=TLcharsymmat(lcneqw_peri);

[delta_q_perimin_cOld,delta_q_perimin_cNew]=TLcharsymmat(delta_q_perimin);
[delta_qdot_perimin_cOld,delta_qdot_perimin_cNew]=TLcharsymmat(delta_qdot_perimin);
[delta_qddot_perimin_cOld,delta_qddot_perimin_cNew]=TLcharsymmat(delta_qddot_perimin);
[delta_a_perimin_cOld,delta_a_perimin_cNew]=TLcharsymmat(delta_a_perimin);
[delta_u_perimin_cOld,delta_u_perimin_cNew]=TLcharsymmat(delta_u_perimin);

[delta_q_perimax_cOld,delta_q_perimax_cNew]=TLcharsymmat(delta_q_perimax);
[delta_qdot_perimax_cOld,delta_qdot_perimax_cNew]=TLcharsymmat(delta_qdot_perimax);
[delta_qddot_perimax_cOld,delta_qddot_perimax_cNew]=TLcharsymmat(delta_qddot_perimax);
[delta_a_perimax_cOld,delta_a_perimax_cNew]=TLcharsymmat(delta_a_perimax);
[delta_u_perimax_cOld,delta_u_perimax_cNew]=TLcharsymmat(delta_u_perimax);

varlcneq_cOld=[lcneqw_peri_cOld;delta_q_perimin_cOld;delta_qdot_perimin_cOld;delta_qddot_perimin_cOld;delta_a_perimin_cOld;delta_u_perimin_cOld;
                                delta_q_perimax_cOld;delta_qdot_perimax_cOld;delta_qddot_perimax_cOld;delta_a_perimax_cOld;delta_u_perimax_cOld];
varlcneq_cNew=[lcneqw_peri_cNew;delta_q_perimin_cNew;delta_qdot_perimin_cNew;delta_qddot_perimin_cNew;delta_a_perimin_cNew;delta_u_perimin_cNew;
                                delta_q_perimax_cNew;delta_qdot_perimax_cNew;delta_qddot_perimax_cNew;delta_a_perimax_cNew;delta_u_perimax_cNew];

lcneqAnz_new=subs(lcneqAnz_val,varlcneq_cOld,varlcneq_cNew);
lcneqb_new=subs(lcneqb,varlcneq_cOld,varlcneq_cNew);

lcneqA_arg=['lcneqw_peri,delta_q_perimin,delta_qdot_perimin,delta_qddot_perimin,delta_a_perimin,delta_u_perimin,',...
                       'delta_q_perimax,delta_qdot_perimax,delta_qddot_perimax,delta_a_perimax,delta_u_perimax'];
TLprintFunction(Grad_FolderName,'Fun_lcneqA',lcneqA_arg,lcneqAnz_new)
lcneqb_arg=['lcneqw_peri,delta_q_perimin,delta_qdot_perimin,delta_qddot_perimin,delta_a_perimin,delta_u_perimin,',...
                        'delta_q_perimax,delta_qdot_perimax,delta_qddot_perimax,delta_a_perimax,delta_u_perimax'];
TLprintFunction(Grad_FolderName,'Fun_lcneqb',lcneqb_arg,lcneqb_new)

%% Generate Functions linear equality Lc eq
[lceqAnz_row,lceqAnz_col,lceqAnz_val]=find(lceqA);
lceqA_size=size(lceqA);
lceqA_strc=TLmatstrc(lceqA);

[lceqw_peri_cOld,lceqw_peri_cNew]=TLcharsymmat(lceqw_peri);

varlceq_cOld=[lceqw_peri_cOld];
varlceq_cNew=[lceqw_peri_cNew];

lceqAnz_new=subs(lceqAnz_val,varlceq_cOld,varlceq_cNew);
lceqb_new=subs(lceqb,varlceq_cOld,varlceq_cNew);

lceqA_arg='lceqw_peri,lceqw_OSx,lceqw_OSy';
TLprintFunction(Grad_FolderName,'Fun_lceqA',lceqA_arg,lceqAnz_new)
lceqb_arg='lceqw_peri,lceqw_OSx,lceqw_OSy,OSx_exp,OSy_exp';
TLprintFunction(Grad_FolderName,'Fun_lceqb',lceqb_arg,lceqb_new)



%% c Jac w.r.t. vars
varnlcneq1=[X];
varnlcneq2=[reshape(Fg_s,[],1)
         reshape(qSeg,[],1);];
varnlcneq =[varnlcneq1; varnlcneq2];
nlcneqJv=jacobian(nlcneq,varnlcneq);

[nlcneqJvnz_row,nlcneqJvnz_col,nlcneqJvnz_val]=find(nlcneqJv);
nlcneqJv_size=size(nlcneqJv);
nlcneqJv_strc=TLmatstrc(nlcneqJv);

%%
[nlcneqw_Fg_cOld,nlcneqw_Fg_cNew]=TLcharsymmat(nlcneqw_Fg);
[delta_qSeg_perimin_cOld,delta_qSeg_perimin_cNew]=TLcharsymmat(delta_qSeg_perimin);
[delta_qSeg_perimax_cOld,delta_qSeg_perimax_cNew]=TLcharsymmat(delta_qSeg_perimax);

[Fac_Xnorm_cOld,Fac_Xnorm_cNew]=TLcharsymmat(Fac_Xnorm);

varnlcneq_cOld=[Fg_cOld;nlcneqw_Fg_cOld;delta_qSeg_perimin_cOld;delta_qSeg_perimax_cOld;qSeg_cOld];
varnlcneq_cNew=[Fg_cNew;nlcneqw_Fg_cNew;delta_qSeg_perimin_cNew;delta_qSeg_perimax_cNew;qSeg_cNew];
% 
tic
nlcneq_new=subs(nlcneq,varnlcneq_cOld,varnlcneq_cNew);
nlcneqJvnz_new=subs(nlcneqJvnz_val,varnlcneq_cOld,varnlcneq_cNew);
substoc_nlcneq=toc;
disp(['substoc_nlcneq=',num2str(substoc_nlcneq),'sec'])

%% Generate Functions - NLc neq

nlcneq_arg=['Fg_s,nlcneqw_Fg,Fgy_HSmax,Fgy_HSmin,Fgyperimax,Fgxperimax,',...
            'Flgyswingmax,nlcneqw_qSeg,delta_qSeg_perimin,delta_qSeg_perimax,qSeg']; 

TLprintFunction(Grad_FolderName,'Fun_nlcneq',nlcneq_arg,nlcneq_new)

nlcneqJvnz_arg=nlcneq_arg;
TLprintFunction(Grad_FolderName,'Fun_nlcneqJvnz',nlcneqJvnz_arg,nlcneqJvnz_new)

%% ceq Jac w.r.t. vars
varnlceq1=[X];
varnlceq2=[reshape(adot_s,[],1);
           reshape(Eq_s,[],1)];
varnlceq=[varnlceq1;varnlceq2];

nlceqJv=jacobian(nlceq,varnlceq);

[nlceqJvnz_row,nlceqJvnz_col]=find(nlceqJv);
nlceqJv_size=size(nlceqJv);
nlceqJvnz_val=nlceqJv(nlceqJvnz_row + nlceqJvnz_col*N_nlceq-N_nlceq);
nlceqJv_strc=TLmatstrc(nlceqJv);
%% subs ceq
[Eq_cOld,Eq_cNew]=TLcharsymmat(Eq_s);
[adot_cOld,adot_cNew]=TLcharsymmat(adot_s);

varnlceq_cOld=[q_cOld;qdot_cOld;qddot_cOld;a_cOld;u_cOld;Fg_cOld;Eq_cOld;adot_cOld;Fac_Xnorm_cOld];
varnlceq_cNew=[q_cNew;qdot_cNew;qddot_cNew;a_cNew;u_cNew;Fg_cNew;Eq_cNew;adot_cNew;Fac_Xnorm_cNew];

tic
nlceq_new=subs(nlceq,varnlceq_cOld,varnlceq_cNew);
nlceqJvnz_new=subs(nlceqJvnz_val,varnlceq_cOld,varnlceq_cNew);
substoc_nlceq=toc;
disp(['substoc_nlceq=',num2str(substoc_nlceq),'sec'])

%%  Generate Functions - NLceq

nlceq_arg=['q_s,qdot_s,qddot_s,a_s,u_s,Eq_s,adot_s,',...
    'T_SP,Fac_Xnorm,nlceqw_qdot,nlceqw_qddot,nlceqw_Eq,nlceqw_adot']; % ,nlceqw_LSL,Velo

TLprintFunction(Grad_FolderName,'Fun_nlceq',nlceq_arg,nlceq_new)
nlceqJvnz_arg=nlceq_arg;
TLprintFunction(Grad_FolderName,'Fun_nlceqJvnz',nlceqJvnz_arg,nlceqJvnz_new)

%% ------------find sparse strcture of jacobian matrix

qJac_X     =double(jacobian(q_s(1,:)    ,var_s(1,:)));
qdotJac_X  =double(jacobian(qdot_s(1,:) ,var_s(1,:)));
qddotJac_X =double(jacobian(qddot_s(1,:),var_s(1,:)));
aJac_X     =double(jacobian(a_s(1,:)    ,var_s(1,:)));
uJac_X     =double(jacobian(u_s(1,:)    ,var_s(1,:)));


qqdotJac_X =[qJac_X;qdotJac_X];
qqdotqddotJac_X=[qJac_X;qdotJac_X;qddotJac_X];

%% EOM
N_cts=EOM_CONS.N_cts;

Jac_Eq_var=zeros(N_step-1,N_q*1  ,N_var);
J_cts_FRx=zeros(N_cts,N_var);
J_cts_FRy=zeros(N_cts,N_var);
J_cts_FLx=zeros(N_cts,N_var);
J_cts_FLy=zeros(N_cts,N_var);

Jac_Fg_var=zeros(N_step,N_g  ,N_var);
Jac_qSeg_var=zeros(N_step,N_q  ,N_var);
for ni=1:N_step
    J_cts_FRy_strc=ones(N_cts,2);
    J_cts_FRx_strc=ones(N_cts,3);
    J_cts_FLy_strc=ones(N_cts,2);
    J_cts_FLx_strc=ones(N_cts,3);
    
    J_yrcts_strc=EOM_CONS.J_yrcts_strc;
    J_ylcts_strc=EOM_CONS.J_ylcts_strc;
    J_yrctsdot_strc=EOM_CONS.J_yrctsdot_strc;
    J_ylctsdot_strc=EOM_CONS.J_ylctsdot_strc;
    J_xrctsdot_strc=EOM_CONS.J_xrctsdot_strc;
    J_xlctsdot_strc=EOM_CONS.J_xlctsdot_strc;
    for i=1:N_cts
        
        J_cts_FRy(i,:) =J_cts_FRy_strc(i,:)*[J_yrcts_strc(i,:)*qJac_X;J_yrctsdot_strc(i,:)*qqdotJac_X];
        J_cts_FRx(i,:) =J_cts_FRx_strc(i,:)*[J_yrcts_strc(i,:)*qJac_X;J_yrctsdot_strc(i,:)*qqdotJac_X;J_xrctsdot_strc(i,:)*qqdotJac_X];
        J_cts_FLy(i,:) =J_cts_FLy_strc(i,:)*[J_ylcts_strc(i,:)*qJac_X;J_ylctsdot_strc(i,:)*qqdotJac_X];
        J_cts_FLx(i,:) =J_cts_FLx_strc(i,:)*[J_ylcts_strc(i,:)*qJac_X;J_ylctsdot_strc(i,:)*qqdotJac_X;J_xlctsdot_strc(i,:)*qqdotJac_X];
    end
    
    J_T_Joi_strc=Torque_CONS.J_T_Joi_strc;
    
    Jac_VarsEq=[qJac_X;qdotJac_X;qddotJac_X;
            J_T_Joi_strc*aJac_X;
        J_cts_FRx;J_cts_FRy;J_cts_FLx;J_cts_FLy];
    Jac_Eq_strc=EOM_CONS.J_Eq_strc;
    if ni<N_step
        Jac_Eq_var(ni,:,:)=Jac_Eq_strc*Jac_VarsEq;
    end
    J_Frgx=sum(J_cts_FRx,1);
    J_Frgy=sum(J_cts_FRy,1);
    J_Flgx=sum(J_cts_FLx,1);
    J_Flgy=sum(J_cts_FLy,1);
    Jac_Fg_var(ni,:,:)=[J_Frgx;J_Frgy;J_Flgx;J_Flgy];
    
    J_qSeg_strc=EOM_CONS.J_qSeg_strc;
    Jac_qSeg_var(ni,:,:)=[qJac_X(1:3,:);J_qSeg_strc*[qJac_X]];
end
%%
Jac_Eq_s=reshape(Jac_Eq_var,[],N_var);
[Jac_Eq_nzrow,Jac_Eq_nzcol]=find(Jac_Eq_s);
Jac_Eq_nzcol_X=(Jac_Eq_nzcol-1)*N_step + mod(Jac_Eq_nzrow-1,N_step-1) + 1;
Jac_Eq_X_strc=sparse(Jac_Eq_nzrow,Jac_Eq_nzcol_X,ones(length(Jac_Eq_nzrow),1),size(Jac_Eq_s,1),N_X);

Jac_Fg_s=reshape(Jac_Fg_var,[],N_var);
[Jac_Fg_nzrow,Jac_Fg_nzcol]=find(Jac_Fg_s);
Jac_Fg_nzcol_X=(Jac_Fg_nzcol-1)*N_step+mod(Jac_Fg_nzrow-1,N_step)+1;
Jac_Fg_X_strc=sparse(Jac_Fg_nzrow,Jac_Fg_nzcol_X,ones(length(Jac_Fg_nzrow),1),size(Jac_Fg_s,1),N_X);


Jac_qSeg_s=reshape(Jac_qSeg_var,[],N_var);
[Jac_qSeg_nzrow,Jac_qSeg_nzcol]=find(Jac_qSeg_s);
Jac_qSeg_nzcol_X=(Jac_qSeg_nzcol-1)*N_step+mod(Jac_qSeg_nzrow-1,N_step)+1;
Jac_qSeg_X_strc=sparse(Jac_qSeg_nzrow,Jac_qSeg_nzcol_X,ones(length(Jac_qSeg_nzrow),1),size(Jac_qSeg_s,1),N_X);

%%
J_adot_var=zeros(N_step,N_u  ,N_var);
auJac_X=[aJac_X;uJac_X];
for ni=1:N_step
    J_adot_Joi_strc=Torque_CONS.J_adot_Joi_strc;
    J_adot_var(ni,:,:)=J_adot_Joi_strc*auJac_X;
end
Jac_adot_s=reshape(J_adot_var,[],N_var);
[Jac_adot_nzrow,Jac_adot_nzcol]=find(Jac_adot_s);
Jac_adot_nzcol_X=(Jac_adot_nzcol-1)*N_step+mod(Jac_adot_nzrow-1,N_step)+1;
Jac_adot_X_strc=sparse(Jac_adot_nzrow,Jac_adot_nzcol_X,ones(length(Jac_adot_nzrow),1),size(Jac_adot_s,1),N_X);

%% Compute varc2 Jac w.r.t. X
% varnlcneq2JX_strc=sym([]);
varnlcneq2JX_strc=[Jac_Fg_X_strc;Jac_qSeg_X_strc];
varnlcneqJX_cons =[eye(N_X); varnlcneq2JX_strc];
nlcneqJX_strc=TLmatstrc(nlcneqJv_strc*varnlcneqJX_cons);

%% Compute varceq Jac w.r.t. X
varnlceq2JX_strc=[ Jac_adot_X_strc;Jac_Eq_X_strc];
varnlceqJX_strc =[eye(N_X);varnlceq2JX_strc];
nlceqJX_strc=TLmatstrc(nlceqJv_strc*varnlceqJX_strc);

%% Combine c, ceq for IPOPT
nlcJX_strc=[nlcneqJX_strc;nlceqJX_strc];

t_end = datetime('now','format','yyyyMMdd_HHmmss');
t_dur=t_end-t_start


%% save 
save([Grad_FolderName,'\','Grad_CONS.mat'],...
    'N_step','N_q','N_u','N_g','N_var','N_X',...
    'qJac_X','qdotJac_X','qddotJac_X','qqdotJac_X','qqdotqddotJac_X',...
    'aJac_X','uJac_X','auJac_X',...
    'N_nlcneq','N_nlceq','N_lcneq','N_lceq',...
    'lcneqAnz_row','lcneqAnz_col','lcneqA_size','lcneqA_strc',...
    'lceqAnz_row' ,'lceqAnz_col' ,'lceqA_size' ,'lceqA_strc',...
    'nlcneqJvnz_row','nlcneqJvnz_col','nlcneqJv_size','nlcneqJv_strc','nlcneqJX_strc',...
    'nlceqJvnz_row' ,'nlceqJvnz_col' ,'nlceqJv_size' ,'nlceqJv_strc' ,'nlceqJX_strc' ,...
    'nlcJX_strc',...
    'substoc_obj','substoc_nlcneq','substoc_nlceq',...
    't_start','t_end','t_dur');

return;















