% Kinematics and Kinemtics of the Joint Model
clear;clc; 

%%
path(pathdef);
addpath('./TLFuns')
FolderName='AutoGeneFuns';

if ~exist(FolderName,'dir')
    mkdir(FolderName);
end
FN_pre=[FolderName,'/Fun_'];

%%
syms g
syms mb mt ms mf
syms Lb  Lt  Ls  Lf
syms Lbm Ltm Lsm Lfm
syms Ib  It  Is  If

syms dhex dhey dtox dtoy dfmx dfmy

syms xb xbdot xbddot
syms yb ybdot ybddot
syms qb qbdot qbddot

syms qrh qrhdot qrhddot
syms qrk qrkdot qrkddot
syms qra qradot qraddot

syms qlh qlhdot qlhddot
syms qlk qlkdot qlkddot
syms qla qladot qladdot

N_cts=5;
% position of contact spheres
ctsx=sym('ctsx',[1,N_cts]);
ctsy=sym('ctsy',[1,N_cts]);

%%
CON_Mass=[mb,mt,ms,mf,Ib,It,Is,If,g];
CON_Len=[Lb, Lbm, Lt, Ltm, Ls, Lsm, dhex,dhey,dtox,dtoy,dfmx, dfmy];

q_ske    =[xb    ,yb    ,qb    ,qrh    ,qrk    ,qra    ,qlh    ,qlk    ,qla    ];
qdot_ske =[xbdot ,ybdot ,qbdot ,qrhdot ,qrkdot ,qradot ,qlhdot ,qlkdot ,qladot ];
qddot_ske=[xbddot,ybddot,qbddot,qrhddot,qrkddot,qraddot,qlhddot,qlkddot,qladdot];

N_q_ske  =length(q_ske);
%%

qrt=qb +qrh;
qrs=qrt+qrk;
qrf=qrs+qra;

qlt=qb +qlh;
qls=qlt+qlk;
qlf=qls+qla;

qrtdot=TLdot(qrt,q_ske,qdot_ske);
qrsdot=TLdot(qrs,q_ske,qdot_ske);
qrfdot=TLdot(qrf,q_ske,qdot_ske);
qltdot=TLdot(qlt,q_ske,qdot_ske);
qlsdot=TLdot(qls,q_ske,qdot_ske);
qlfdot=TLdot(qlf,q_ske,qdot_ske);


qrtddot=TLddot(qrt,q_ske,qdot_ske,qddot_ske);
qrsddot=TLddot(qrs,q_ske,qdot_ske,qddot_ske);
qrfddot=TLddot(qrf,q_ske,qdot_ske,qddot_ske);
qltddot=TLddot(qlt,q_ske,qdot_ske,qddot_ske);
qlsddot=TLddot(qls,q_ske,qdot_ske,qddot_ske);
qlfddot=TLddot(qlf,q_ske,qdot_ske,qddot_ske);

qSeg=[qrt;qrs;qrf;qlt;qls;qlf];
qdotSeg=[qrtdot;qrsdot;qrfdot;qltdot;qlsdot;qlfdot];
qddotSeg=[qrtddot;qrsddot;qrfddot;qltddot;qlsddot;qlfddot];

%%
matlabFunction(qrt ,'file',[FN_pre,'qrt'] ,'Vars',{q_ske}); 
matlabFunction(qrs ,'file',[FN_pre,'qrs'] ,'Vars',{q_ske}); 
matlabFunction(qrf ,'file',[FN_pre,'qrf'] ,'Vars',{q_ske}); 
matlabFunction(qlt ,'file',[FN_pre,'qlt'] ,'Vars',{q_ske}); 
matlabFunction(qls ,'file',[FN_pre,'qls'] ,'Vars',{q_ske}); 
matlabFunction(qlf ,'file',[FN_pre,'qlf'] ,'Vars',{q_ske}); 
matlabFunction(qSeg ,'file',[FN_pre,'qSeg'] ,'Vars',{q_ske});

matlabFunction(qrtdot ,'file',[FN_pre,'qrtdot'] ,'Vars',{qdot_ske}); 
matlabFunction(qrsdot ,'file',[FN_pre,'qrsdot'] ,'Vars',{qdot_ske}); 
matlabFunction(qrfdot ,'file',[FN_pre,'qrfdot'] ,'Vars',{qdot_ske}); 
matlabFunction(qltdot ,'file',[FN_pre,'qltdot'] ,'Vars',{qdot_ske}); 
matlabFunction(qlsdot ,'file',[FN_pre,'qlsdot'] ,'Vars',{qdot_ske}); 
matlabFunction(qlfdot ,'file',[FN_pre,'qlfdot'] ,'Vars',{qdot_ske}); 
matlabFunction(qdotSeg ,'file',[FN_pre,'qdotSeg'] ,'Vars',{qdot_ske});

matlabFunction(qrtddot ,'file',[FN_pre,'qrtddot'] ,'Vars',{qddot_ske}); 
matlabFunction(qrsddot ,'file',[FN_pre,'qrsddot'] ,'Vars',{qddot_ske}); 
matlabFunction(qrfddot ,'file',[FN_pre,'qrfddot'] ,'Vars',{qddot_ske}); 
matlabFunction(qltddot ,'file',[FN_pre,'qltddot'] ,'Vars',{qddot_ske}); 
matlabFunction(qlsddot ,'file',[FN_pre,'qlsddot'] ,'Vars',{qddot_ske}); 
matlabFunction(qlfddot ,'file',[FN_pre,'qlfddot'] ,'Vars',{qddot_ske}); 
matlabFunction(qddotSeg ,'file',[FN_pre,'qddotSeg'] ,'Vars',{qddot_ske});

%%

J_qSeg=jacobian(qSeg,q_ske);
J_qdotSeg=jacobian(qdotSeg,[q_ske,qdot_ske]);
J_qddotSeg=jacobian(qddotSeg,[q_ske,qdot_ske,qddot_ske]);

J_qSeg_strc=TLmatstrc(J_qSeg); % structure of the jacobian matrix
J_qdotSeg_strc=TLmatstrc(J_qdotSeg);
J_qddotSeg_strc=TLmatstrc(J_qddotSeg);

%%

matlabFunction(J_qSeg,'file',[FN_pre,'J_qSeg'],'Vars',{q_ske});
matlabFunction(J_qdotSeg,'file',[FN_pre,'J_qdotSeg'],'Vars',{qdot_ske}); 
matlabFunction(J_qddotSeg,'file',[FN_pre,'J_qddotSeg'],'Vars',{qddot_ske}); 

%%
xn=xb+TLrot2D(qb,1)*[0;Lb];
yn=yb+TLrot2D(qb,2)*[0;Lb];
xbm=xb+TLrot2D(qb,1)*[0;Lbm];
ybm=yb+TLrot2D(qb,2)*[0;Lbm];

xrh=xb;
yrh=yb;
xrk=xb+TLrot2D(qrt,1)*[0;-Lt];
yrk=yb+TLrot2D(qrt,2)*[0;-Lt];
xra=xrk+TLrot2D(qrs,1)*[0;-Ls];
yra=yrk+TLrot2D(qrs,2)*[0;-Ls];

xrtm=xb+TLrot2D(qrt,1)*[0;-Ltm];
yrtm=yb+TLrot2D(qrt,2)*[0;-Ltm];
xrsm=xrk+TLrot2D(qrs,1)*[0;-Lsm];
yrsm=yrk+TLrot2D(qrs,2)*[0;-Lsm];

xrfm=xra+TLrot2D(qrf,1)*[dfmx;dfmy];
yrfm=yra+TLrot2D(qrf,2)*[dfmx;dfmy];
xrhe=xra+TLrot2D(qrf,1)*[dhex;dhey];
yrhe=yra+TLrot2D(qrf,2)*[dhex;dhey];
xrto=xra+TLrot2D(qrf,1)*[dtox;dtoy];
yrto=yra+TLrot2D(qrf,2)*[dtox;dtoy];
xrcts=xra+TLrot2D(qrf,1)*[ctsx;ctsy];
yrcts=yra+TLrot2D(qrf,2)*[ctsx;ctsy];

xlh=xb;
ylh=yb;
xlk=xb+TLrot2D(qlt,1)*[0;-Lt];
ylk=yb+TLrot2D(qlt,2)*[0;-Lt];
xla=xlk+TLrot2D(qls,1)*[0;-Ls];
yla=ylk+TLrot2D(qls,2)*[0;-Ls];

xltm=xb+TLrot2D(qlt,1)*[0;-Ltm];
yltm=yb+TLrot2D(qlt,2)*[0;-Ltm];
xlsm=xlk+TLrot2D(qls,1)*[0;-Lsm];
ylsm=ylk+TLrot2D(qls,2)*[0;-Lsm];

xlfm=xla+TLrot2D(qlf,1)*[dfmx;dfmy];
ylfm=yla+TLrot2D(qlf,2)*[dfmx;dfmy];
xlhe=xla+TLrot2D(qlf,1)*[dhex;dhey];
ylhe=yla+TLrot2D(qlf,2)*[dhex;dhey];
xlto=xla+TLrot2D(qlf,1)*[dtox;dtoy];
ylto=yla+TLrot2D(qlf,2)*[dtox;dtoy];
xlcts=xla+TLrot2D(qlf,1)*[ctsx;ctsy];
ylcts=yla+TLrot2D(qlf,2)*[ctsx;ctsy];

pJoi=[xrh,yrh,xrk,yrk,xra,yra, xlh,ylh,xlk,ylk,xla,yla];

%%

matlabFunction(xrh,'file',[FN_pre,'xrh'],'Vars',{q_ske,CON_Len}); 
matlabFunction(yrh,'file',[FN_pre,'yrh'],'Vars',{q_ske,CON_Len}); 
matlabFunction(xrk,'file',[FN_pre,'xrk'],'Vars',{q_ske,CON_Len}); 
matlabFunction(yrk,'file',[FN_pre,'yrk'],'Vars',{q_ske,CON_Len}); 
matlabFunction(xra,'file',[FN_pre,'xra'],'Vars',{q_ske,CON_Len}); 
matlabFunction(yra,'file',[FN_pre,'yra'],'Vars',{q_ske,CON_Len}); 

matlabFunction(xlh,'file',[FN_pre,'xlh'],'Vars',{q_ske,CON_Len}); 
matlabFunction(ylh,'file',[FN_pre,'ylh'],'Vars',{q_ske,CON_Len}); 
matlabFunction(xlk,'file',[FN_pre,'xlk'],'Vars',{q_ske,CON_Len}); 
matlabFunction(ylk,'file',[FN_pre,'ylk'],'Vars',{q_ske,CON_Len}); 
matlabFunction(xla,'file',[FN_pre,'xla'],'Vars',{q_ske,CON_Len}); 
matlabFunction(yla,'file',[FN_pre,'yla'],'Vars',{q_ske,CON_Len}); 

matlabFunction(pJoi,'file',[FN_pre,'posiJoi'],'Vars',{q_ske,CON_Len}); 


matlabFunction(xrtm,'file',[FN_pre,'xrtm'],'Vars',{q_ske,CON_Len}); 
matlabFunction(yrtm,'file',[FN_pre,'yrtm'],'Vars',{q_ske,CON_Len}); 
matlabFunction(xrsm,'file',[FN_pre,'xrsm'],'Vars',{q_ske,CON_Len}); 
matlabFunction(yrsm,'file',[FN_pre,'yrsm'],'Vars',{q_ske,CON_Len}); 
matlabFunction(xrfm,'file',[FN_pre,'xrfm'],'Vars',{q_ske,CON_Len}); 
matlabFunction(yrfm,'file',[FN_pre,'yrfm'],'Vars',{q_ske,CON_Len}); 

matlabFunction(xrhe,'file',[FN_pre,'xrhe'],'Vars',{q_ske,CON_Len}); 
matlabFunction(yrhe,'file',[FN_pre,'yrhe'],'Vars',{q_ske,CON_Len}); 
matlabFunction(xrto,'file',[FN_pre,'xrto'],'Vars',{q_ske,CON_Len}); 
matlabFunction(yrto,'file',[FN_pre,'yrto'],'Vars',{q_ske,CON_Len}); 


matlabFunction(xltm,'file',[FN_pre,'xltm'],'Vars',{q_ske,CON_Len}); 
matlabFunction(yltm,'file',[FN_pre,'yltm'],'Vars',{q_ske,CON_Len}); 
matlabFunction(xlsm,'file',[FN_pre,'xlsm'],'Vars',{q_ske,CON_Len}); 
matlabFunction(ylsm,'file',[FN_pre,'ylsm'],'Vars',{q_ske,CON_Len}); 
matlabFunction(xlfm,'file',[FN_pre,'xlfm'],'Vars',{q_ske,CON_Len}); 
matlabFunction(ylfm,'file',[FN_pre,'ylfm'],'Vars',{q_ske,CON_Len}); 

matlabFunction(xlhe,'file',[FN_pre,'xlhe'],'Vars',{q_ske,CON_Len}); 
matlabFunction(ylhe,'file',[FN_pre,'ylhe'],'Vars',{q_ske,CON_Len}); 
matlabFunction(xlto,'file',[FN_pre,'xlto'],'Vars',{q_ske,CON_Len}); 
matlabFunction(ylto,'file',[FN_pre,'ylto'],'Vars',{q_ske,CON_Len}); 

matlabFunction(xn,'file',[FN_pre,'xn'],'Vars',{q_ske,CON_Len}); 
matlabFunction(yn,'file',[FN_pre,'yn'],'Vars',{q_ske,CON_Len}); 
matlabFunction(xbm,'file',[FN_pre,'xbm'],'Vars',{q_ske,CON_Len}); 
matlabFunction(ybm,'file',[FN_pre,'ybm'],'Vars',{q_ske,CON_Len}); 

matlabFunction(xrcts,'file',[FN_pre,'xrcts'],'Vars',{q_ske,CON_Len,ctsx,ctsy}); 
matlabFunction(yrcts,'file',[FN_pre,'yrcts'],'Vars',{q_ske,CON_Len,ctsx,ctsy}); 
matlabFunction(xlcts,'file',[FN_pre,'xlcts'],'Vars',{q_ske,CON_Len,ctsx,ctsy}); 
matlabFunction(ylcts,'file',[FN_pre,'ylcts'],'Vars',{q_ske,CON_Len,ctsx,ctsy}); 

%%

pdotJoi=TLdot(pJoi,q_ske,qdot_ske).';
pddotJoi=TLddot(pJoi,q_ske,qdot_ske,qddot_ske).';

J_pJoi=jacobian(pJoi,q_ske);
J_pdotJoi=jacobian(pdotJoi,[q_ske,qdot_ske]);
J_pddotJoi=jacobian(pddotJoi,[q_ske,qdot_ske,qddot_ske]);

% structure of the jacobian matrix

J_pJoi_strc=TLmatstrc(J_pJoi); 
J_pdotJoi_strc=TLmatstrc(J_pdotJoi); 
J_pddotJoi_strc=TLmatstrc(J_pddotJoi); 
%%

matlabFunction(pdotJoi,'file',[FN_pre,'posidotJoi'],'Vars',{q_ske,qdot_ske,CON_Len}); 
matlabFunction(pddotJoi,'file',[FN_pre,'posiddotJoi'],'Vars',{q_ske,qdot_ske,qddot_ske,CON_Len}); 

matlabFunction(J_pJoi,'file',[FN_pre,'J_posiJoi'],'Vars',{q_ske,CON_Len});
matlabFunction(J_pdotJoi,'file',[FN_pre,'J_posidotJoi'],'Vars',{q_ske,qdot_ske,CON_Len});
matlabFunction(J_pddotJoi,'file',[FN_pre,'J_posiddotJoi'],'Vars',{q_ske,qdot_ske,qddot_ske,CON_Len});
% return;


%% ddot

xlctsdot=TLdot(xlcts,q_ske,qdot_ske).';
ylctsdot=TLdot(ylcts,q_ske,qdot_ske).';
xrctsdot=TLdot(xrcts,q_ske,qdot_ske).';
yrctsdot=TLdot(yrcts,q_ske,qdot_ske).';

%%

matlabFunction(xrctsdot,'file',[FN_pre,'xrctsdot'],'Vars',{q_ske,qdot_ske,CON_Len,ctsx,ctsy}); 
matlabFunction(yrctsdot,'file',[FN_pre,'yrctsdot'],'Vars',{q_ske,qdot_ske,CON_Len,ctsx,ctsy}); 
matlabFunction(xlctsdot,'file',[FN_pre,'xlctsdot'],'Vars',{q_ske,qdot_ske,CON_Len,ctsx,ctsy}); 
matlabFunction(ylctsdot,'file',[FN_pre,'ylctsdot'],'Vars',{q_ske,qdot_ske,CON_Len,ctsx,ctsy});

%%
xbmddot=TLddot(xbm,q_ske,qdot_ske,qddot_ske);
ybmddot=TLddot(ybm,q_ske,qdot_ske,qddot_ske);

xrtmddot=TLddot(xrtm,q_ske,qdot_ske,qddot_ske);
yrtmddot=TLddot(yrtm,q_ske,qdot_ske,qddot_ske);
xrsmddot=TLddot(xrsm,q_ske,qdot_ske,qddot_ske);
yrsmddot=TLddot(yrsm,q_ske,qdot_ske,qddot_ske);
xrfmddot=TLddot(xrfm,q_ske,qdot_ske,qddot_ske);
yrfmddot=TLddot(yrfm,q_ske,qdot_ske,qddot_ske);

xltmddot=TLddot(xltm,q_ske,qdot_ske,qddot_ske);
yltmddot=TLddot(yltm,q_ske,qdot_ske,qddot_ske);
xlsmddot=TLddot(xlsm,q_ske,qdot_ske,qddot_ske);
ylsmddot=TLddot(ylsm,q_ske,qdot_ske,qddot_ske);
xlfmddot=TLddot(xlfm,q_ske,qdot_ske,qddot_ske);
ylfmddot=TLddot(ylfm,q_ske,qdot_ske,qddot_ske);

Fx_tot=xbmddot*mb+xrtmddot*mt+xrsmddot*ms+xrfmddot*mf+...
                  xltmddot*mt+xlsmddot*ms+xlfmddot*mf;
Fy_tot=(ybmddot+g)*mb+(yrtmddot+g)*mt+(yrsmddot+g)*ms+(yrfmddot+g)*mf+...
                      (yltmddot+g)*mt+(ylsmddot+g)*ms+(ylfmddot+g)*mf;
              
%%

matlabFunction(xbmddot,'file',[FN_pre,'xbmddot'],'Vars',{q_ske,qdot_ske,qddot_ske,CON_Len}); 
matlabFunction(ybmddot,'file',[FN_pre,'ybmddot'],'Vars',{q_ske,qdot_ske,qddot_ske,CON_Len}); 

matlabFunction(xrtmddot,'file',[FN_pre,'xrtmddot'],'Vars',{q_ske,qdot_ske,qddot_ske,CON_Len}); 
matlabFunction(yrtmddot,'file',[FN_pre,'yrtmddot'],'Vars',{q_ske,qdot_ske,qddot_ske,CON_Len}); 
matlabFunction(xrsmddot,'file',[FN_pre,'xrsmddot'],'Vars',{q_ske,qdot_ske,qddot_ske,CON_Len}); 
matlabFunction(yrsmddot,'file',[FN_pre,'yrsmddot'],'Vars',{q_ske,qdot_ske,qddot_ske,CON_Len}); 
matlabFunction(xrfmddot,'file',[FN_pre,'xrfmddot'],'Vars',{q_ske,qdot_ske,qddot_ske,CON_Len}); 
matlabFunction(yrfmddot,'file',[FN_pre,'yrfmddot'],'Vars',{q_ske,qdot_ske,qddot_ske,CON_Len}); 

matlabFunction(xltmddot,'file',[FN_pre,'xltmddot'],'Vars',{q_ske,qdot_ske,qddot_ske,CON_Len}); 
matlabFunction(yltmddot,'file',[FN_pre,'yltmddot'],'Vars',{q_ske,qdot_ske,qddot_ske,CON_Len}); 
matlabFunction(xlsmddot,'file',[FN_pre,'xlsmddot'],'Vars',{q_ske,qdot_ske,qddot_ske,CON_Len}); 
matlabFunction(ylsmddot,'file',[FN_pre,'ylsmddot'],'Vars',{q_ske,qdot_ske,qddot_ske,CON_Len}); 
matlabFunction(xlfmddot,'file',[FN_pre,'xlfmddot'],'Vars',{q_ske,qdot_ske,qddot_ske,CON_Len}); 
matlabFunction(ylfmddot,'file',[FN_pre,'ylfmddot'],'Vars',{q_ske,qdot_ske,qddot_ske,CON_Len}); 

matlabFunction(Fx_tot,'file',[FN_pre,'Fx_tot'],'Vars',{q_ske,qdot_ske,qddot_ske,CON_Mass,CON_Len}); 
matlabFunction(Fy_tot,'file',[FN_pre,'Fy_tot'],'Vars',{q_ske,qdot_ske,qddot_ske,CON_Mass,CON_Len}); 

%%
J_xlcts=jacobian(xlcts,q_ske);
J_ylcts=jacobian(ylcts,q_ske);
J_xrcts=jacobian(xrcts,q_ske);
J_yrcts=jacobian(yrcts,q_ske);

J_xlctsdot=jacobian(xlctsdot,[q_ske,qdot_ske]);
J_ylctsdot=jacobian(ylctsdot,[q_ske,qdot_ske]);
J_xrctsdot=jacobian(xrctsdot,[q_ske,qdot_ske]);
J_yrctsdot=jacobian(yrctsdot,[q_ske,qdot_ske]);

J_xlcts_strc=TLmatstrc(J_xlcts); % structure of the jacobian matrix
J_ylcts_strc=TLmatstrc(J_ylcts);
J_xrcts_strc=TLmatstrc(J_xrcts);
J_yrcts_strc=TLmatstrc(J_yrcts);
J_xlctsdot_strc=TLmatstrc(J_xlctsdot);
J_ylctsdot_strc=TLmatstrc(J_ylctsdot);
J_xrctsdot_strc=TLmatstrc(J_xrctsdot);
J_yrctsdot_strc=TLmatstrc(J_yrctsdot);

%%
matlabFunction(J_xrcts,'file',[FN_pre,'J_xrcts'],'Vars',{q_ske,CON_Len,ctsx,ctsy}); 
matlabFunction(J_yrcts,'file',[FN_pre,'J_yrcts'],'Vars',{q_ske,CON_Len,ctsx,ctsy}); 
matlabFunction(J_xlcts,'file',[FN_pre,'J_xlcts'],'Vars',{q_ske,CON_Len,ctsx,ctsy}); 
matlabFunction(J_ylcts,'file',[FN_pre,'J_ylcts'],'Vars',{q_ske,CON_Len,ctsx,ctsy}); 

matlabFunction(J_xrctsdot,'file',[FN_pre,'J_xrctsdot'],'Vars',{q_ske,qdot_ske,CON_Len,ctsx,ctsy}); 
matlabFunction(J_yrctsdot,'file',[FN_pre,'J_yrctsdot'],'Vars',{q_ske,qdot_ske,CON_Len,ctsx,ctsy}); 
matlabFunction(J_xlctsdot,'file',[FN_pre,'J_xlctsdot'],'Vars',{q_ske,qdot_ske,CON_Len,ctsx,ctsy}); 
matlabFunction(J_ylctsdot,'file',[FN_pre,'J_ylctsdot'],'Vars',{q_ske,qdot_ske,CON_Len,ctsx,ctsy}); 


%% compute GRF

cts_FRx=sym('cts_FRx',[1,N_cts]);
cts_FRy=sym('cts_FRy',[1,N_cts]);
cts_FLx=sym('cts_FLx',[1,N_cts]);
cts_FLy=sym('cts_FLy',[1,N_cts]);

Frgx=sum(cts_FRx);
Frgy=sum(cts_FRy);
Flgx=sum(cts_FLx);
Flgy=sum(cts_FLy);

Mrg=sum(cts_FRx.*(yrfm-0)+cts_FRy.*(xrcts-xrfm));
Mlg=sum(cts_FLx.*(ylfm-0)+cts_FLy.*(xlcts-xlfm));

%%
matlabFunction(Frgx,'file',[FN_pre,'Frgx'],'Vars',{cts_FRx,cts_FRy,cts_FLx,cts_FLy}); 
matlabFunction(Frgy,'file',[FN_pre,'Frgy'],'Vars',{cts_FRx,cts_FRy,cts_FLx,cts_FLy}); 
matlabFunction(Flgx,'file',[FN_pre,'Flgx'],'Vars',{cts_FRx,cts_FRy,cts_FLx,cts_FLy}); 
matlabFunction(Flgy,'file',[FN_pre,'Flgy'],'Vars',{cts_FRx,cts_FRy,cts_FLx,cts_FLy}); 
matlabFunction(Mrg,'file',[FN_pre,'Mrg'],'Vars',{cts_FRx,cts_FRy,ctsx,ctsy,q_ske,CON_Len}); 
matlabFunction(Mlg,'file',[FN_pre,'Mlg'],'Vars',{cts_FLx,cts_FLy,ctsx,ctsy,q_ske,CON_Len}); 

%%
syms Trh Trk Tra Tlh Tlk Tla
T_Joi=[Trh,Trk,Tra,Tlh,Tlk,Tla];
P_Joi=[Trh.*(-qrhdot),Trk.*(-qrkdot),Tra.*(-qradot),...
       Tlh.*(-qlhdot),Tlk.*(-qlkdot),Tla.*(-qladot)];

J_PJoi=jacobian(P_Joi,[qdot_ske,T_Joi]);
J_PJoi_strc=TLmatstrc(J_PJoi);

N_T_Joi  =length(T_Joi);
%%
matlabFunction(P_Joi,'file',[FN_pre,'P_Joi'],'Vars',{qdot_ske,T_Joi}); 
matlabFunction(J_PJoi,'file',[FN_pre,'J_PJoi'],'Vars',{qdot_ske,T_Joi}); 

%%
Frax=Frgx-mf*xrfmddot;
Fray=Frgy-mf*yrfmddot-mf*g;
Flax=Flgx-mf*xlfmddot;
Flay=Flgy-mf*ylfmddot-mf*g;


Frkx=Frax-ms*xrsmddot;
Frky=Fray-ms*yrsmddot-ms*g;
Flkx=Flax-ms*xlsmddot;
Flky=Flay-ms*ylsmddot-ms*g;

Frhx=Frkx-mt*xrtmddot;
Frhy=Frky-mt*yrtmddot-mt*g;
Flhx=Flkx-mt*xltmddot;
Flhy=Flky-mt*yltmddot-mt*g;
F_Joi=[Frax,Fray,Flax,Flay,  Frkx,Frky,Flkx,Flky,  Frhx,Frhy,Flhx,Flhy];
%%

matlabFunction(F_Joi,'file',[FN_pre,'F_Joi'],'Vars',{CON_Mass,CON_Len,...
    ctsx,ctsy,q_ske,qdot_ske,qddot_ske,cts_FRx,cts_FRy,cts_FLx,cts_FLy});

%%

Eq1=Frhx+Flhx-mb*xbmddot;
Eq2=Frhy+Flhy-mb*ybmddot-mb*g;

Eq3=Frhx*(ybm-yb)+Flhx*(ybm-yb)+Frhy*(xb-xbm)+Flhy*(xb-xbm)+Tlh+Trh-Ib*qbddot;

Eq4=Frkx*(yrtm-yrk)+Frky*(xrk-xrtm)+Frhx*(yb-yrtm)+Frhy*(xrtm-xb)+Trk-Trh-It*qrtddot;
Eq5=Flkx*(yltm-ylk)+Flky*(xlk-xltm)+Flhx*(yb-yltm)+Flhy*(xltm-xb)+Tlk-Tlh-It*qltddot;

Eq6=Frax*(yrsm-yra)+Fray*(xra-xrsm)+Frkx*(yrk-yrsm)+Frky*(xrsm-xrk)+Tra-Trk-Is*qrsddot;
Eq7=Flax*(ylsm-yla)+Flay*(xla-xlsm)+Flkx*(ylk-ylsm)+Flky*(xlsm-xlk)+Tla-Tlk-Is*qlsddot;

Eq8=Mrg+Frax*(yra-yrfm)+Fray*(xrfm-xra)-Tra-If*qrfddot;
Eq9=Mlg+Flax*(yla-ylfm)+Flay*(xlfm-xla)-Tla-If*qlfddot;

Eq=[Eq1;Eq2;Eq3;Eq4;Eq6;Eq8;Eq5;Eq7;Eq9];
% symvar(Eq)

%%
Vars_Eq=[q_ske,qdot_ske,qddot_ske,T_Joi,cts_FRx,cts_FRy,cts_FLx,cts_FLy];
J_Eq=jacobian(Eq,Vars_Eq);
[J_Eq_nzrow,J_Eq_nzcol,J_Eq_nzval]=find(J_Eq);
J_Eq_size=size(J_Eq);
% symvar(J_Eq_nzval).' 

J_Eq_strc=TLmatstrc(J_Eq); % structure of the jacobian matrix

%%
matlabFunction(Eq,'file',[FN_pre,'Eq'],'Vars',...
    {CON_Mass,CON_Len,ctsx,ctsy,q_ske,qdot_ske,qddot_ske,T_Joi,cts_FRx,cts_FRy,cts_FLx,cts_FLy});
matlabFunction(J_Eq_nzval,'file',[FN_pre,'J_Eq_nzval'],'Vars',...
    {CON_Mass,CON_Len,ctsx,ctsy,q_ske,qdot_ske,qddot_ske,T_Joi,cts_FRx,cts_FRy,cts_FLx,cts_FLy});


%%
save([FolderName,'/EOM_CONS.mat'],...
        'CON_Mass','CON_Len',...
        'N_cts','N_q_ske','N_T_Joi',...
        'J_qSeg_strc','J_qdotSeg_strc','J_qddotSeg_strc',...
        'J_pJoi_strc','J_pdotJoi_strc','J_pddotJoi_strc',...
        'J_xlcts_strc','J_ylcts_strc','J_xrcts_strc','J_yrcts_strc',...
        'J_xlctsdot_strc','J_ylctsdot_strc','J_xrctsdot_strc','J_yrctsdot_strc',...
        'J_PJoi_strc',...
        'J_Eq_nzrow','J_Eq_nzcol','J_Eq_size','J_Eq_strc');

















