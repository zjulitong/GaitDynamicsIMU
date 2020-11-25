clear;clc;

path(pathdef);
addpath('./TLFuns')
FolderName='AutoGeneFuns';

if ~exist(FolderName,'dir')
    mkdir(FolderName);
end
FN_pre=[FolderName,'/Fun_'];

%%
syms pAx pAy qF
syms pAdotx pAdoty qFdot
syms dFx dFy

pFx=pAx+TLrot2D(qF,1)*[dFx;dFy];
pFy=pAy+TLrot2D(qF,2)*[dFx;dFy];

pFdotx=jacobian(pFx,[pAx,qF])*[pAdotx;qFdot];
pFdoty=jacobian(pFy,[pAy,qF])*[pAdoty;qFdot];

%%
matlabFunction(pFx ,'file',[FN_pre,'pFx'] ,'Vars',{pAx,qF,dFx,dFy}); 
matlabFunction(pFy ,'file',[FN_pre,'pFy'] ,'Vars',{pAy,qF,dFx,dFy}); 

matlabFunction(pFdotx ,'file',[FN_pre,'pFdotx'] ,'Vars',{pAx,qF,pAdotx,qFdot,dFx,dFy}); 
matlabFunction(pFdoty ,'file',[FN_pre,'pFdoty'] ,'Vars',{pAy,qF,pAdoty,qFdot,dFx,dFy}); 







