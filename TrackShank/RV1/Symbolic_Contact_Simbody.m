%%
clear;clc;
syms R k c ud V 
syms y ydot xdot

%%

eps = 1e-5;
eps2 = 1e-16;
bv = 50;
bd = 300; 

d=R-y;
ddot=-ydot;

k_s = 0.5*(k)^(2/3);

F_Hz = (4/3)*k_s*sqrt(R*k_s)*((sqrt(d*d+eps))^(3/2));

F_D = F_Hz*(1+1.5*c*ddot); % dissipation

Fy = (0.5*tanh(bv*(ddot+1/(1.5*c)))+0.5 + eps2)*...
    (0.5*tanh(bd*d)+0.5 + eps2)*F_D;

Fx=-Fy.*ud.*tanh(xdot./V);

JT_Fy=jacobian(Fy,[y,ydot]).'; % transpose of jacobian
JT_Fx=jacobian(Fx,[y,ydot,xdot]).';

symvar(JT_Fy)
symvar(JT_Fx)
%%

path(pathdef);
FolderName='AutoGeneFuns';
if ~exist(FolderName,'dir')
    mkdir(FolderName);
end
FN_pre=[FolderName,'/Fun_'];

matlabFunction(Fy   ,'file',[FN_pre,'Contact_Sim_Fy']   ,'Vars',{R,k,c,     y,ydot}); 
matlabFunction(Fx   ,'file',[FN_pre,'Contact_Sim_Fx']   ,'Vars',{R,k,c,ud,V,y,ydot,xdot}); 
matlabFunction(JT_Fy,'file',[FN_pre,'Contact_Sim_JT_Fy'],'Vars',{R,k,c,     y,ydot}); 
matlabFunction(JT_Fx,'file',[FN_pre,'Contact_Sim_JT_Fx'],'Vars',{R,k,c,ud,V,y,ydot,xdot}); 

return;







