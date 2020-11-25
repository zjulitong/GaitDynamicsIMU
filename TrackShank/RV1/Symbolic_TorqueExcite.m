
clear;clc;

path(pathdef);
addpath('TLFuns');
FolderName='AutoGeneFuns';

if ~exist(FolderName,'dir')
    mkdir(FolderName);
end
FN_pre=[FolderName,'/Fun_'];

%%
syms urh urk ura ulh ulk ula
syms arh ark ara alh alk ala

syms Trhmax Trkmax Tramax Tlhmax Tlkmax Tlamax 

syms c1 c2

u_Joi=[urh urk ura ulh ulk ula];
a_Joi=[arh ark ara alh alk ala];
TJmax=[Trhmax Trkmax Tramax Tlhmax Tlkmax Tlamax ];
c_exc=[c1 c2];

adot_Joi=(u_Joi-a_Joi).*(c1*u_Joi+c2);

T_Joi=a_Joi.*TJmax;

%%
J_T_Joi=jacobian(T_Joi,a_Joi);
J_adot_Joi=jacobian(adot_Joi,[a_Joi,u_Joi]);

J_T_Joi_strc=TLmatstrc(J_T_Joi);
J_adot_Joi_strc=TLmatstrc(J_adot_Joi);
N_u=length(u_Joi);
%%

matlabFunction(T_Joi,'file',[FN_pre,'T_Joi'],'Vars',{a_Joi,TJmax}); 
matlabFunction(J_T_Joi,'file',[FN_pre,'J_T_Joi'],'Vars',{TJmax}); 

matlabFunction(adot_Joi,'file',[FN_pre,'adot_Joi'],'Vars',{a_Joi,u_Joi,c_exc}); 
matlabFunction(J_adot_Joi,'file',[FN_pre,'J_adot_Joi'],'Vars',{a_Joi,u_Joi,c_exc}); 
%%
save([FolderName,'/Torque_CONS.mat'],...
        'N_u','J_T_Joi_strc','J_adot_Joi_strc');



