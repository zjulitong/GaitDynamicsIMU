function [c,ceq] = FootID_ConFun(X,auxdata)


%%
c=[];
ceq=[];
%%
% 
% N_cts=auxdata.N_cts;
% 
% 
% %%
% % extract X
% R1       = X(1);
% R5       = X(2);
% yc       = X(3);
% k       = X(4:(4+N_cts-1))*1e4;
% b       = X((4+N_cts):(4+N_cts+N_cts-1));
% v0      = X(4+N_cts+N_cts);
% 
% 
% %%
% 
% dhex=auxdata.PX_HE+R1;
% dhey=-auxdata.PY_AK+R1;
% dtox=auxdata.PX_TO-R5;
% dtoy=-auxdata.PY_AK+R5;
% 
% ctsx =linspace(dhex, dtox,N_cts);
% ctsy =linspace(dhey, dtoy,N_cts);
% ctsR      = ctsy-(ctsy(1)-R1);
% 
% 
% pRAx=auxdata.pAJ(:,1);
% pRAy=auxdata.pAJ(:,2);
% pLAx=auxdata.pAJ(:,3);
% pLAy=auxdata.pAJ(:,4);
% qRF=auxdata.qFO(:,1);
% qLF=auxdata.qFO(:,2);
% 
% 
% yrcts=Fun_pFy(pRAy,qRF,ctsx,ctsy);
% ylcts=Fun_pFy(pLAy,qLF,ctsx,ctsy);
% 
% FrameCountDS=auxdata.FrameCountDS;
% 
% 
% ceq=[];
% c=[]; 








end
























