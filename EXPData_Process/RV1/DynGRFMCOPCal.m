function GRFMCOP=DynGRFMCOPCal(Res,aux)

[Devicenum,~,~]=xlsread(aux.DynDevice_FileName);

if mean(Devicenum(5:end,10))<mean(Devicenum(5:end,10+9)) % left on is F1, right is F2
    id1=4;
    id2=4+9;
else
    id2=4;
    id1=4+9;
end

F1=Devicenum(5:end, id1   :(id1+2));
M1=Devicenum(5:end,(id1+3):(id1+5))/1000;
P1=Devicenum(5:end,(id1+6):(id1+8))/1000;

F2=Devicenum(5:end, id2   :(id2+2));
M2=Devicenum(5:end,(id2+3):(id2+5))/1000;
P2=Devicenum(5:end,(id2+6):(id2+8))/1000;

%%

F1_FP=((aux.Tmat_LFP(1:3,1:3)^-1)*F1.').';
M1_FP=((aux.Tmat_LFP(1:3,1:3)^-1)*M1.').';
P1_FP_vic=((aux.Tmat_LFP(1:3,1:3)^-1)*(P1.'-aux.Tmat_LFP(1:3,4))).';


Px=(-M1_FP(:,2))./F1_FP(:,3); Px(isnan(Px))=0;
Py=(+M1_FP(:,1))./F1_FP(:,3); Py(isnan(Py))=0;
Pz=0*Px; 
Tx=0*Px;
Ty=0*Px;
Tz=M1_FP(:,3)+F1_FP(:,1).*Py-F1_FP(:,2).*Px;
P1_FP=[Px,Py,Pz];
T1_FP=[Tx,Ty,Tz];
ErrP1=P1_FP-P1_FP_vic;
if max(ErrP1)>0.01
    disp('ErrorP1 is too large');
end

F1_g=(aux.Tmat_LFP(1:3,1:3)*F1_FP.').';
T1_g=(aux.Tmat_LFP(1:3,1:3)*T1_FP.').';
P1_g=(aux.Tmat_LFP(1:3,1:3)*P1_FP.'+aux.Tmat_LFP(1:3,4)).';

%---------------------------------------
F2_FP=((aux.Tmat_RFP(1:3,1:3)^-1)*F2.').';
M2_FP=((aux.Tmat_RFP(1:3,1:3)^-1)*M2.').';
P2_FP_vic=((aux.Tmat_RFP(1:3,1:3)^-1)*(P2.'-aux.Tmat_RFP(1:3,4))).';

% vicon didn't use the true Z0 value, and simplified into 0.
Px=(-M2_FP(:,2))./F2_FP(:,3); Px(isnan(Px))=0;
Py=(+M2_FP(:,1))./F2_FP(:,3); Py(isnan(Py))=0;
Pz=0*Px;
Tx=0*Px;
Ty=0*Px;
Tz=M2_FP(:,3)+F2_FP(:,1).*Py-F2_FP(:,2).*Px;
P2_FP=[Px,Py,Pz];
T2_FP=[Tx,Ty,Tz];
ErrP2=P2_FP-P2_FP_vic;
if max(ErrP2)>0.01
    disp('ErrorP2 is too large');
end

F2_g=(aux.Tmat_RFP_true(1:3,1:3)*F2_FP.').';
T2_g=(aux.Tmat_RFP_true(1:3,1:3)*T2_FP.').';
P2_g=(aux.Tmat_RFP_true(1:3,1:3)*P2_FP.'+aux.Tmat_RFP_true(1:3,4)).';

%%
if aux.plotGRFCOP==1
figure;hold on; 
set(gcf,'defaultLineLineWidth',1.5);
for i=1:3
    subplot(3,3,i);hold on;title('F')
    plot(F1(:,i),'b');
    plot(F2(:,i),'g');
    plot(F1_g(:,i),'c--');
    plot(F2_g(:,i),'m--');
    subplot(3,3,i+3);hold on;title('M/T')
    plot(M1(:,i),'b');
    plot(M2(:,i),'g');
    plot(T1_g(:,i),'c--');
    plot(T2_g(:,i),'m--');
    
    subplot(3,3,i+6);hold on;title('P')
    plot(P1(:,i),'b');
    plot(P2(:,i),'g');
    plot(P1_g(:,i),'c--');
    plot(P2_g(:,i),'m--');

end
end
%%
rat=aux.ForceFrameRate/aux.TrajFrameRate;

cycle=[floor(Res.Events.idx_HS1/aux.ForceFrameRate*aux.TrajFrameRate)*rat :rat: floor(Res.Events.idx_HS2/aux.ForceFrameRate*aux.TrajFrameRate)*rat];

%%
GRFMCOP.F1_g=-F1_g(cycle,:);
GRFMCOP.T1_g=-T1_g(cycle,:);
GRFMCOP.P1_g=P1_g(cycle,:);

GRFMCOP.F2_g=-F2_g(cycle,:);
GRFMCOP.T2_g=-T2_g(cycle,:);
GRFMCOP.P2_g=P2_g(cycle,:);







