function Res_Dyn=DynTrajProcess(Res,aux)

%%

[Trajnum,DynTrajtxt,~]=xlsread(aux.DynTraj_FileName);
Trajnum(:,3:3:end)=Trajnum(:,3:3:end)+2.6; % to compensate for the offset of calibration wand.
Trajnum(:,4:3:end)=Trajnum(:,4:3:end)+2.6;
Trajnum(:,5:3:end)=Trajnum(:,5:3:end)-27.7;

DynTraj=Trajnum(5:end,:)/1000;

for i=1:(length(DynTrajtxt(3,:)))
    mkr_name=DynTrajtxt{3,i};
    if ~isempty(mkr_name)
        chid=strfind(mkr_name,':');
        mkr_name=mkr_name((chid+1):end);
        eval(['aux.idx_dyn.',mkr_name,'=',num2str(i),';']);
    end
end
if isfield(aux.idx_dyn,'RHEEL')
    aux.idx_dyn.RHEE=aux.idx_dyn.RHEEL;
end
if isfield(aux.idx_dyn,'LHEEL')
    aux.idx_dyn.LHEE=aux.idx_dyn.LHEEL;
end
%%
idx_HS1=floor(Res.Events.idx_HS1/aux.ForceFrameRate*aux.TrajFrameRate);
idx_TO1=floor(Res.Events.idx_TO1/aux.ForceFrameRate*aux.TrajFrameRate);
idx_HS2=floor(Res.Events.idx_HS2/aux.ForceFrameRate*aux.TrajFrameRate);
%%
idx=aux.idx_dyn;

RHEE_HS1=DynTraj(idx_HS1,idx.RHEE+(0:2));
LHEE_HS1=DynTraj(idx_HS1,idx.LHEE+(0:2));

RHEE_HS2=DynTraj(idx_HS2,idx.RHEE+(0:2));
LHEE_HS2=DynTraj(idx_HS2,idx.LHEE+(0:2));

Lstep_R=RHEE_HS1(2)-LHEE_HS1(2);
Lstep_L=LHEE_HS2(2)-RHEE_HS2(2);

Res_Dyn.tid_mkr=DynTraj(idx_HS1:idx_HS2,1)*1000;
Res_Dyn.FrameCount  =idx_HS2-idx_HS1+1;
Res_Dyn.FrameCountDS=idx_TO1-idx_HS1+1;
Res_Dyn.Time_step=(idx_HS2-idx_HS1)/aux.TrajFrameRate;
Res_Dyn.Time_DS  =(idx_TO1-idx_HS1)/aux.TrajFrameRate;
Res_Dyn.t_mkr=(0:(idx_HS2-idx_HS1)).'/aux.TrajFrameRate;
DynTraj=DynTraj(idx_HS1:idx_HS2,:);

%%
if nnz(isnan(DynTraj(:,[idx.LASI,idx.LPSI,idx.RPSI,idx.RASI,...
        idx.RHIP,idx.LHIP,idx.RKNE,idx.LKNE,idx.LANK,idx.RANK,...
        idx.LHEE,idx.RHEE,idx.LTOE,idx.RTOE])))>0
    disp(['NaN exists in DynTraj: ',aux.DynTraj_FileName]);
end
% L-R should be negative, P-A should be negative
Dir_Err=[DynTraj(:,idx.LASI+(0))-DynTraj(:,idx.RASI+(0)),DynTraj(:,idx.LPSI+(0))-DynTraj(:,idx.RPSI+(0)),...
         DynTraj(:,idx.LHIP+(0))-DynTraj(:,idx.RHIP+(0)),DynTraj(:,idx.LKNE+(0))-DynTraj(:,idx.RKNE+(0)),...
         DynTraj(:,idx.LANK+(0))-DynTraj(:,idx.RANK+(0)),DynTraj(:,idx.LHEE+(0))-DynTraj(:,idx.RHEE+(0)),...
         DynTraj(:,idx.LTOE+(0))-DynTraj(:,idx.RTOE+(0)),...
         DynTraj(:,idx.LPSI+(1))-DynTraj(:,idx.LASI+(1)),DynTraj(:,idx.RPSI+(1))-DynTraj(:,idx.RASI+(1)),...
         DynTraj(:,idx.LANK+(2))-DynTraj(:,idx.LKNE+(2)),DynTraj(:,idx.RANK+(2))-DynTraj(:,idx.RKNE+(2)),...
         DynTraj(:,idx.LKNE+(2))-DynTraj(:,idx.LHIP+(2)),DynTraj(:,idx.RKNE+(2))-DynTraj(:,idx.RHIP+(2))];
if sum(Dir_Err>0)>0
    disp(['Label Error in DynTraj']);
end
%%
for framei=1:size(DynTraj,1)

%% Extract markers
traj_L_ASIS=DynTraj(framei,idx.LASI+(0:2)).';
traj_L_PSIS=DynTraj(framei,idx.LPSI+(0:2)).';
traj_R_PSIS=DynTraj(framei,idx.RPSI+(0:2)).';
traj_R_ASIS=DynTraj(framei,idx.RASI+(0:2)).';

traj_R_GT=DynTraj(framei,idx.RHIP+(0:2)).';
traj_L_GT=DynTraj(framei,idx.LHIP+(0:2)).';


traj_RKNE=DynTraj(framei,idx.RKNE+(0:2)).';
traj_RANK=DynTraj(framei,idx.RANK+(0:2)).';
traj_RHEE=DynTraj(framei,idx.RHEE+(0:2)).';
traj_RTOE=DynTraj(framei,idx.RTOE+(0:2)).';

traj_LKNE=DynTraj(framei,idx.LKNE+(0:2)).';
traj_LANK=DynTraj(framei,idx.LANK+(0:2)).';
traj_LHEE=DynTraj(framei,idx.LHEE+(0:2)).';
traj_LTOE=DynTraj(framei,idx.LTOE+(0:2)).';
%% Technical LCS of the Pelvis
Otec_PV =(traj_L_ASIS+traj_R_ASIS)/2;
itec_PV = traj_R_ASIS-traj_L_ASIS;
itec_PV = itec_PV/norm(itec_PV);

vtec_PV = Otec_PV-(traj_L_PSIS+traj_R_PSIS)/2;

ktec_PV = cross(itec_PV,vtec_PV);
ktec_PV = ktec_PV/norm(ktec_PV);

jtec_PV = cross(ktec_PV,itec_PV);

Ttec_PV = Tmaker(Otec_PV,itec_PV,jtec_PV,ktec_PV);

%%

traj_RHJC_inPV=[0.36;-0.19;-0.3]*norm(traj_R_ASIS-traj_L_ASIS);
traj_RHJC = Ttec_PV(1:3,1:3)*traj_RHJC_inPV + Otec_PV;

traj_LHJC_inPV=[-0.36;-0.19;-0.3]*norm(traj_R_ASIS-traj_L_ASIS);
traj_LHJC = Ttec_PV(1:3,1:3)*traj_LHJC_inPV + Otec_PV;

traj_OP=(traj_RHJC+traj_LHJC)/2;
traj_OPGT=(traj_R_GT+traj_L_GT)/2;

%%
Traj_OP(framei,1:3)=traj_OP;
Traj_R_GT(framei,1:3)=traj_R_GT;
Traj_L_GT(framei,1:3)=traj_L_GT;
Traj_OPGT(framei,1:3)=traj_OPGT;
Traj_RHJC(framei,1:3)=traj_RHJC;
Traj_RKJC(framei,1:3)=traj_RKNE;
Traj_RAJC(framei,1:3)=traj_RANK;
Traj_LHJC(framei,1:3)=traj_LHJC;
Traj_LKJC(framei,1:3)=traj_LKNE;
Traj_LAJC(framei,1:3)=traj_LANK;

Traj_RHEE(framei,1:3)=traj_RHEE;
Traj_RTOE(framei,1:3)=traj_RTOE;
Traj_LHEE(framei,1:3)=traj_LHEE;
Traj_LTOE(framei,1:3)=traj_LTOE;

AngP_Pelv(framei,1)=TL_Xangle(ktec_PV,'k');
AngP_RThi(framei,1)=TL_Xangle(traj_OPGT-traj_RKNE,'k');
AngP_RSha(framei,1)=TL_Xangle(traj_RKNE-traj_RANK,'k');
AngP_RFoo(framei,1)=TL_Xangle(traj_RTOE-traj_RHEE,'j');
AngP_LThi(framei,1)=TL_Xangle(traj_OPGT-traj_LKNE,'k');
AngP_LSha(framei,1)=TL_Xangle(traj_LKNE-traj_LANK,'k');
AngP_LFoo(framei,1)=TL_Xangle(traj_LTOE-traj_LHEE,'j');

Len_RThi(framei)=sqrt((traj_R_GT(2)-traj_RKNE(2)).^2+(traj_R_GT(3)-traj_RKNE(3)).^2);
Len_RSha(framei)=sqrt((traj_RKNE(2)-traj_RANK(2)).^2+(traj_RKNE(3)-traj_RANK(3)).^2);
Len_RFoo(framei)=sqrt((traj_RTOE(2)-traj_RHEE(2)).^2+(traj_RTOE(3)-traj_RHEE(3)).^2);
Len_LThi(framei)=sqrt((traj_L_GT(2)-traj_LKNE(2)).^2+(traj_L_GT(3)-traj_LKNE(3)).^2);
Len_LSha(framei)=sqrt((traj_LKNE(2)-traj_LANK(2)).^2+(traj_LKNE(3)-traj_LANK(3)).^2);
Len_LFoo(framei)=sqrt((traj_LTOE(2)-traj_LHEE(2)).^2+(traj_LTOE(3)-traj_LHEE(3)).^2);

r_segcom=[0.4095;0.4459;0.4415];
COMP_RThi(framei,1:3) = (traj_OPGT + r_segcom(1).*(traj_RKNE-traj_OPGT));
COMP_RSha(framei,1:3) = (traj_RKNE + r_segcom(1).*(traj_RANK-traj_RKNE));
COMP_RFoo(framei,1:3) = (traj_RHEE + r_segcom(1).*(traj_RTOE-traj_RHEE));

COMP_LThi(framei,1:3) = (traj_OPGT + r_segcom(1).*(traj_LKNE-traj_OPGT));
COMP_LSha(framei,1:3) = (traj_LKNE + r_segcom(1).*(traj_LANK-traj_LKNE));
COMP_LFoo(framei,1:3) = (traj_LHEE + r_segcom(1).*(traj_LTOE-traj_LHEE));

end


%%
Res_Dyn.AngP_RHip=AngP_RThi-AngP_Pelv;
Res_Dyn.AngP_RKne=AngP_RSha-AngP_RThi;
Res_Dyn.AngP_RAnk=AngP_RFoo-AngP_RSha;
Res_Dyn.AngP_LHip=AngP_LThi-AngP_Pelv;
Res_Dyn.AngP_LKne=AngP_LSha-AngP_LThi;
Res_Dyn.AngP_LAnk=AngP_LFoo-AngP_LSha;

%%
[Res_Dyn.TrajV_OP, Res_Dyn.TrajA_OP]=COM_diff(Traj_OP,aux);
[Res_Dyn.TrajV_OPGT, Res_Dyn.TrajA_OPGT]=COM_diff(Traj_OPGT,aux);
[Res_Dyn.TrajV_R_GT, Res_Dyn.TrajA_R_GT]=COM_diff(Traj_R_GT,aux);
[Res_Dyn.TrajV_L_GT, Res_Dyn.TrajA_L_GT]=COM_diff(Traj_L_GT,aux);

[Res_Dyn.TrajV_RHJC, Res_Dyn.TrajA_RHJC]=COM_diff(Traj_RHJC,aux);
[Res_Dyn.TrajV_RKJC, Res_Dyn.TrajA_RKJC]=COM_diff(Traj_RKJC,aux);
[Res_Dyn.TrajV_RAJC, Res_Dyn.TrajA_RAJC]=COM_diff(Traj_RAJC,aux);
[Res_Dyn.TrajV_LHJC, Res_Dyn.TrajA_LHJC]=COM_diff(Traj_LHJC,aux);
[Res_Dyn.TrajV_LKJC, Res_Dyn.TrajA_LKJC]=COM_diff(Traj_LKJC,aux);
[Res_Dyn.TrajV_LAJC, Res_Dyn.TrajA_LAJC]=COM_diff(Traj_LAJC,aux);

[Res_Dyn.COMV_RThi, Res_Dyn.COMA_RThi]=COM_diff(COMP_RThi,aux);
[Res_Dyn.COMV_RSha, Res_Dyn.COMA_RSha]=COM_diff(COMP_RSha,aux);
[Res_Dyn.COMV_RFoo, Res_Dyn.COMA_RFoo]=COM_diff(COMP_RFoo,aux);
[Res_Dyn.COMV_LThi, Res_Dyn.COMA_LThi]=COM_diff(COMP_LThi,aux);
[Res_Dyn.COMV_LSha, Res_Dyn.COMA_LSha]=COM_diff(COMP_LSha,aux);
[Res_Dyn.COMV_LFoo, Res_Dyn.COMA_LFoo]=COM_diff(COMP_LFoo,aux);

[Res_Dyn.AngV_RHip,Res_Dyn.AngA_RHip]=COM_diff(Res_Dyn.AngP_RHip,aux);
[Res_Dyn.AngV_RKne,Res_Dyn.AngA_RKne]=COM_diff(Res_Dyn.AngP_RKne,aux);
[Res_Dyn.AngV_RAnk,Res_Dyn.AngA_RAnk]=COM_diff(Res_Dyn.AngP_RAnk,aux);
[Res_Dyn.AngV_LHip,Res_Dyn.AngA_LHip]=COM_diff(Res_Dyn.AngP_LHip,aux);
[Res_Dyn.AngV_LKne,Res_Dyn.AngA_LKne]=COM_diff(Res_Dyn.AngP_LKne,aux);
[Res_Dyn.AngV_LAnk,Res_Dyn.AngA_LAnk]=COM_diff(Res_Dyn.AngP_LAnk,aux);

[Res_Dyn.AngV_Pelv,Res_Dyn.AngA_Pelv]=COM_diff(AngP_Pelv,aux);
[Res_Dyn.AngV_RThi,Res_Dyn.AngA_RThi]=COM_diff(AngP_RThi,aux);
[Res_Dyn.AngV_RSha,Res_Dyn.AngA_RSha]=COM_diff(AngP_RSha,aux);
[Res_Dyn.AngV_RFoo,Res_Dyn.AngA_RFoo]=COM_diff(AngP_RFoo,aux);
[Res_Dyn.AngV_LThi,Res_Dyn.AngA_LThi]=COM_diff(AngP_LThi,aux);
[Res_Dyn.AngV_LSha,Res_Dyn.AngA_LSha]=COM_diff(AngP_LSha,aux);
[Res_Dyn.AngV_LFoo,Res_Dyn.AngA_LFoo]=COM_diff(AngP_LFoo,aux);

%%

Res_Dyn.AngP_Pelv=AngP_Pelv;
Res_Dyn.AngP_RThi=AngP_RThi;
Res_Dyn.AngP_RSha=AngP_RSha;
Res_Dyn.AngP_RFoo=AngP_RFoo;
Res_Dyn.AngP_LThi=AngP_LThi;
Res_Dyn.AngP_LSha=AngP_LSha;
Res_Dyn.AngP_LFoo=AngP_LFoo;

Res_Dyn.Len_RThi=mean(Len_RThi);
Res_Dyn.Len_RSha=mean(Len_RSha);
Res_Dyn.Len_RFoo=mean(Len_RFoo);
Res_Dyn.Len_LThi=mean(Len_LThi);
Res_Dyn.Len_LSha=mean(Len_LSha);
Res_Dyn.Len_LFoo=mean(Len_LFoo);

Res_Dyn.Traj_OP=Traj_OP;
Res_Dyn.Traj_OPGT=Traj_OPGT;
Res_Dyn.Traj_R_GT=Traj_R_GT;
Res_Dyn.Traj_L_GT=Traj_L_GT;

Res_Dyn.Traj_RHJC=Traj_RHJC;
Res_Dyn.Traj_RKJC=Traj_RKJC;
Res_Dyn.Traj_RAJC=Traj_RAJC;
Res_Dyn.Traj_LHJC=Traj_LHJC;
Res_Dyn.Traj_LKJC=Traj_LKJC;
Res_Dyn.Traj_LAJC=Traj_LAJC;

Res_Dyn.Traj_RHEE=Traj_RHEE;
Res_Dyn.Traj_RTOE=Traj_RTOE;
Res_Dyn.Traj_LHEE=Traj_LHEE;
Res_Dyn.Traj_LTOE=Traj_LTOE;


Res_Dyn.COMP_RThi=COMP_RThi;
Res_Dyn.COMP_RSha=COMP_RSha;
Res_Dyn.COMP_RFoo=COMP_RFoo;
Res_Dyn.COMP_LThi=COMP_LThi;
Res_Dyn.COMP_LSha=COMP_LSha;
Res_Dyn.COMP_LFoo=COMP_LFoo;


Res_Dyn.Lstep_R=Lstep_R;
Res_Dyn.Lstep_L=Lstep_L;
Res_Dyn.Lstep_RdL=Lstep_R/Lstep_L; % ratio
















