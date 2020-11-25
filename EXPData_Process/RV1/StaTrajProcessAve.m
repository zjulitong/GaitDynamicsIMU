function Res_Sta=StaTrajProcessAve(StaTraj_FileName,aux)

%%
[StaTrajnum,StaTrajtxt,~]=xlsread(StaTraj_FileName);
StaTrajnum(:,3:3:end)=StaTrajnum(:,3:3:end)+2.6; % to compensate for the offset of calibration wand.
StaTrajnum(:,4:3:end)=StaTrajnum(:,4:3:end)+2.6;
StaTrajnum(:,5:3:end)=StaTrajnum(:,5:3:end)-27.7;

StaTraj2End=StaTrajnum(5:end,:)/1000;

for i=1:(length(StaTrajtxt(3,:)))
    mkr_name=StaTrajtxt{3,i};
    if ~isempty(mkr_name)
        chid=strfind(mkr_name,':');
        mkr_name=mkr_name((chid+1):end);
        eval(['aux.idx_stat.',mkr_name,'=',num2str(i),';']);
    end
end
if isfield(aux.idx_stat,'RHEEL')
    aux.idx_stat.RHEE=aux.idx_stat.RHEEL;
end
if isfield(aux.idx_stat,'LHEEL')
    aux.idx_stat.LHEE=aux.idx_stat.LHEEL;
end
idx=aux.idx_stat;
framei=1;
%%
f_start=1;
for fi=1:size(StaTraj2End,1)
    traj=StaTraj2End(fi,idx.LASI:(idx.LTOE+2));
    if sum(isnan(traj))==0
        f_start=fi;
        break;
    end
end
f_end=size(StaTraj2End,1);
for fi=(f_start+1):size(StaTraj2End,1)
    traj=StaTraj2End(fi,idx.LASI:(idx.LTOE+2));
    if sum(isnan(traj))>0
        f_end=fi-1;
        break;
    end
end
StaTraj=StaTraj2End(f_start:f_end,:); 
FrameCount=size(StaTraj,1);
%%
if nnz(isnan(StaTraj(:,[idx.LASI,idx.LPSI,idx.RPSI,idx.RASI,...
        idx.RHIP,idx.LHIP,idx.RKNE,idx.LKNE,idx.LANK,idx.RANK,...
        idx.LHEE,idx.RHEE,idx.LTOE,idx.RTOE])))>0
    disp('NaN exists in StaTraj');
end
% L-R should be negative, P-A should be negative
Dir_Err=[StaTraj(:,idx.LASI+(0))-StaTraj(:,idx.RASI+(0)),StaTraj(:,idx.LPSI+(0))-StaTraj(:,idx.RPSI+(0)),...
         StaTraj(:,idx.LHIP+(0))-StaTraj(:,idx.RHIP+(0)),StaTraj(:,idx.LKNE+(0))-StaTraj(:,idx.RKNE+(0)),...
         StaTraj(:,idx.LANK+(0))-StaTraj(:,idx.RANK+(0)),StaTraj(:,idx.LHEE+(0))-StaTraj(:,idx.RHEE+(0)),...
         StaTraj(:,idx.LTOE+(0))-StaTraj(:,idx.RTOE+(0)),...
         StaTraj(:,idx.LPSI+(1))-StaTraj(:,idx.LASI+(1)),StaTraj(:,idx.RPSI+(1))-StaTraj(:,idx.RASI+(1)),...
         StaTraj(:,idx.LANK+(2))-StaTraj(:,idx.LKNE+(2)),StaTraj(:,idx.RANK+(2))-StaTraj(:,idx.RKNE+(2)),...
         StaTraj(:,idx.LKNE+(2))-StaTraj(:,idx.LHIP+(2)),StaTraj(:,idx.RKNE+(2))-StaTraj(:,idx.RHIP+(2))];
if sum(Dir_Err>0)>0
    disp(['Label Error in DynTraj']);
end
%%
for framei=1:FrameCount
%% Extract markers
traj_L_ASIS=StaTraj(framei,idx.LASI+(0:2)).';
traj_L_PSIS=StaTraj(framei,idx.LPSI+(0:2)).';
traj_R_PSIS=StaTraj(framei,idx.RPSI+(0:2)).';
traj_R_ASIS=StaTraj(framei,idx.RASI+(0:2)).';

traj_R_GT=StaTraj(framei,idx.RHIP+(0:2)).';
traj_L_GT=StaTraj(framei,idx.LHIP+(0:2)).';

traj_RKNE=StaTraj(framei,idx.RKNE+(0:2)).';
traj_RANK=StaTraj(framei,idx.RANK+(0:2)).';
traj_RHEE=StaTraj(framei,idx.RHEE+(0:2)).';
traj_RTOE=StaTraj(framei,idx.RTOE+(0:2)).';

traj_LKNE=StaTraj(framei,idx.LKNE+(0:2)).';
traj_LANK=StaTraj(framei,idx.LANK+(0:2)).';
traj_LHEE=StaTraj(framei,idx.LHEE+(0:2)).';
traj_LTOE=StaTraj(framei,idx.LTOE+(0:2)).';

%% Segment LCS of the Pelvis
Oseg_PV =(traj_L_ASIS+traj_R_ASIS)/2;
iseg_PV = traj_R_ASIS-traj_L_ASIS;
iseg_PV = iseg_PV/norm(iseg_PV);

vseg_PV = Oseg_PV-(traj_L_PSIS+traj_R_PSIS)/2;

kseg_PV = cross(iseg_PV,vseg_PV);
kseg_PV = kseg_PV/norm(kseg_PV);

jseg_PV = cross(kseg_PV,iseg_PV);

Tseg_PV = Tmaker(Oseg_PV,iseg_PV,jseg_PV,kseg_PV);

traj_RHJC_inPV=[0.36;-0.19;-0.3]*norm(traj_R_ASIS-traj_L_ASIS);
traj_RHJC = Tseg_PV(1:3,1:3)*traj_RHJC_inPV + Oseg_PV;

traj_LHJC_inPV=[-0.36;-0.19;-0.3]*norm(traj_R_ASIS-traj_L_ASIS);
traj_LHJC = Tseg_PV(1:3,1:3)*traj_LHJC_inPV + Oseg_PV;

traj_OP=(traj_RHJC+traj_LHJC)/2;
traj_OPGT=(traj_R_GT+traj_L_GT)/2;

AngP_Pelv_Sta(framei,1)=TL_Xangle(kseg_PV,'k');

AngP_RThi_Sta(framei,1)=TL_Xangle(traj_OPGT-traj_RKNE,'k');
AngP_RSha_Sta(framei,1)=TL_Xangle(traj_RKNE-traj_RANK,'k');
AngP_RFoo_Sta(framei,1)=TL_Xangle(traj_RTOE-traj_RHEE,'j');

AngP_LThi_Sta(framei,1)=TL_Xangle(traj_OPGT-traj_LKNE,'k');
AngP_LSha_Sta(framei,1)=TL_Xangle(traj_LKNE-traj_LANK,'k');
AngP_LFoo_Sta(framei,1)=TL_Xangle(traj_LTOE-traj_LHEE,'j');

PosiX_RHEE(framei,1)=traj_RHEE(2)-traj_RANK(2);
PosiX_RTOE(framei,1)=traj_RTOE(2)-traj_RANK(2);
PosiY_RANK(framei,1)=traj_RANK(3);
PosiX_LHEE(framei,1)=traj_LHEE(2)-traj_LANK(2);
PosiX_LTOE(framei,1)=traj_LTOE(2)-traj_LANK(2);
PosiY_LANK(framei,1)=traj_LANK(3);

Len_RThi(framei)=sqrt((traj_R_GT(2)-traj_RKNE(2)).^2+(traj_R_GT(3)-traj_RKNE(3)).^2);
Len_RSha(framei)=sqrt((traj_RKNE(2)-traj_RANK(2)).^2+(traj_RKNE(3)-traj_RANK(3)).^2);
Len_RFoo(framei)=sqrt((traj_RTOE(2)-traj_RHEE(2)).^2+(traj_RTOE(3)-traj_RHEE(3)).^2);
Len_LThi(framei)=sqrt((traj_L_GT(2)-traj_LKNE(2)).^2+(traj_L_GT(3)-traj_LKNE(3)).^2);
Len_LSha(framei)=sqrt((traj_LKNE(2)-traj_LANK(2)).^2+(traj_LKNE(3)-traj_LANK(3)).^2);
Len_LFoo(framei)=sqrt((traj_LTOE(2)-traj_LHEE(2)).^2+(traj_LTOE(3)-traj_LHEE(3)).^2);
end
%%
Res_Sta.AngP_Pelv_Sta=mean(AngP_Pelv_Sta);
Res_Sta.AngP_RThi_Sta=mean(AngP_RThi_Sta);
Res_Sta.AngP_RSha_Sta=mean(AngP_RSha_Sta);
Res_Sta.AngP_RFoo_Sta=mean(AngP_RFoo_Sta);

Res_Sta.AngP_LThi_Sta=mean(AngP_LThi_Sta);
Res_Sta.AngP_LSha_Sta=mean(AngP_LSha_Sta);
Res_Sta.AngP_LFoo_Sta=mean(AngP_LFoo_Sta);

Res_Sta.AngP_RHip_Sta=Res_Sta.AngP_RThi_Sta-Res_Sta.AngP_Pelv_Sta;
Res_Sta.AngP_RKne_Sta=Res_Sta.AngP_RSha_Sta-Res_Sta.AngP_RThi_Sta;
Res_Sta.AngP_RAnk_Sta=Res_Sta.AngP_RFoo_Sta-Res_Sta.AngP_RSha_Sta;
Res_Sta.AngP_LHip_Sta=Res_Sta.AngP_LThi_Sta-Res_Sta.AngP_Pelv_Sta;
Res_Sta.AngP_LKne_Sta=Res_Sta.AngP_LSha_Sta-Res_Sta.AngP_LThi_Sta;
Res_Sta.AngP_LAnk_Sta=Res_Sta.AngP_LFoo_Sta-Res_Sta.AngP_LSha_Sta;

Res_Sta.PosiX_RHEE=mean(PosiX_RHEE);
Res_Sta.PosiX_RTOE=mean(PosiX_RTOE);
Res_Sta.PosiY_RANK=mean(PosiY_RANK);
Res_Sta.PosiX_LHEE=mean(PosiX_LHEE);
Res_Sta.PosiX_LTOE=mean(PosiX_LTOE);
Res_Sta.PosiY_LANK=mean(PosiY_LANK);

Res_Sta.Len_RThi=mean(Len_RThi);
Res_Sta.Len_RSha=mean(Len_RSha);
Res_Sta.Len_RFoo=mean(Len_RFoo);
Res_Sta.Len_LThi=mean(Len_LThi);
Res_Sta.Len_LSha=mean(Len_LSha);
Res_Sta.Len_LFoo=mean(Len_LFoo);
%%
if aux.plotSta==1
figure;
plot3(StaTraj(framei,3:3:end),StaTraj(framei,4:3:end),...
    StaTraj(framei,5:3:end),'.b','MarkerSize',15);
hold on;
plot3(traj_RHJC(1,:),traj_RHJC(2,:),traj_RHJC(3,:),'.k','MarkerSize',15)
plot3(traj_LHJC(1,:),traj_LHJC(2,:),traj_LHJC(3,:),'.k','MarkerSize',15)
plot3(traj_OP(1,:),traj_OP(2,:),traj_OP(3,:),'.k','MarkerSize',15)
plot3(traj_OPGT(1,:),traj_OPGT(2,:),traj_OPGT(3,:),'.k','MarkerSize',15)
plot_coord(Tseg_PV,0.2);
axis equal
end












