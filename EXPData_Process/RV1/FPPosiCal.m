function aux=FPPosiCal(aux)



%%
[FPLnum,FPLtxt,~]=xlsread(aux.FPLeft_FileName);
FPLnum(:,3:3:end)=FPLnum(:,3:3:end)+2.6; % to compensate for the offset of calibration wand.
FPLnum(:,4:3:end)=FPLnum(:,4:3:end)+2.6;
FPLnum(:,5:3:end)=FPLnum(:,5:3:end)-27.7;

FPLTraj=FPLnum(5:end,:)/1000;
aux.TrajFrameRate=FPLnum(1,1);

for i=1:(length(FPLtxt(3,:)))
    mkr_name=FPLtxt{3,i};
    if ~isempty(mkr_name)
%         modelname='IMUModel';
        chid=strfind(mkr_name,':');
%         mkr_name=mkr_name((length(modelname)+2):end);
        mkr_name=mkr_name((chid+1):end);
        eval(['aux.idx_FPCal.',mkr_name,'=',num2str(i),';']);
    end
end


TFORI=FPLTraj(:,aux.idx_FPCal.TFORI+(0:2)); % Origin
TFXP=FPLTraj(:,aux.idx_FPCal.TFXP+(0:2));% X-Positive
TFYP=FPLTraj(:,aux.idx_FPCal.TFYP+(0:2)); % Y-Positive


O_LWD=mean(TFORI).';
i_LWD=(mean(TFXP)-mean(TFORI)).';
i_LWD=i_LWD/norm(i_LWD);
v_LWD=(mean(TFYP)-mean(TFORI)).';
k_LWD=cross(i_LWD,v_LWD);
k_LWD=k_LWD/norm(k_LWD);
j_LWD=cross(k_LWD,i_LWD);

Tmat_LWD=Tmaker(O_LWD,i_LWD,j_LWD,k_LWD); % wand position of left forceplate


%%
[FPRnum,~,~]=xlsread(aux.FPRight_FileName);
FPRnum(:,3:3:end)=FPRnum(:,3:3:end)+2.6; % to compensate for the offset of calibration wand.
FPRnum(:,4:3:end)=FPRnum(:,4:3:end)+2.6;
FPRnum(:,5:3:end)=FPRnum(:,5:3:end)-27.7;

FPRTraj=FPRnum(5:end,:)/1000;

TFORI=FPRTraj(:,aux.idx_FPCal.TFORI+(0:2));
TFXP=FPRTraj(:,aux.idx_FPCal.TFXP+(0:2));
TFYP=FPRTraj(:,aux.idx_FPCal.TFYP+(0:2));

O_RWD=mean(TFORI).';

i_RWD=(mean(TFXP)-mean(TFORI)).';
i_RWD=i_RWD/norm(i_RWD);
v_RWD=(mean(TFYP)-mean(TFORI)).';
k_RWD=cross(i_RWD,v_RWD);
k_RWD=k_RWD/norm(k_RWD);
j_RWD=cross(k_RWD,i_RWD);

Tmat_RWD=Tmaker(O_RWD,i_RWD,j_RWD,k_RWD); % wand position of right forceplate

%% 
aux.Tmat_RFP_true=Tmat_RWD*(Tmat_LWD^-1)*aux.Tmat_LFP;

%%
if aux.plotFP==1
figure;hold on;
axis equal
rectangle('Position',[0 0 0.4 0.6]);
rectangle('Position',[aux.Tmat_RFP(1,4)-0.2 aux.Tmat_RFP(2,4)-0.3 0.4 0.6]);
plot_coord(aux.Tmat_0,0.1);
plot_coord(aux.Tmat_LFP,0.1);
plot_coord(aux.Tmat_RFP,0.1);

plot_coord(Tmat_RWD,0.2);
plot_coord(Tmat_LWD,0.2);
plot_coord(aux.Tmat_RFP_true,0.1,'-.');
grid minor
xlabel('x');
ylabel('y');zlabel('z');

end



















