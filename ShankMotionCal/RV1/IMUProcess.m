% This is the main function to extract the steps from the raw measurements. 
clear;clc;

%%
SubjCount=12;
GaitTypes={'Norm','Slow','Fast','Rmin','Lmin'};

TrialCounts=zeros(length(GaitTypes),SubjCount);

CaliIdx=[1 1 1 1 1;   1 1 1 1 1;   1 1 1 1 1;   1 1 1 1 1;   1 1 1 1 1;   
         1 1 1 1 1;   1 1 1 1 1;   1 1 1 1 1;   1 1 1 1 1;   1 1 1 1 2;    
         1 1 1 1 2;   1 1 1 1 2;]; 
folderName='../RawData/';
saveflag= 1 ;
plotstand= 1 ;
ploteach= 0 ;
idx=0;
Tab_collect=[];

%% Read Data in Matlab
for isubj= 1:SubjCount % subject
    subName=['S',num2str(isubj,'%02d')];
    for itype= [1:5]
        GaitType=GaitTypes{itype};
        TrialLeft_All = csvread([folderName,'/',subName,'/',GaitType,'L','.csv'],0,0);%L
        SegmentCount=getSegmentCount(TrialLeft_All);
        TrialCounts(itype,isubj)=SegmentCount;
        
        rgtrial=[ (CaliIdx(isubj,itype)+1):SegmentCount];
        if isubj==4&&itype==5
            rgtrial=[ (CaliIdx(isubj,itype)+1):13];
        elseif isubj==12&&itype==3
            rgtrial=[ 3:7,9:13];
        end   
for itrial=  rgtrial

breakTimeAll=[];
clearvars('-except','folderName', 'saveflag', 'plotstand', 'ploteach', 'CaliIdx',...
    'isubj', 'subName', 'SegmentCount', 'TrialCounts', 'rgtrial', 'itrial',...
    'breakTimeAll',  'GaitType', 'GaitTypes', 'itype','Tab_collect','idx');

IMU2AnkleLength_Left=5/100;% IMU to ankle joint is 5 cm.
IMU2AnkleLength_Right=5/100;

findbreak=1;
TrialLeft_All = csvread([folderName,'/',subName,'/',GaitType,'L','.csv'],0,0);%L
TrialRight_All = csvread([folderName,'/',subName,'/',GaitType,'R','.csv'],0,0);%R

findbreak=0;
TrialLeft=getDataSegment(TrialLeft_All,itrial,1);
TrialRight=getDataSegment(TrialRight_All,itrial,1);
TrialLeft=TrialLeft(1:min(length(TrialLeft(:,1)),length(TrialRight(:,1))),:);
TrialRight=TrialRight(1:min(length(TrialLeft(:,1)),length(TrialRight(:,1))),:);
Trial=[TrialRight(:,1) TrialRight(:,3) -TrialRight(:,2) TrialRight(:,4) TrialRight(:,6) -TrialRight(:,5) TrialRight(:,7),...
                       TrialLeft(:,3) -TrialLeft(:,2) TrialLeft(:,4) TrialLeft(:,6) -TrialLeft(:,5) TrialLeft(:,7)];
%Find break
findbreak=1;
time=Trial(:,1);
breakTime=0;
breakPeriod=0;
if findbreak==1
for ipre=2:length(time) 
    if round((time(ipre)-time(ipre-1))/10)~=1
        if round((time(ipre+1)-time(ipre))/10)==1
            breakTime=breakTime+1;
            if time(ipre-1)<time(ipre)&&time(ipre-1)>time(ipre-2)
              breakPeriod(breakTime)=round((time(ipre)-time(ipre-2))/10)-1;
              for iipre=length(Trial(:,1)):-1:ipre+length(Trial(:,1))-length(time)
                Trial(iipre+breakPeriod(breakTime)-1,:)=Trial(iipre,:);
              end
              breakDistance=(Trial(ipre+length(Trial(:,1))-length(time),:)-Trial(ipre-1-breakPeriod(breakTime)+length(Trial(:,1))-length(time),:))/(breakPeriod(breakTime)+1);
              for iipre=ipre-1+length(Trial(:,1))-length(time):-1:ipre+length(Trial(:,1))-length(time)-breakPeriod(breakTime)
                  Trial(iipre,:)=Trial(iipre+1,:)-breakDistance;
              end
            else
              breakPeriod(breakTime)=round((time(ipre)-time(ipre-3))/10)-1;
              for iipre=length(Trial(:,1)):-1:ipre+length(Trial(:,1))-length(time)
                Trial(iipre+breakPeriod(breakTime)-2,:)=Trial(iipre,:);
              end
              breakDistance=(Trial(ipre+length(Trial(:,1))-length(time),:)-Trial(ipre-1-breakPeriod(breakTime)+length(Trial(:,1))-length(time),:))/(breakPeriod(breakTime)+1);
              for iipre=ipre-1+length(Trial(:,1))-length(time):-1:ipre+length(Trial(:,1))-length(time)-breakPeriod(breakTime)
                  Trial(iipre,:)=Trial(iipre+1,:)-breakDistance;
              end
            end
        end
    end
end
end
if breakTime~=0
    disp(['breakTime=',num2str(breakTime),'; breakPeriod=',num2str(breakPeriod)]);
end
%% Gait
%------
TrialLeft=Trial(:,8:13);
TrialStand=getDataSegment(TrialLeft_All,CaliIdx(isubj,itype),1);
TrialStandLeft=[TrialStand(:,3) -TrialStand(:,2) TrialStand(:,4) TrialStand(:,6) -TrialStand(:,5) TrialStand(:,7)];
TrialLeft = AxisCalibrationBreak( TrialLeft,TrialStandLeft );

[outdata_Left,~,~,~,~,~,~,...
    ~,~,~,~,~,~,~,~,~,~,~,...
    ~,~,~,~,~,~,gaitCycle_Left,HorizontalDisplacementCycle_Left,...
    LateralDisplacementCycle_Left,VerticalDisplacementCycle_Left,RollCycle_Left,PitchCycle_Left,YawCycle_Left] = GetGaitStm32_191022(TrialLeft,IMU2AnkleLength_Left);
TrialRight=Trial(:,2:7);
TrialRight(:,2)=-TrialRight(:,2);
TrialRight(:,3)=-TrialRight(:,3);
TrialRight(:,5)=-TrialRight(:,5);
TrialRight(:,6)=-TrialRight(:,6);
%------
TrialStand=getDataSegment(TrialRight_All,CaliIdx(isubj,itype),1);
TrialStandRight=[ TrialStand(:,3) -TrialStand(:,2) TrialStand(:,4) TrialStand(:,6) -TrialStand(:,5) TrialStand(:,7)];
TrialStandRight(:,2)=-TrialStandRight(:,2);
TrialStandRight(:,3)=-TrialStandRight(:,3);
TrialStandRight(:,5)=-TrialStandRight(:,5);
TrialStandRight(:,6)=-TrialStandRight(:,6);
TrialRight = AxisCalibrationBreak( TrialRight,TrialStandRight );

[outdata_Right,~,~,~,~,~,...
    ~,~,~,~,...
    ~,~,~,~,~,~,~,~,~,~,~,~,~,...
    ~,gaitCycle_Right,HorizontalDisplacementCycle_Right,LateralDisplacementCycle_Right,VerticalDisplacementCycle_Right,RollCycle_Right,...
    PitchCycle_Right,YawCycle_Right] = GetGaitStm32_191022(TrialRight,IMU2AnkleLength_Right);

%%
if itrial==2 && plotstand==1
    figure;
    subplot(2,1,1);hold on;title([subName,GaitType, 'L']);
    plot(TrialStandLeft);
    subplot(2,1,2);hold on;title([subName,GaitType, 'R']);
    plot(TrialStandRight);
end
if ploteach==1
    figure;
    subplot(2,1,1);hold on;title(['iinCycle=',num2str(itrial)]);
    plot(TrialLeft);
    subplot(2,1,2);hold on;title(['iinCycle=',num2str(itrial)]);
    plot(TrialRight);
end
%%
idx=idx+1;
Tab_collect{idx,1}=isubj;
Tab_collect{idx,2}=itype;
Tab_collect{idx,3}=itrial;
Tab_collect{idx,4}=subName;
Tab_collect{idx,5}=GaitType;
Tab_collect{idx,6}=size(TrialLeft,1);

%% 
gaitCycle_Left=[gaitCycle_Left outdata_Left(:,9)];
gaitCycle_Right=[gaitCycle_Right outdata_Right(:,9)];
disp(['SubNum=',num2str(isubj),'; TypeNum=',num2str(itype),'; TriNum=',num2str(itrial)])
if saveflag==1
    FolderOut=['./ExtractData/',subName];
    if ~exist(FolderOut,'dir')
        mkdir(FolderOut);
    end
    save([FolderOut,'/',GaitType,'_',num2str(itrial,'%02d'),'.mat'],'gaitCycle_Left',...
    'HorizontalDisplacementCycle_Left','LateralDisplacementCycle_Left','VerticalDisplacementCycle_Left','RollCycle_Left','PitchCycle_Left','YawCycle_Left',...
    'TrialLeft','gaitCycle_Right','HorizontalDisplacementCycle_Right','LateralDisplacementCycle_Right','VerticalDisplacementCycle_Right',...
    'RollCycle_Right','PitchCycle_Right','YawCycle_Right','TrialRight');
end
end
    end
end

% return;
%%
TimeLen_table=cell2table(Tab_collect,'VariableNames',...
    {'isubj','itype','itrial','subName','GaitType','TimeLen'});
if saveflag==1
    save(['TimeLen_',char(datetime('now','format','yyyyMMdd_HHmmss'))],'TimeLen_table');
end
set(0,'DefaultFigureVisible', 'on')






