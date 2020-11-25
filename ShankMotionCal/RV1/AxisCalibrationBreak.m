function [ TrialDataOut ] = AxisCalibrationBreak( TrialData,TrialStand )
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
AcceRatio = 4096;
GyroRatio = 32.768;
TrialData(:,1:3)=TrialData(:,1:3)/AcceRatio;
TrialData(:,4:6)=TrialData(:,4:6)/GyroRatio;
TrialData(:,6)=-TrialData(:,6);
TrialData(:,3)=-TrialData(:,3);
TrialStand(:,1:3)=TrialStand(:,1:3)/AcceRatio;
TrialStand(:,4:6)=TrialStand(:,4:6)/GyroRatio;
TrialStand(:,6)=-TrialStand(:,6);
TrialStand(:,3)=-TrialStand(:,3);
f_low=10;
fs=100;
[B,A] = butter(2,2*f_low/fs,'low');
Omega_Z_Filter=filtfilt(B,A,TrialData(:,6));
ThresholdSwitch=0;
HeelStrikeCount=0;
ThresholdSwitchTime=0;
HeelStrike=0;
ToeOff=0;

sumaxb=0;
sumayb=0;
sumazb=0;
for i=1:length(TrialStand(:,1))
   ax=TrialStand(i,1);
   ay=TrialStand(i,2);
   az=TrialStand(i,3);
   sumaxb=sumaxb+ax;
   sumayb=sumayb+ay;
   sumazb=sumazb+az;
end
avaxb=sumaxb/length(TrialStand(:,1));
avayb=sumayb/length(TrialStand(:,1));
avazb=sumazb/length(TrialStand(:,1));

for i=2:length(Omega_Z_Filter)
    if ThresholdSwitch==0&&Omega_Z_Filter(i-1)>=50&&Omega_Z_Filter(i)<50
        ThresholdSwitch=1;
        HeelStrikeCount=HeelStrikeCount+1;
        ThresholdSwitchTime(HeelStrikeCount)=i;
    end
    if ThresholdSwitch==1
        if Omega_Z_Filter(i-1)<=0&&Omega_Z_Filter(i)>Omega_Z_Filter(i-1)
            HeelStrike(HeelStrikeCount)=i-1;
            for ii=ThresholdSwitchTime(HeelStrikeCount)-1:-1:2
                if Omega_Z_Filter(ii)<=0&&Omega_Z_Filter(ii)<Omega_Z_Filter(ii-1)
                    ToeOff(HeelStrikeCount)=ii;
                    break;
                end
            end
            ThresholdSwitch=0;
        end
    end
end

sumgxb=0;
sumgyb=0;
sumgzb=0;
numgzb=0;
AngleX_newX=0;
AngleZ_newZ=0;

for i=1:length(HeelStrike)-1
    if ToeOff(i)~=0&&ToeOff(i+1)~=0
        for ii=HeelStrike(i):ToeOff(i+1)
            if TrialData(ii,6)<0
                sumgxb=sumgxb+TrialData(ii,4);
                sumgyb=sumgyb+TrialData(ii,5);
                sumgzb=sumgzb+TrialData(ii,6);
                numgzb=numgzb+1;
            end
        end
    end
end
if numgzb~=0
    avgxb=sumgxb/numgzb;
    avgyb=sumgyb/numgzb;
    avgzb=sumgzb/numgzb;
    AngleX_newX=atan(avgxb/avgzb);
    AngleZ_newZ=atan(-avgyb/sqrt(avgzb^2+avgxb^2));
end
AngleX_newXc=cos(AngleX_newX);
AngleX_newXs=sin(AngleX_newX);
AngleZ_newZc=cos(AngleZ_newZ);
AngleZ_newZs=sin(AngleZ_newZ);
InitialAngle=atan((avayb*AngleZ_newZc+(AngleX_newXs*avaxb+AngleX_newXc*avazb)*(-AngleZ_newZs))/(avazb*AngleX_newXs-avaxb*AngleX_newXc));

for i=1:length(TrialData(:,1))
     ay=TrialData(i,2);
     az=TrialData(i,3);
     ax=TrialData(i,1);
     gy=TrialData(i,5);
     gz=TrialData(i,6);
     gx=TrialData(i,4);
     gyroZ_Left(i)=(gz*AngleX_newXc+gx*AngleX_newXs)*AngleZ_newZc+gy*AngleZ_newZs;   
     gyroXtem_Left=-gz*AngleX_newXs+gx*AngleX_newXc;
     gyroYtem_Left=-(gz*AngleX_newXc+gx*AngleX_newXs)*AngleZ_newZs+gy*AngleZ_newZc;
     gyroY_Left(i)=gyroYtem_Left*cos(-InitialAngle)-gyroXtem_Left*sin(-InitialAngle);
     gyroX_Left(i)=gyroXtem_Left*cos(-InitialAngle)+gyroYtem_Left*sin(-InitialAngle);
     accZ_Left(i)=(az*AngleX_newXc+ax*AngleX_newXs)*AngleZ_newZc+ay*AngleZ_newZs;
     accXtem_Left=-az*AngleX_newXs+ax*AngleX_newXc;
     accYtem_Left=ay*AngleZ_newZc-(az*AngleX_newXc+ax*AngleX_newXs)*AngleZ_newZs; 
     accY_Left(i)=accYtem_Left*cos(-InitialAngle)-accXtem_Left*sin(-InitialAngle);
     accX_Left(i)=accXtem_Left*cos(-InitialAngle)+accYtem_Left*sin(-InitialAngle);
end
TrialData(:,1)=accX_Left*AcceRatio;
TrialData(:,2)=accY_Left*AcceRatio;
TrialData(:,3)=-accZ_Left*AcceRatio;
TrialData(:,4)=gyroX_Left*GyroRatio;
TrialData(:,5)=gyroY_Left*GyroRatio;
TrialData(:,6)=-gyroZ_Left*GyroRatio;
TrialDataOut=TrialData;
end

