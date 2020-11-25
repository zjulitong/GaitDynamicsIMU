function [outdata,RollView,VelocityFrontView,VelocityVerticalView,FootHeightTimeView,RollErrorDistanceView,AccVerticalErrorDistanceView,...
    RollErrorFilterView,ZeroVelocityDetectedView,AccVerticalErrorFilterView,RollErrorView,AccFrontView,AccVerticalView,AccVerticalErrorView,...
    ZeroVerticalView,AccVerticalErrorSqMeanView,VelocityVerticalCompensation3View,HorizontalDisplacementView,LateralDisplacementView,...
    VerticalDisplacementView,PitchTrueFilterView,PitchView,YawView,outParameter,...
    gaitCycle,HorizontalDisplacementCycle,LateralDisplacementCycle,VerticalDisplacementCycle,RollCycle,PitchCycle,YawCycle]...
    = GetGaitStm32_191022(TrialData,IMU2AnkleLength)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
%RollZD
%Compensation
%Output
%% Initialization
% IMU2AnkleLength=0.05;
AcceRatio = 4096;
GyroRatio = 32.768*180/pi;
CalculationDelay=20;
ax=0;
ay=0;
az=0;
gx=0;
gy=0;
gz=0;
SampleSequence=1+CalculationDelay;
AccX=zeros(SampleSequence,1);
AccY=zeros(SampleSequence,1);
AccZ=zeros(SampleSequence,1);
GyroX=zeros(SampleSequence,1);
GyroY=zeros(SampleSequence,1);
GyroZ=zeros(SampleSequence,1);
RollZD=zeros(SampleSequence,1);
PitchZD=zeros(SampleSequence,1);
YawZD=zeros(SampleSequence,1);
RollAHRS=zeros(SampleSequence,1);
PitchAHRS=zeros(SampleSequence,1);
YawAHRS=zeros(SampleSequence,1);
Roll=0;
Pitch=0;
Yaw=0;
QuaterX=0;
QuaterY=0;
QuaterZ=0;
q0=1;
q1=0;
q2=0;
q3=0;
lq0=1;
lq1=0;
lq2=0;
lq3=0;
SampleLoop=0;
dt=0.01;
GyroFilterParameter=[0.0675 0.1349 0.0675 1.143 -0.4128];
GyroFilterDelay=2;
GyroSequence=2+CalculationDelay-GyroFilterDelay;
GyroFilter=zeros(GyroSequence,1);
RollError1=0;
RollError2=0;
RollError=0;
RollTrue1=0;
RollTrue2=0;
RollTrue=0;
PitchTrue1=0;
PitchTrue2=0;
PitchTrue=0;
AccVerticalError2=0;
AccVerticalError1=0;
AccVerticalError=0;
AngleFilterParameter=[0.0078 0.0156 0.0078 1.7347 -0.766];
% AngleFilterParameter=[0.0055 0.0111 0.0055 1.7786 -0.8008];
AngleErrorFilterDelay=8;
ErrorDistanceNum=6;
AngleErrorSequence=1+ErrorDistanceNum+CalculationDelay-AngleErrorFilterDelay;
RollErrorFilter=zeros(AngleErrorSequence,1);
RollTrueFilter=zeros(AngleErrorSequence,1);
PitchTrueFilter=zeros(AngleErrorSequence,1);
AccVerticalErrorFilter=zeros(AngleErrorSequence,1);
GyroDelayIndex=SampleSequence-CalculationDelay;
GyroFilterDelayIndex=GyroSequence-CalculationDelay+GyroFilterDelay;
ErrorDelayIndex=AngleErrorSequence-CalculationDelay+AngleErrorFilterDelay;
GravityAcc=9.8;
SwingThreshold=50/180*pi;
ThresholdSwitch=0;
HeelstrikeTime=0;
ToeOffTime=0;
LastMaxGyroZ=0;
MaxGyroZ=0;
FootIn=0;
ZeroVelocityDetected=0;
ZeroVertical=0;
RollErrorDistance=0;
RollErrorDistanceMax=0.01;
RollErrorDistanceMaxDown=0.01;
RollErrorDistanceMaxUp=0.06;
% RollErrorDistanceMaxRatio=0.02;
RollErrorDistanceMaxRatio=0;
% RollErrorDistanceMax=0;
AccVerticalRollErrorRatio=3;
AccVerticalErrorDistance=0;
AccVerticalErrorDistanceMax=RollErrorDistanceMax*AccVerticalRollErrorRatio;
AccVerticalErrorDistanceMaxDown=RollErrorDistanceMaxDown*AccVerticalRollErrorRatio;
GyroZDown=3.0;
GyroZMax=0;
GaitStart=0;
GaitCycleMax=500;
AccFront1=0;
AccVertical1=0;
AccLateral1=0;
AccFront=0;
AccVertical=0;
AccLateral=0;
VelocityFront=0;
VelocityVertical=0;
VelocityLateral=0;
DistanceFront=0;
DistanceVertical=0;
DistanceLateral=0;

RollRemove=0;
PitchRemove=0;
RollErrorDistanceMin=0;
q0BU=1;
q1BU=0;
q2BU=0;
q3BU=0;
lq0BU=1;
lq1BU=0;
lq2BU=0;
lq3BU=0;
q0ZD=1;
q1ZD=0;
q2ZD=0;
q3ZD=0;
lq0ZD=1;
lq1ZD=0;
lq2ZD=0;
lq3ZD=0;
VelocityFrontBU=0;
VelocityVerticalBU=0;
VelocityLateralBU=0;
DistanceFrontBU=0;
DistanceVerticalBU=0;
DistanceLateralBU=0;
RollBU=0;
PitchBU=0;
YawBU=0;
q0IF=1;
q1IF=0;
q2IF=0;
q3IF=0;
lq0IF=1;
lq1IF=0;
lq2IF=0;
lq3IF=0;
VelocityFrontIF=0;
VelocityVerticalIF=0;
VelocityLateralIF=0;
DistanceFrontIF=0;
DistanceVerticalIF=0;
DistanceLateralIF=0;
RollIF=0;
PitchIF=0;
YawIF=0;
OutFootFlat=1;
IFtoBU=0;
YawRange=0;
ToeOffTem=0;
StrideLength=0;
LastHeelstrikeTime=0;
StepCount=0;
ShankAngle=0;
ShankAngleMax=0;
ShankAngleMin=0;
AnkleHeight=0;
AnkleHeightMax=0;
FootHeight=0;
FootHeightTem=0;
IntegrationResetTime=0;
AnkleHeightMaxTime=0;
FootHeightTime=0;
GaitStartTime=0;
ShankStanceAngle=0;
ShankStanceStraightenAngle=0;
StepOutPutCountView=0;
ShankStanceStraightenAngleTem=0;
ShankStanceAngleTem=0;
YawAHRSStart=0;
RollErrorFilterThreshold=1.5*RollErrorDistanceMax;
AccVerticalErrorFilterThreshold=1.5*AccVerticalErrorDistanceMax;
lastIntegrationResetTime=1;
lastMidStanceStartTime=1;
lastZeroStateEndRoll=0;
lastZeroStateEndPitch=0;
lastMidStanceVerticalStartTime=1;
lastIntegrationVerticalResetTime=1;
FootInAfterHSMin=15;
Regress=[0.1765,1.5008,-0.1870,0.8025,0.0489];
AHRS = MahonyAHRS('SamplePeriod', 0.01, 'Kp', 0.5);
%% Loop
for i=1:length(TrialData(:,1))
    %%
    ax=TrialData(i,1);
    ay=TrialData(i,2);
    az=-TrialData(i,3);
    gx=TrialData(i,4);
    gy=TrialData(i,5);
    gz=-TrialData(i,6);
    for SampleLoop=1:SampleSequence-1
        AccX(SampleLoop)=AccX(SampleLoop+1);
        AccY(SampleLoop)=AccY(SampleLoop+1);
        AccZ(SampleLoop)=AccZ(SampleLoop+1);
        GyroX(SampleLoop)=GyroX(SampleLoop+1);
        GyroY(SampleLoop)=GyroY(SampleLoop+1);
        GyroZ(SampleLoop)=GyroZ(SampleLoop+1);
        RollZD(SampleLoop)=RollZD(SampleLoop+1);
        PitchZD(SampleLoop)=PitchZD(SampleLoop+1);
        YawZD(SampleLoop)=YawZD(SampleLoop+1);
        RollAHRS(SampleLoop)=RollAHRS(SampleLoop+1);
		PitchAHRS(SampleLoop)=PitchAHRS(SampleLoop+1);
        YawAHRS(SampleLoop)=YawAHRS(SampleLoop+1);
    end
    AccX(SampleSequence)=ax/AcceRatio;
    AccY(SampleSequence)=ay/AcceRatio;
    AccZ(SampleSequence)=az/AcceRatio;
    GyroX(SampleSequence)=gx/GyroRatio;
    GyroY(SampleSequence)=gy/GyroRatio;
    GyroZ(SampleSequence)=gz/GyroRatio;
    GyroZView(i)=GyroZ(SampleSequence);
    GyroYView(i)=GyroY(SampleSequence);
    GyroXView(i)=GyroX(SampleSequence);
    Gyroscope=[GyroZ(SampleSequence) -GyroY(SampleSequence) -GyroX(SampleSequence)];
    Accelerometer=[AccZ(SampleSequence) -AccY(SampleSequence) -AccX(SampleSequence)];
    AHRS.UpdateIMU(Gyroscope,Accelerometer);	% gyroscope units must be radians
    quaternion = AHRS.Quaternion;
    EulerKF_Left = quatern2euler(quaternConj(quaternion)) ;
    RollAHRS(SampleSequence)=EulerKF_Left(1);
	PitchAHRS(SampleSequence)=EulerKF_Left(2);
    YawAHRS(SampleSequence)=EulerKF_Left(3);
    %update roll angle
    QuaterXZD=GyroZ(SampleSequence)/2*dt;
    QuaterYZD=-GyroY(SampleSequence)/2*dt;
    QuaterZZD=-GyroX(SampleSequence)/2*dt;
    lq0ZD=q0ZD;
    lq1ZD=q1ZD;
    lq2ZD=q2ZD;
    lq3ZD=q3ZD;
    q0ZD=lq0ZD-QuaterXZD*lq1ZD-QuaterYZD*lq2ZD-QuaterZZD*lq3ZD;
    q1ZD=lq1ZD+QuaterXZD*lq0ZD+QuaterZZD*lq2ZD-QuaterYZD*lq3ZD;
    q2ZD=lq2ZD+QuaterYZD*lq0ZD-QuaterZZD*lq1ZD+QuaterXZD*lq3ZD;
    q3ZD=lq3ZD+QuaterZZD*lq0ZD+QuaterYZD*lq1ZD-QuaterXZD*lq2ZD;
    qLength=sqrt(q0ZD^2+q1ZD^2+q2ZD^2+q3ZD^2);
    q0ZD=q0ZD/qLength;
    q1ZD=q1ZD/qLength;
    q2ZD=q2ZD/qLength;
    q3ZD=q3ZD/qLength;
    PitchZD(SampleSequence)=-asin(2*(q1ZD*q3ZD-q0ZD*q2ZD));
    YawZD(SampleSequence)=atan2(2*(q0ZD*q3ZD+q1ZD*q2ZD),(q0ZD^2+q1ZD^2-q2ZD^2-q3ZD^2));
    RollZD(SampleSequence)=atan2(2*(q0ZD*q1ZD+q2ZD*q3ZD),(q3ZD^2-q2ZD^2-q1ZD^2+q0ZD^2));
    RollZDView(i)=RollZD(GyroDelayIndex);
    %GyroFilter
    for SampleLoop=1:GyroSequence-1
        GyroFilter(SampleLoop)=GyroFilter(SampleLoop+1);
    end
    GyroFilter(GyroSequence)=GyroFilterParameter(4)*GyroFilter(GyroSequence-1)+GyroFilterParameter(5)*GyroFilter(GyroSequence-2)+...
        GyroFilterParameter(1)*GyroZ(SampleSequence)+GyroFilterParameter(2)*GyroZ(SampleSequence-1)+GyroFilterParameter(3)*GyroZ(SampleSequence-2);
    GyroFilterView(i)=GyroFilter(GyroSequence);
    %Zero velocity detection & angle error
    for SampleLoop=1:AngleErrorSequence-1
        RollErrorFilter(SampleLoop)=RollErrorFilter(SampleLoop+1);
        RollTrueFilter(SampleLoop)=RollTrueFilter(SampleLoop+1);
        PitchTrueFilter(SampleLoop)=PitchTrueFilter(SampleLoop+1);
        AccVerticalErrorFilter(SampleLoop)=AccVerticalErrorFilter(SampleLoop+1);
    end
    AccGyroZ=(GyroZ(SampleSequence)-GyroZ(SampleSequence-1))/dt;
    RollError2=RollError1;
    RollError1=RollError;
    RollError=atan((-IMU2AnkleLength*AccGyroZ+AccY(SampleSequence)*GravityAcc)/(AccX(SampleSequence)*GravityAcc-IMU2AnkleLength*(GyroZ(SampleSequence))^2))-RollZD(SampleSequence);
    RollErrorView(i)=RollError;
    RollErrorFilter(AngleErrorSequence)=AngleFilterParameter(4)*RollErrorFilter(AngleErrorSequence-1)+AngleFilterParameter(5)*RollErrorFilter(AngleErrorSequence-2)+AngleFilterParameter(1)*RollError+AngleFilterParameter(2)*RollError1+AngleFilterParameter(3)*RollError2;
    RollErrorFilterView(i)=RollErrorFilter(ErrorDelayIndex);
    
    RollTrue2=RollTrue1;
    RollTrue1=RollTrue;
    RollTrue=RollError+RollZD(SampleSequence);
    RollTrueFilter(AngleErrorSequence)=AngleFilterParameter(4)*RollTrueFilter(AngleErrorSequence-1)+AngleFilterParameter(5)*RollTrueFilter(AngleErrorSequence-2)+AngleFilterParameter(1)*RollTrue+AngleFilterParameter(2)*RollTrue1+AngleFilterParameter(3)*RollTrue2;
    RollTrueFilterView(i)=RollTrueFilter(AngleErrorSequence);
    
    PitchTrue2=PitchTrue1;
    PitchTrue1=PitchTrue;
    AccGyroY=(GyroY(SampleSequence)-GyroY(SampleSequence-1))/dt;
    PitchTrue=-atan((-IMU2AnkleLength*AccGyroY-AccZ(SampleSequence)*GravityAcc)/((AccX(SampleSequence)*cos(RollTrue)+AccY(SampleSequence)*sin(RollTrue))*GravityAcc-IMU2AnkleLength*(GyroY(SampleSequence))^2));
    PitchTrueView(i)=PitchTrue;
    PitchTrueFilter(AngleErrorSequence)=AngleFilterParameter(4)*PitchTrueFilter(AngleErrorSequence-1)+AngleFilterParameter(5)*PitchTrueFilter(AngleErrorSequence-2)+AngleFilterParameter(1)*PitchTrue+AngleFilterParameter(2)*PitchTrue1+AngleFilterParameter(3)*PitchTrue2;
    PitchTrueFilterView(i)=PitchTrueFilter(AngleErrorSequence);
    
    AccVerticalError2=AccVerticalError1;
    AccVerticalError1=AccVerticalError;
    AccVerticalError=((-AccY(SampleSequence)*sin(RollZD(SampleSequence))-AccX(SampleSequence)*cos(RollZD(SampleSequence)))*cos(PitchZD(SampleSequence))-AccZ(SampleSequence)*sin(PitchZD(SampleSequence))-1)*GravityAcc+IMU2AnkleLength*(GyroZ(SampleSequence)-GyroZ(SampleSequence-1))*sin(RollZD(SampleSequence))/dt;
    AccVerticalErrorView(i)=AccVerticalError;
    AccVerticalErrorFilter(AngleErrorSequence)=AngleFilterParameter(4)*AccVerticalErrorFilter(AngleErrorSequence-1)+AngleFilterParameter(5)*AccVerticalErrorFilter(AngleErrorSequence-2)+AngleFilterParameter(1)*AccVerticalError+AngleFilterParameter(2)*AccVerticalError1+AngleFilterParameter(3)*AccVerticalError2;
    AccVerticalErrorFilterView(i)=AccVerticalErrorFilter(ErrorDelayIndex);
    %%
    %initial
    if GaitStart==0
        if GyroFilter(GyroSequence-1)<=SwingThreshold&&GyroFilter(GyroSequence)>SwingThreshold
            GaitStart=1;
            GaitStartTime=i-CalculationDelay;
            Roll=RollAHRS(GyroDelayIndex);
            Pitch=PitchAHRS(GyroDelayIndex);
            Yaw=YawAHRS(GyroDelayIndex)-YawAHRSStart;
            RollZD(SampleSequence)=RollZD(SampleSequence)+Roll-RollZD(GyroDelayIndex);
            PitchZD(SampleSequence)=PitchZD(SampleSequence)+Pitch-PitchZD(GyroDelayIndex);
            YawZD(SampleSequence)=YawZD(SampleSequence)+Yaw-YawZD(GyroDelayIndex);
            q0ZD=cos(RollZD(SampleSequence)/2)*cos(PitchZD(SampleSequence)/2)*cos(YawZD(SampleSequence)/2)+sin(RollZD(SampleSequence)/2)*sin(PitchZD(SampleSequence)/2)*sin(YawZD(SampleSequence)/2);
            q1ZD=sin(RollZD(SampleSequence)/2)*cos(PitchZD(SampleSequence)/2)*cos(YawZD(SampleSequence)/2)-cos(RollZD(SampleSequence)/2)*sin(PitchZD(SampleSequence)/2)*sin(YawZD(SampleSequence)/2);
            q2ZD=cos(RollZD(SampleSequence)/2)*sin(PitchZD(SampleSequence)/2)*cos(YawZD(SampleSequence)/2)+sin(RollZD(SampleSequence)/2)*cos(PitchZD(SampleSequence)/2)*sin(YawZD(SampleSequence)/2);
            q3ZD=-sin(RollZD(SampleSequence)/2)*sin(PitchZD(SampleSequence)/2)*cos(YawZD(SampleSequence)/2)+cos(RollZD(SampleSequence)/2)*cos(PitchZD(SampleSequence)/2)*sin(YawZD(SampleSequence)/2);
            q0=cos(Roll/2)*cos(Pitch/2)*cos(Yaw/2)+sin(Roll/2)*sin(Pitch/2)*sin(Yaw/2);
            q1=sin(Roll/2)*cos(Pitch/2)*cos(Yaw/2)-cos(Roll/2)*sin(Pitch/2)*sin(Yaw/2);
            q2=cos(Roll/2)*sin(Pitch/2)*cos(Yaw/2)+sin(Roll/2)*cos(Pitch/2)*sin(Yaw/2);
            q3=-sin(Roll/2)*sin(Pitch/2)*cos(Yaw/2)+cos(Roll/2)*cos(Pitch/2)*sin(Yaw/2);
            HeelstrikeTime=0;
            ToeOffTime=0;
            ToeOffTem=i-CalculationDelay;
            VelocityFrontToeOffTem=VelocityFront;
            DistanceFrontToeOffTem=DistanceFront;
            ShankAngle=0;
            ShankAngleMax=0;
            ShankAngleMin=0;
            AnkleHeightMax=0;
            GyroZMax=0;
        elseif i>20
            Roll=RollAHRS(GyroDelayIndex);
            RollView(i)=Roll;
            Pitch=PitchAHRS(GyroDelayIndex);
            Yaw=YawAHRS(GyroDelayIndex)-YawAHRSStart;
            AccFront1=-AccY(GyroDelayIndex)*GravityAcc*cos(Roll)+AccX(GyroDelayIndex)*GravityAcc*sin(Roll);
            AccVertical1=-AccY(GyroDelayIndex)*GravityAcc*sin(Roll)-AccX(GyroDelayIndex)*GravityAcc*cos(Roll);
            AccLateral1=AccVertical1*sin(Pitch)+AccZ(GyroDelayIndex)*GravityAcc*cos(Pitch);
            AccVertical=AccVertical1*cos(Pitch)-AccZ(GyroDelayIndex)*GravityAcc*sin(Pitch)-GravityAcc;
            AccFront=AccFront1*cos(Yaw)+AccLateral1*sin(Yaw);
            AccLateral=-AccFront1*sin(Yaw)+AccLateral1*cos(Yaw);
            VelocityFront=VelocityFront+AccFront*dt;
            VelocityVertical=VelocityVertical+AccVertical*dt;
            VelocityLateral=VelocityLateral+AccLateral*dt;
            DistanceFront=DistanceFront+VelocityFront*dt;
            DistanceVertical=DistanceVertical+VelocityVertical*dt;
            DistanceLateral=DistanceLateral+VelocityLateral*dt;
            VelocityFrontView(i)=VelocityFront;
            VelocityVerticalView(i)=VelocityVertical;
            VelocityLateralView(i)=VelocityLateral;
            DistanceFrontView(i)=DistanceFront;
            DistanceVerticalView(i)=DistanceVertical;
            DistanceLateralView(i)=DistanceLateral;
            RollErrorDistance=0;
            AccVerticalErrorDistance=0;
            RollErrorMean=0;
            AccVerticalErrorMean=0;
            for SampleLoop=ErrorDelayIndex-ErrorDistanceNum:ErrorDelayIndex+ErrorDistanceNum
                RollErrorMean=RollErrorMean+RollErrorFilter(SampleLoop);
                AccVerticalErrorMean=AccVerticalErrorMean+AccVerticalErrorFilter(SampleLoop);
            end
            RollErrorMean=RollErrorMean/(2*ErrorDistanceNum+1);
            AccVerticalErrorMean=AccVerticalErrorMean/(2*ErrorDistanceNum+1);
            for SampleLoop=ErrorDelayIndex-ErrorDistanceNum:ErrorDelayIndex+ErrorDistanceNum
                RollErrorDistance=RollErrorDistance+(RollErrorMean-RollErrorFilter(SampleLoop))*(RollErrorMean-RollErrorFilter(SampleLoop));
                AccVerticalErrorDistance=AccVerticalErrorDistance+(AccVerticalErrorMean-AccVerticalErrorFilter(SampleLoop))*(AccVerticalErrorMean-AccVerticalErrorFilter(SampleLoop));
            end
            RollErrorDistance=sqrt(RollErrorDistance/(2*ErrorDistanceNum+1));
            RollErrorDistanceView(i)=RollErrorDistance;
            AccVerticalErrorDistance=sqrt(AccVerticalErrorDistance/(2*ErrorDistanceNum+1));
            AccVerticalErrorDistanceView(i)=AccVerticalErrorDistance;
            if RollErrorDistance<RollErrorDistanceMaxDown&&AccVerticalErrorDistance<AccVerticalErrorDistanceMaxDown
                VelocityFront=-IMU2AnkleLength*GyroZ(GyroDelayIndex)*cos(Roll);
                VelocityLateral=-IMU2AnkleLength*GyroY(GyroDelayIndex)*cos(Pitch)-IMU2AnkleLength*GyroZ(GyroDelayIndex)*sin(Roll)*sin(Pitch);
                VelocityVertical=-IMU2AnkleLength*GyroZ(GyroDelayIndex)*sin(Roll)*cos(Pitch)+IMU2AnkleLength*GyroY(GyroDelayIndex)*sin(Pitch);
                DistanceFront=-sin(Roll)*IMU2AnkleLength;
                DistanceLateral=cos(Roll)*IMU2AnkleLength*sin(Pitch);
                DistanceVertical=cos(Roll)*IMU2AnkleLength*cos(Pitch);
                YawAHRSStart=YawAHRS(GyroDelayIndex);
                IntegrationResetTime=i-CalculationDelay;
                lastIntegrationResetTime=i-CalculationDelay;
                lastZeroStateEndRoll=Roll;
                lastZeroStateEndPitch=Pitch;
                lastIntegrationVerticalResetTime=i-CalculationDelay;
            end
        end
    elseif (HeelstrikeTime~=0&&i-CalculationDelay-HeelstrikeTime>GaitCycleMax&&ThresholdSwitch==0)||(i==length(TrialData(:,1))&&FootIn==1&&ZeroVelocityDetected>0)
        %outputdatastart
        StrideLength=StrideLength-(-sin(RollZ1+RollRemove)*IMU2AnkleLength*cos(YawZ1)+cos(RollZ1+RollRemove)*IMU2AnkleLength*sin(PitchZ1+PitchRemove)*sin(YawZ1));
        StrideLLength=StrideLLength-(cos(RollZ1+RollRemove)*IMU2AnkleLength*sin(PitchZ1+PitchRemove)*cos(YawZ1)+sin(RollZ1+RollRemove)*IMU2AnkleLength*sin(YawZ1));
        StrideVLength=StrideVLength-cos(RollZ1+RollRemove)*IMU2AnkleLength*cos(PitchZ1+PitchRemove);
        AccFrontDiff=-sin(RollRemove)*GravityAcc;
        StrideLengthCom1=(VelocityFrontError-AccFrontDiff*PreStance)*Swing/2;
        StrideLengthCom2=(2*VelocityFrontError-AccFrontDiff*PreStance)*PreStance/2;
        AccLateralDiff=sin(PitchRemove)*cos(RollRemove)*GravityAcc;
        StrideLLengthCom1=(VelocityLateralError-AccLateralDiff*PreStance)*Swing/2;
        StrideLLengthCom2=(2*VelocityLateralError-AccLateralDiff*PreStance)*PreStance/2;
        AccVerticalDiff=(1-cos(RollRemove)*cos(PitchRemove))*GravityAcc;
        AccVerticalCompen=(VelocityVerticalError-AccVerticalDiff*PreVerticalStance)/Swing;
        StrideVLengthCom1=(VelocityVerticalError-AccVerticalDiff*PreVerticalStance)*Swing/2;
        StrideVLengthCom2=(2*VelocityVerticalError-AccVerticalDiff*PreVerticalStance)*PreVerticalStance/2;
        meanRollDiffSwing=-(VelocityFrontError-AccFrontDiff*PreStance)/Swing/GravityAcc;
        StrideVLengthCom3=meanRollDiffSwing*(DistanceFrontHeelstrike-DistanceFrontToeOff-(VelocityFrontToeOff+VelocityFrontHeelstrike)*Swing/2);
        StrideLength=StrideLength+StrideLengthCom1+StrideLengthCom2;
        StrideLLength=StrideLLength+StrideLLengthCom1+StrideLLengthCom2;
        StrideVLength=StrideVLength+StrideVLengthCom1+StrideVLengthCom2;%+StrideVLengthCom3;
        FootHeight=FootHeight+AccVerticalCompen*((FootHeightTime-ToeOffTime)/100.0)*((FootHeightTime-ToeOffTime)/100.0)/2;
        for ii=ToeOffTime+CalculationDelay:HeelstrikeTime+CalculationDelay
            VelocityFrontView(ii)=VelocityFrontView(ii)+(VelocityFrontError-AccFrontDiff*PreStance)*(ii-ToeOffTime-CalculationDelay)*dt/Swing;
            VelocityVerticalView(ii)=VelocityVerticalView(ii)+(VelocityVerticalError-AccVerticalDiff*PreStance)*(ii-ToeOffTime-CalculationDelay)*dt/Swing;
            VelocityLateralView(ii)=VelocityLateralView(ii)+(VelocityLateralError-AccLateralDiff*PreStance)*(ii-ToeOffTime-CalculationDelay)*dt/Swing;
            DistanceFrontView(ii)=DistanceFrontView(ii)+(VelocityFrontError-AccFrontDiff*PreStance)*(ii-ToeOffTime-CalculationDelay)^2*dt^2/Swing/2;
            DistanceVerticalView(ii)=DistanceVerticalView(ii)+(VelocityVerticalError-AccVerticalDiff*PreStance)*(ii-ToeOffTime-CalculationDelay)^2*dt^2/Swing/2;
            DistanceLateralView(ii)=DistanceLateralView(ii)+(VelocityLateralError-AccLateralDiff*PreStance)*(ii-ToeOffTime-CalculationDelay)^2*dt^2/Swing/2;
            RollView(ii)=RollView(ii)+RollRemove*(ii-ToeOffTime-CalculationDelay)*dt/Swing;
            PitchView(ii)=PitchView(ii)+PitchRemove*(ii-ToeOffTime-CalculationDelay)*dt/Swing;
        end
        for ii=ToeOffTime+CalculationDelay:HeelstrikeTime+CalculationDelay
            VelocityVerticalCompensation3View(ii)=meanRollDiffSwing*(VelocityFrontView(ii)-(VelocityFrontToeOff+(VelocityFrontHeelstrike-VelocityFrontToeOff)/Swing*(ii-ToeOffTime-CalculationDelay)*dt));
        end
        for ii=HeelstrikeTime+CalculationDelay+1:IntegrationResetTime+CalculationDelay-1
            RollView(ii)=RollView(ii)+RollRemove;
            PitchView(ii)=PitchView(ii)+PitchRemove;
        end
        for ii=HeelstrikeTime+CalculationDelay+1:HeelstrikeTime+CalculationDelay+PreStance/dt
            VelocityFrontView(ii)=VelocityFrontView(ii)+(VelocityFrontError-AccFrontDiff*PreStance)+AccFrontDiff*(ii-HeelstrikeTime-CalculationDelay)*dt;
            VelocityLateralView(ii)=VelocityLateralView(ii)+(VelocityLateralError-AccLateralDiff*PreStance)+AccLateralDiff*(ii-HeelstrikeTime-CalculationDelay)*dt;
            DistanceFrontView(ii)=DistanceFrontView(ii)+StrideLengthCom1+(2*(VelocityFrontError-AccFrontDiff*PreStance)+AccFrontDiff*(ii-HeelstrikeTime-CalculationDelay)*dt)*(ii-HeelstrikeTime-CalculationDelay)*dt/2;
            DistanceLateralView(ii)=DistanceLateralView(ii)+StrideLLengthCom1+(2*(VelocityLateralError-AccLateralDiff*PreStance)+AccLateralDiff*(ii-HeelstrikeTime-CalculationDelay)*dt)*(ii-HeelstrikeTime-CalculationDelay)*dt/2;
        end
        for ii=HeelstrikeTime+CalculationDelay+1:HeelstrikeTime+CalculationDelay+PreVerticalStance/dt
            VelocityVerticalView(ii)=VelocityVerticalView(ii)+(VelocityVerticalError-AccVerticalDiff*PreVerticalStance)+AccVerticalDiff*(ii-HeelstrikeTime-CalculationDelay)*dt;
            DistanceVerticalView(ii)=DistanceVerticalView(ii)+StrideVLengthCom1+(2*(VelocityVerticalError-AccVerticalDiff*PreVerticalStance)+AccVerticalDiff*(ii-HeelstrikeTime-CalculationDelay)*dt)*(ii-HeelstrikeTime-CalculationDelay)*dt/2;
        end
        StepOutPutCountView=StepOutPutCountView+1;
        StrideLengthCom1View(StepOutPutCountView)=StrideLengthCom1;
        StrideLengthCom2View(StepOutPutCountView)=StrideLengthCom2;
        VelocityFrontErrorView(StepOutPutCountView)=VelocityFrontError;
        PreStanceView(StepOutPutCountView)=PreStance;
%         MidStanceStartTimeView(StepOutPutCountView)=lastMidStanceStartTime;
%         IntegrationResetTimeView(StepOutPutCountView)=lastIntegrationResetTime;
%         ZeroStateEndRollView(StepOutPutCountView)=lastZeroStateEndRoll;
%         ZeroStateEndPitchView(StepOutPutCountView)=lastZeroStateEndPitch;
%         MidStanceVerticalStartTimeView(StepOutPutCountView)=lastMidStanceVerticalStartTime;
%         IntegrationVerticalResetTimeView(StepOutPutCountView)=lastIntegrationVerticalResetTime;
        MidStanceStartTimeView(StepOutPutCountView)=MidStanceStartTime;
        IntegrationResetTimeView(StepOutPutCountView)=IntegrationResetTime;
        ZeroStateEndRollView(StepOutPutCountView)=ZeroStateEndRoll;
        ZeroStateEndPitchView(StepOutPutCountView)=ZeroStateEndPitch;
        MidStanceVerticalStartTimeView(StepOutPutCountView)=MidStanceVerticalStartTime;
        IntegrationVerticalResetTimeView(StepOutPutCountView)=IntegrationVerticalResetTime;
        if StepOutPutCountView==1
            InitialIntegrationView=lastIntegrationResetTime;
        end
        lastMidStanceStartTime=MidStanceStartTime;
        lastIntegrationResetTime=IntegrationResetTime;
        lastZeroStateEndRoll=ZeroStateEndRoll;
        lastZeroStateEndPitch=ZeroStateEndPitch;
        lastMidStanceVerticalStartTime=MidStanceVerticalStartTime;
        lastIntegrationVerticalResetTime=IntegrationVerticalResetTime;
        StrideVLengthCom1View(StepOutPutCountView)=StrideVLengthCom1;
        StrideVLengthCom2View(StepOutPutCountView)=StrideVLengthCom2;
        StrideVLengthCom3View(StepOutPutCountView)=StrideVLengthCom3;
        DiffVerticalVelocityView(StepOutPutCountView)=StrideVLengthCom2/PreStance;
        RollErrorDistanceMaxView(StepOutPutCountView)=RollErrorDistanceMax;
        FootHeightTimeView(StepOutPutCountView)=FootHeightTime;
        StrideLengthView(StepOutPutCountView)=StrideLength;
        StrideLLengthView(StepOutPutCountView)=StrideLLength;
        StrideVLengthView(StepOutPutCountView)=StrideVLength;
        YawRangeView(StepOutPutCountView)=YawRange;
        StrideLengthOut=sqrt(StrideLength^2+StrideLLength^2);
        GaitCycle=(HeelstrikeTime-LastHeelstrikeTime)/100.0;
        GaitSpeed=StrideLengthOut/GaitCycle;
        StancePeriod=(ToeOffTime-LastHeelstrikeTime)/100.0;
        SwingPeriod=(HeelstrikeTime-ToeOffTime)/100.0;
        HeelstrikeTimeView(StepOutPutCountView)=HeelstrikeTime;
        ToeOffTimeView(StepOutPutCountView)=ToeOffTime;
        StrideLengthOutView(StepOutPutCountView)=StrideLengthOut;
        GaitCycleView(StepOutPutCountView)=GaitCycle;
        GaitSpeedView(StepOutPutCountView)=GaitSpeed;
        StancePeriodView(StepOutPutCountView)=StancePeriod;
        SwingPeriodView(StepOutPutCountView)=SwingPeriod;
        MotionRangeShankView(StepOutPutCountView)=MotionRangeShank;
        FootHeightView(StepOutPutCountView)=FootHeight;
        StepCount=StepCount+1;
        StepCountView(StepOutPutCountView)=StepCount;
        UpstairStraightenAngleView(StepOutPutCountView)=UpstairStraightenAngle;
        ShankStanceAngleRangeView(StepOutPutCountView)=ShankStanceAngleRange;
        StandardStrideVLength=StrideVLength/StrideLengthOut;
        StandardMotionRangeShank=MotionRangeShank/StrideLengthOut;
        StandardUpstairStraightenAngle=UpstairStraightenAngle/StrideLengthOut;
        StandardShankStanceAngleRange=ShankStanceAngleRange/StrideLengthOut;
        WalkingStatus=Regress(1)+Regress(2)*StandardStrideVLength+Regress(3)*StandardMotionRangeShank+Regress(4)*StandardUpstairStraightenAngle+Regress(5)*StandardShankStanceAngleRange;
        WalkingStatusView(StepOutPutCountView)=WalkingStatus;
        %outputdataend
        GaitStart=0;
        StepCount=0;
        VelocityFront=0;
        VelocityLateral=0;
        VelocityVertical=0;
        DistanceFront=0;
        DistanceLateral=0;
        DistanceVertical=0;
        YawAHRSStart=YawAHRS(GyroDelayIndex);
    end
    GaitStartView(i)=GaitStart;
    
  if GaitStart==1
    %angle velocity integration
    QuaterX=GyroZ(GyroDelayIndex)/2*dt;
    QuaterY=-GyroY(GyroDelayIndex)/2*dt;
    QuaterZ=-GyroX(GyroDelayIndex)/2*dt;
    lq0=q0;
    lq1=q1;
    lq2=q2;
    lq3=q3;
    q0=lq0-QuaterX*lq1-QuaterY*lq2-QuaterZ*lq3;
    q1=lq1+QuaterX*lq0+QuaterZ*lq2-QuaterY*lq3;
    q2=lq2+QuaterY*lq0-QuaterZ*lq1+QuaterX*lq3;
    q3=lq3+QuaterZ*lq0+QuaterY*lq1-QuaterX*lq2;
    qLength=sqrt(q0^2+q1^2+q2^2+q3^2);
    q0=q0/qLength;
    q1=q1/qLength;
    q2=q2/qLength;
    q3=q3/qLength;
    Pitch=-asin(2*(q1*q3-q0*q2));
    Roll=atan2(2*(q0*q1+q2*q3),(q3^2-q2^2-q1^2+q0^2));
    RollView(i)=Roll;
    PitchView(i)=Pitch;
    Yaw=atan2(2*(q0*q3+q1*q2),(q0^2+q1^2-q2^2-q3^2));
    YawView(i)=Yaw;
    
    %acceleration integration
    AccFront1=-AccY(GyroDelayIndex)*GravityAcc*cos(Roll)+AccX(GyroDelayIndex)*GravityAcc*sin(Roll);
    AccVertical1=-AccY(GyroDelayIndex)*GravityAcc*sin(Roll)-AccX(GyroDelayIndex)*GravityAcc*cos(Roll);
    AccLateral1=AccVertical1*sin(Pitch)+AccZ(GyroDelayIndex)*GravityAcc*cos(Pitch);
    AccVertical=AccVertical1*cos(Pitch)-AccZ(GyroDelayIndex)*GravityAcc*sin(Pitch)-GravityAcc;
    AccFront=AccFront1*cos(Yaw)+AccLateral1*sin(Yaw);
    AccLateral=-AccFront1*sin(Yaw)+AccLateral1*cos(Yaw);
    AccFrontView(i)=AccFront;
    AccVerticalView(i)=AccVertical;
    VelocityFront=VelocityFront+AccFront*dt;
    VelocityVertical=VelocityVertical+AccVertical*dt;
    VelocityLateral=VelocityLateral+AccLateral*dt;
    DistanceFront=DistanceFront+VelocityFront*dt;
    DistanceVertical=DistanceVertical+VelocityVertical*dt;
    DistanceLateral=DistanceLateral+VelocityLateral*dt;
    VelocityFrontView(i)=VelocityFront;
    VelocityVerticalView(i)=VelocityVertical;
    VelocityLateralView(i)=VelocityLateral;
    DistanceFrontView(i)=DistanceFront;
    DistanceVerticalView(i)=DistanceVertical;
    DistanceLateralView(i)=DistanceLateral;
    if FootIn==1&&i-CalculationDelay-HeelstrikeTime>FootInAfterHSMin
        lq0BU=q0BU;
        lq1BU=q1BU;
        lq2BU=q2BU;
        lq3BU=q3BU;
        q0BU=lq0BU-QuaterX*lq1BU-QuaterY*lq2BU-QuaterZ*lq3BU;
        q1BU=lq1BU+QuaterX*lq0BU+QuaterZ*lq2BU-QuaterY*lq3BU;
        q2BU=lq2BU+QuaterY*lq0BU-QuaterZ*lq1BU+QuaterX*lq3BU;
        q3BU=lq3BU+QuaterZ*lq0BU+QuaterY*lq1BU-QuaterX*lq2BU;
        qLength=sqrt(q0BU^2+q1BU^2+q2BU^2+q3BU^2);
        q0BU=q0BU/qLength;
        q1BU=q1BU/qLength;
        q2BU=q2BU/qLength;
        q3BU=q3BU/qLength;
        PitchBU=-asin(2*(q1BU*q3BU-q0BU*q2BU));
        RollBU=atan2(2*(q0BU*q1BU+q2BU*q3BU),(q3BU^2-q2BU^2-q1BU^2+q0BU^2));
        YawBU=atan2(2*(q0BU*q3BU+q1BU*q2BU),(q0BU^2+q1BU^2-q2BU^2-q3BU^2));
        AccFront1=-AccY(GyroDelayIndex)*GravityAcc*cos(RollBU)+AccX(GyroDelayIndex)*GravityAcc*sin(RollBU);
        AccVertical1=-AccY(GyroDelayIndex)*GravityAcc*sin(RollBU)-AccX(GyroDelayIndex)*GravityAcc*cos(RollBU);
        AccLateral1=AccVertical1*sin(PitchBU)+AccZ(GyroDelayIndex)*GravityAcc*cos(PitchBU);
        AccVertical=AccVertical1*cos(PitchBU)-AccZ(GyroDelayIndex)*GravityAcc*sin(PitchBU)-GravityAcc;
        AccFront=AccFront1*cos(YawBU)+AccLateral1*sin(YawBU);
        AccLateral=-AccFront1*sin(YawBU)+AccLateral1*cos(YawBU);
        VelocityFrontBU=VelocityFrontBU+AccFront*dt;
        VelocityVerticalBU=VelocityVerticalBU+AccVertical*dt;
        VelocityLateralBU=VelocityLateralBU+AccLateral*dt;
        DistanceFrontBU=DistanceFrontBU+VelocityFrontBU*dt;
        DistanceVerticalBU=DistanceVerticalBU+VelocityVerticalBU*dt;
        DistanceLateralBU=DistanceLateralBU+VelocityLateralBU*dt;
        VelocityFrontBUView(i)=VelocityFrontBU;
        VelocityVerticalBUView(i)=VelocityVerticalBU;
        VelocityLateralBUView(i)=VelocityLateralBU;
        DistanceFrontBUView(i)=DistanceFrontBU;
        DistanceVerticalBUView(i)=DistanceVerticalBU;
        DistanceLateralBUView(i)=DistanceLateralBU;
        RollBUView(i)=RollBU;
        PitchBUView(i)=PitchBU;
        YawBUView(i)=YawBU;
    end
    if OutFootFlat==0
        lq0IF=q0IF;
        lq1IF=q1IF;
        lq2IF=q2IF;
        lq3IF=q3IF;
        q0IF=lq0IF-QuaterX*lq1IF-QuaterY*lq2IF-QuaterZ*lq3IF;
        q1IF=lq1IF+QuaterX*lq0IF+QuaterZ*lq2IF-QuaterY*lq3IF;
        q2IF=lq2IF+QuaterY*lq0IF-QuaterZ*lq1IF+QuaterX*lq3IF;
        q3IF=lq3IF+QuaterZ*lq0IF+QuaterY*lq1IF-QuaterX*lq2IF;
        qLength=sqrt(q0IF^2+q1IF^2+q2IF^2+q3IF^2);
        q0IF=q0IF/qLength;
        q1IF=q1IF/qLength;
        q2IF=q2IF/qLength;
        q3IF=q3IF/qLength;
        PitchIF=-asin(2*(q1IF*q3IF-q0IF*q2IF));
        RollIF=atan2(2*(q0IF*q1IF+q2IF*q3IF),(q3IF^2-q2IF^2-q1IF^2+q0IF^2));
        YawIF=atan2(2*(q0IF*q3IF+q1IF*q2IF),(q0IF^2+q1IF^2-q2IF^2-q3IF^2));
        AccFront1=-AccY(GyroDelayIndex)*GravityAcc*cos(RollIF)+AccX(GyroDelayIndex)*GravityAcc*sin(RollIF);
        AccVertical1=-AccY(GyroDelayIndex)*GravityAcc*sin(RollIF)-AccX(GyroDelayIndex)*GravityAcc*cos(RollIF);
        AccLateral1=AccVertical1*sin(PitchIF)+AccZ(GyroDelayIndex)*GravityAcc*cos(PitchIF);
        AccVertical=AccVertical1*cos(PitchIF)-AccZ(GyroDelayIndex)*GravityAcc*sin(PitchIF)-GravityAcc;
        AccFront=AccFront1*cos(YawIF)+AccLateral1*sin(YawIF);
        AccLateral=-AccFront1*sin(YawIF)+AccLateral1*cos(YawIF);
        VelocityFrontIF=VelocityFrontIF+AccFront*dt;
        VelocityVerticalIF=VelocityVerticalIF+AccVertical*dt;
        VelocityLateralIF=VelocityLateralIF+AccLateral*dt;
        DistanceFrontIF=DistanceFrontIF+VelocityFrontIF*dt;
        DistanceVerticalIF=DistanceVerticalIF+VelocityVerticalIF*dt;
        DistanceLateralIF=DistanceLateralIF+VelocityLateralIF*dt;
        VelocityFrontIFView(i)=VelocityFrontIF;
        VelocityVerticalIFView(i)=VelocityVerticalIF;
        VelocityLateralIFView(i)=VelocityLateralIF;
        DistanceFrontIFView(i)=DistanceFrontIF;
        DistanceVerticalIFView(i)=DistanceVerticalIF;
        DistanceLateralIFView(i)=DistanceLateralIF;
        RollIFView(i)=RollIF;
        PitchIFView(i)=PitchIF;
        YawIFView(i)=YawIF;
    end
    %motion of range of shank
    ShankAngle=ShankAngle+GyroZ(GyroDelayIndex)*dt;
    if ShankAngle>ShankAngleMax
        ShankAngleMax=ShankAngle;
    elseif ShankAngle<ShankAngleMin
        ShankAngleMin=ShankAngle;
    end
    if GyroZ(GyroDelayIndex)<0
        ShankStanceAngle=ShankStanceAngle+GyroZ(GyroDelayIndex)*dt;
    else
        ShankStanceStraightenAngle=ShankStanceStraightenAngle+GyroZ(GyroDelayIndex)*dt;
    end
    %max height and velocity
    if ThresholdSwitch==0
        AnkleHeight=DistanceVerticalBU-cos(RollBU)*IMU2AnkleLength*cos(PitchBU);
    else
        AnkleHeight=DistanceVertical-cos(Roll)*IMU2AnkleLength*cos(Pitch);
    end
    if AnkleHeight>AnkleHeightMax||AnkleHeightMax==0
        AnkleHeightMax=AnkleHeight;
        AnkleHeightMaxTime=i-CalculationDelay;
    end
    if GyroZ(GyroDelayIndex)>GyroZMax
        GyroZMax=GyroZ(GyroDelayIndex);
    end
    %detect gait event
    if ThresholdSwitch==1
        if GyroZ(GyroDelayIndex)<=0&&GyroZ(GyroDelayIndex+1)>GyroZ(GyroDelayIndex)
            if MaxGyroZ>3*SwingThreshold
                SecondMaxLimit=2*SwingThreshold;
            elseif MaxGyroZ>2*SwingThreshold
                SecondMaxLimit=MaxGyroZ-SwingThreshold;
            else
                SecondMaxLimit=SwingThreshold;
            end
            for maxi=GyroFilterDelayIndex:GyroSequence
                if GyroFilter(maxi)>SecondMaxLimit
                    break;
                end
                if maxi==GyroSequence
                    if HeelstrikeTime~=0
                        LastHeelstrikeTime=HeelstrikeTime;
                    else
                        LastHeelstrikeTime=GaitStartTime;
                    end
                    HeelstrikeTime=i-CalculationDelay;
                    VelocityFrontHeelstrike=VelocityFront;
                    DistanceFrontHeelstrike=DistanceFront;
                    ThresholdSwitch=0;
                    FootIn=1;
                    ZeroVelocityDetected=0;
                    RollErrorDistanceMin=0;
                    RollRemove=0;
                    PitchRemove=0;
                    MotionRangeShank=ShankAngleMax-ShankAngleMin;
                    FootHeightTem=AnkleHeightMax;
                    FootHeightTime=AnkleHeightMaxTime;
                    ShankStanceAngle=0;
                    ShankStanceStraightenAngle=0;
                end
            end
        end
    elseif GyroFilter(GyroFilterDelayIndex-1)<=SwingThreshold&&GyroFilter(GyroFilterDelayIndex)>SwingThreshold
        if ToeOffTem~=0
            LastMaxGyroZ=MaxGyroZ;
            MaxGyroZ=0;
            for maxi=1:SampleSequence
                if MaxGyroZ<GyroZ(maxi)
                    MaxGyroZ=GyroZ(maxi);
                end
            end
            if HeelstrikeTime~=0&&ToeOffTem-HeelstrikeTime<20&&HeelstrikeTime~=0
                if LastMaxGyroZ-MaxGyroZ>=0.5&&MaxGyroZ<2*SwingThreshold
                    MaxGyroZ=LastMaxGyroZ;
                else
                    %not walking
                    GaitStart=0;
                    StepCount=0;
                    VelocityFront=0;
                    VelocityLateral=0;
                    VelocityVertical=0;
                    DistanceFront=0;
                    DistanceLateral=0;
                    DistanceVertical=0;
                    YawAHRSStart=YawAHRS(GyroDelayIndex);
                end
            else
                ThresholdSwitch=1;
                FootIn=0;
                if HeelstrikeTime~=0
                    %outputdatastart
                    StrideLength=StrideLength-(-sin(RollZ1+RollRemove)*IMU2AnkleLength*cos(YawZ1)+cos(RollZ1+RollRemove)*IMU2AnkleLength*sin(PitchZ1+PitchRemove)*sin(YawZ1));
                    StrideLLength=StrideLLength-(cos(RollZ1+RollRemove)*IMU2AnkleLength*sin(PitchZ1+PitchRemove)*cos(YawZ1)+sin(RollZ1+RollRemove)*IMU2AnkleLength*sin(YawZ1));
                    StrideVLength=StrideVLength-cos(RollZ1+RollRemove)*IMU2AnkleLength*cos(PitchZ1+PitchRemove);
                    AccFrontDiff=-sin(RollRemove)*GravityAcc;
                    StrideLengthCom1=(VelocityFrontError-AccFrontDiff*PreStance)*Swing/2;
                    StrideLengthCom2=(2*VelocityFrontError-AccFrontDiff*PreStance)*PreStance/2;
                    AccLateralDiff=sin(PitchRemove)*cos(RollRemove)*GravityAcc;
                    StrideLLengthCom1=(VelocityLateralError-AccLateralDiff*PreStance)*Swing/2;
                    StrideLLengthCom2=(2*VelocityLateralError-AccLateralDiff*PreStance)*PreStance/2;
                    AccVerticalDiff=(1-cos(RollRemove)*cos(PitchRemove))*GravityAcc;
                    AccVerticalCompen=(VelocityVerticalError-AccVerticalDiff*PreVerticalStance)/Swing;
                    StrideVLengthCom1=(VelocityVerticalError-AccVerticalDiff*PreVerticalStance)*Swing/2;
                    StrideVLengthCom2=(2*VelocityVerticalError-AccVerticalDiff*PreVerticalStance)*PreVerticalStance/2;
                    meanRollDiffSwing=-(VelocityFrontError-AccFrontDiff*PreStance)/Swing/GravityAcc;
                    StrideVLengthCom3=meanRollDiffSwing*(DistanceFrontHeelstrike-DistanceFrontToeOff-(VelocityFrontToeOff+VelocityFrontHeelstrike)*Swing/2);
                    StrideLength=StrideLength+StrideLengthCom1+StrideLengthCom2;
                    StrideLLength=StrideLLength+StrideLLengthCom1+StrideLLengthCom2;
                    StrideVLength=StrideVLength+StrideVLengthCom1+StrideVLengthCom2;%+StrideVLengthCom3;
                    FootHeight=FootHeight+AccVerticalCompen*((FootHeightTime-ToeOffTime)/100.0)*((FootHeightTime-ToeOffTime)/100.0)/2;
                    for ii=ToeOffTime+CalculationDelay:HeelstrikeTime+CalculationDelay
                        VelocityFrontView(ii)=VelocityFrontView(ii)+(VelocityFrontError-AccFrontDiff*PreStance)*(ii-ToeOffTime-CalculationDelay)*dt/Swing;
                        VelocityVerticalView(ii)=VelocityVerticalView(ii)+(VelocityVerticalError-AccVerticalDiff*PreStance)*(ii-ToeOffTime-CalculationDelay)*dt/Swing;
                        VelocityLateralView(ii)=VelocityLateralView(ii)+(VelocityLateralError-AccLateralDiff*PreStance)*(ii-ToeOffTime-CalculationDelay)*dt/Swing;
                        DistanceFrontView(ii)=DistanceFrontView(ii)+(VelocityFrontError-AccFrontDiff*PreStance)*(ii-ToeOffTime-CalculationDelay)^2*dt^2/Swing/2;
                        DistanceVerticalView(ii)=DistanceVerticalView(ii)+(VelocityVerticalError-AccVerticalDiff*PreStance)*(ii-ToeOffTime-CalculationDelay)^2*dt^2/Swing/2;
                        DistanceLateralView(ii)=DistanceLateralView(ii)+(VelocityLateralError-AccLateralDiff*PreStance)*(ii-ToeOffTime-CalculationDelay)^2*dt^2/Swing/2;
                        RollView(ii)=RollView(ii)+RollRemove*(ii-ToeOffTime-CalculationDelay)*dt/Swing;
                        PitchView(ii)=PitchView(ii)+PitchRemove*(ii-ToeOffTime-CalculationDelay)*dt/Swing;
                    end
                    for ii=ToeOffTime+CalculationDelay:HeelstrikeTime+CalculationDelay
                        VelocityVerticalCompensation3View(ii)=meanRollDiffSwing*(VelocityFrontView(ii)-(VelocityFrontToeOff+(VelocityFrontHeelstrike-VelocityFrontToeOff)/Swing*(ii-ToeOffTime-CalculationDelay)*dt));
                    end
                    for ii=HeelstrikeTime+CalculationDelay+1:IntegrationResetTime+CalculationDelay
                        RollView(ii)=RollView(ii)+RollRemove;
                        PitchView(ii)=PitchView(ii)+PitchRemove;
                    end
                    for ii=HeelstrikeTime+CalculationDelay+1:HeelstrikeTime+CalculationDelay+PreStance/dt
                        VelocityFrontView(ii)=VelocityFrontView(ii)+(VelocityFrontError-AccFrontDiff*PreStance)+AccFrontDiff*(ii-HeelstrikeTime-CalculationDelay)*dt;
                        VelocityLateralView(ii)=VelocityLateralView(ii)+(VelocityLateralError-AccLateralDiff*PreStance)+AccLateralDiff*(ii-HeelstrikeTime-CalculationDelay)*dt;
                        DistanceFrontView(ii)=DistanceFrontView(ii)+StrideLengthCom1+(2*(VelocityFrontError-AccFrontDiff*PreStance)+AccFrontDiff*(ii-HeelstrikeTime-CalculationDelay)*dt)*(ii-HeelstrikeTime-CalculationDelay)*dt/2;
                        DistanceLateralView(ii)=DistanceLateralView(ii)+StrideLLengthCom1+(2*(VelocityLateralError-AccLateralDiff*PreStance)+AccLateralDiff*(ii-HeelstrikeTime-CalculationDelay)*dt)*(ii-HeelstrikeTime-CalculationDelay)*dt/2;
                    end
                    for ii=HeelstrikeTime+CalculationDelay+1:HeelstrikeTime+CalculationDelay+PreVerticalStance/dt
                        VelocityVerticalView(ii)=VelocityVerticalView(ii)+(VelocityVerticalError-AccVerticalDiff*PreVerticalStance)+AccVerticalDiff*(ii-HeelstrikeTime-CalculationDelay)*dt;
                        DistanceVerticalView(ii)=DistanceVerticalView(ii)+StrideVLengthCom1+(2*(VelocityVerticalError-AccVerticalDiff*PreVerticalStance)+AccVerticalDiff*(ii-HeelstrikeTime-CalculationDelay)*dt)*(ii-HeelstrikeTime-CalculationDelay)*dt/2;
                    end
                    
                    StepOutPutCountView=StepOutPutCountView+1;
                    StrideLengthCom1View(StepOutPutCountView)=StrideLengthCom1;
                    StrideLengthCom2View(StepOutPutCountView)=StrideLengthCom2;
                    VelocityFrontErrorView(StepOutPutCountView)=VelocityFrontError;
                    PreStanceView(StepOutPutCountView)=PreStance;
%                     MidStanceStartTimeView(StepOutPutCountView)=lastMidStanceStartTime;
%                     IntegrationResetTimeView(StepOutPutCountView)=lastIntegrationResetTime;
%                     ZeroStateEndRollView(StepOutPutCountView)=lastZeroStateEndRoll;
%                     ZeroStateEndPitchView(StepOutPutCountView)=lastZeroStateEndPitch;
%                     MidStanceVerticalStartTimeView(StepOutPutCountView)=lastMidStanceVerticalStartTime;
%                     IntegrationVerticalResetTimeView(StepOutPutCountView)=lastIntegrationVerticalResetTime;
                    MidStanceStartTimeView(StepOutPutCountView)=MidStanceStartTime;
                    IntegrationResetTimeView(StepOutPutCountView)=IntegrationResetTime;
                    ZeroStateEndRollView(StepOutPutCountView)=ZeroStateEndRoll;
                    ZeroStateEndPitchView(StepOutPutCountView)=ZeroStateEndPitch;
                    MidStanceVerticalStartTimeView(StepOutPutCountView)=MidStanceVerticalStartTime;
                    IntegrationVerticalResetTimeView(StepOutPutCountView)=IntegrationVerticalResetTime;
                    if StepOutPutCountView==1
                        InitialIntegrationView=lastIntegrationResetTime;
                    end
                    lastMidStanceStartTime=MidStanceStartTime;
                    lastIntegrationResetTime=IntegrationResetTime;
                    lastZeroStateEndRoll=ZeroStateEndRoll;
                    lastZeroStateEndPitch=ZeroStateEndPitch;
                    lastMidStanceVerticalStartTime=MidStanceVerticalStartTime;
                    lastIntegrationVerticalResetTime=IntegrationVerticalResetTime;
                    StrideVLengthCom1View(StepOutPutCountView)=StrideVLengthCom1;
                    StrideVLengthCom2View(StepOutPutCountView)=StrideVLengthCom2;
                    StrideVLengthCom3View(StepOutPutCountView)=StrideVLengthCom3;
                    DiffVerticalVelocityView(StepOutPutCountView)=StrideVLengthCom2/PreStance;
                    RollErrorDistanceMaxView(StepOutPutCountView)=RollErrorDistanceMax;
                    FootHeightTimeView(StepOutPutCountView)=FootHeightTime;
                    StrideLengthView(StepOutPutCountView)=StrideLength;
                    StrideLLengthView(StepOutPutCountView)=StrideLLength;
                    StrideVLengthView(StepOutPutCountView)=StrideVLength;
                    YawRangeView(StepOutPutCountView)=YawRange;
                    StrideLengthOut=sqrt(StrideLength^2+StrideLLength^2);
                    GaitCycle=(HeelstrikeTime-LastHeelstrikeTime)/100.0;
                    GaitSpeed=StrideLengthOut/GaitCycle;
                    StancePeriod=(ToeOffTime-LastHeelstrikeTime)/100.0;
                    SwingPeriod=(HeelstrikeTime-ToeOffTime)/100.0;
                    HeelstrikeTimeView(StepOutPutCountView)=HeelstrikeTime;
                    ToeOffTimeView(StepOutPutCountView)=ToeOffTime;
                    StrideLengthOutView(StepOutPutCountView)=StrideLengthOut;
                    GaitCycleView(StepOutPutCountView)=GaitCycle;
                    GaitSpeedView(StepOutPutCountView)=GaitSpeed;
                    StancePeriodView(StepOutPutCountView)=StancePeriod;
                    SwingPeriodView(StepOutPutCountView)=SwingPeriod;
                    MotionRangeShankView(StepOutPutCountView)=MotionRangeShank;
                    FootHeightView(StepOutPutCountView)=FootHeight;
                    StepCount=StepCount+1;
                    StepCountView(StepOutPutCountView)=StepCount;
                    UpstairStraightenAngleView(StepOutPutCountView)=UpstairStraightenAngle;
                    ShankStanceAngleRangeView(StepOutPutCountView)=ShankStanceAngleRange;
                    StandardStrideVLength=StrideVLength/StrideLengthOut;
                    StandardMotionRangeShank=MotionRangeShank/StrideLengthOut;
                    StandardUpstairStraightenAngle=UpstairStraightenAngle/StrideLengthOut;
                    StandardShankStanceAngleRange=ShankStanceAngleRange/StrideLengthOut;
                    WalkingStatus=Regress(1)+Regress(2)*StandardStrideVLength+Regress(3)*StandardMotionRangeShank+Regress(4)*StandardUpstairStraightenAngle+Regress(5)*StandardShankStanceAngleRange;
                    WalkingStatusView(StepOutPutCountView)=WalkingStatus;
                    %outputdataend
                    q0=q0BU;
                    q1=q1BU;
                    q2=q2BU;
                    q3=q3BU;
                    VelocityFront=VelocityFrontBU;
                    VelocityVertical=VelocityVerticalBU;
                    VelocityLateral=VelocityLateralBU;
                    DistanceFront=DistanceFrontBU;
                    DistanceVertical=DistanceVerticalBU;
                    DistanceLateral=DistanceLateralBU;
                    RollZD(SampleSequence)=RollZD(SampleSequence)+RollBU-RollZD(GyroDelayIndex);
                    PitchZD(SampleSequence)=PitchZD(SampleSequence)+PitchBU-PitchZD(GyroDelayIndex);
                    YawZD(SampleSequence)=YawZD(SampleSequence)+YawBU-YawZD(GyroDelayIndex);
                    q0ZD=cos(RollZD(SampleSequence)/2)*cos(PitchZD(SampleSequence)/2)*cos(YawZD(SampleSequence)/2)+sin(RollZD(SampleSequence)/2)*sin(PitchZD(SampleSequence)/2)*sin(YawZD(SampleSequence)/2);
                    q1ZD=sin(RollZD(SampleSequence)/2)*cos(PitchZD(SampleSequence)/2)*cos(YawZD(SampleSequence)/2)-cos(RollZD(SampleSequence)/2)*sin(PitchZD(SampleSequence)/2)*sin(YawZD(SampleSequence)/2);
                    q2ZD=cos(RollZD(SampleSequence)/2)*sin(PitchZD(SampleSequence)/2)*cos(YawZD(SampleSequence)/2)+sin(RollZD(SampleSequence)/2)*cos(PitchZD(SampleSequence)/2)*sin(YawZD(SampleSequence)/2);
                    q3ZD=-sin(RollZD(SampleSequence)/2)*sin(PitchZD(SampleSequence)/2)*cos(YawZD(SampleSequence)/2)+cos(RollZD(SampleSequence)/2)*cos(PitchZD(SampleSequence)/2)*sin(YawZD(SampleSequence)/2);
                    for ii=MidStanceStartTime+CalculationDelay+1:i
                        VelocityFrontView(ii)=VelocityFrontBUView(ii);
                        VelocityLateralView(ii)=VelocityLateralBUView(ii);
                        DistanceFrontView(ii)=DistanceFrontBUView(ii);
                        DistanceLateralView(ii)=DistanceLateralBUView(ii);
                    end
                    for ii=MidStanceVerticalStartTime+CalculationDelay+1:i
                        VelocityVerticalView(ii)=VelocityVerticalBUView(ii);
                        DistanceVerticalView(ii)=DistanceVerticalBUView(ii);
                    end
                    for ii=IntegrationResetTime+CalculationDelay+1:i
                        RollView(ii)=RollBUView(ii);
                        PitchView(ii)=PitchBUView(ii);
                        YawView(ii)=YawBUView(ii);
                    end
                    
                end
                ToeOffTime=ToeOffTem;
                VelocityFrontToeOff=VelocityFrontToeOffTem;
                DistanceFrontToeOff=DistanceFrontToeOffTem;
                ToeOffTem=0;
                UpstairStraightenAngle=ShankStanceStraightenAngleTem;
                ShankStanceAngleRange=-(ShankStanceAngleTem+ShankStanceStraightenAngleTem);
            end 
        end
    elseif GyroFilter(GyroFilterDelayIndex)<0
        if GyroFilter(GyroFilterDelayIndex-1)>GyroFilter(GyroFilterDelayIndex)
            if GyroFilter(GyroFilterDelayIndex+1)>GyroFilter(GyroFilterDelayIndex)
                ToeOffTem=i-CalculationDelay;
                if HeelstrikeTime==0
                    VelocityFrontToeOffTem=VelocityFront;
                    DistanceFrontToeOffTem=DistanceFront;
                else
                    VelocityFrontToeOffTem=VelocityFrontBU;
                    DistanceFrontToeOffTem=DistanceFrontBU;
                end
                ShankAngle=0;
                ShankAngleMax=0;
                ShankAngleMin=0;
                AnkleHeightMax=0;
                ShankStanceAngleTem=ShankStanceAngle;
                ShankStanceStraightenAngleTem=ShankStanceStraightenAngle;
            end
        end
    end
    %Zero velocity detection
    if FootIn==1&&i-CalculationDelay-HeelstrikeTime>=FootInAfterHSMin
        RollErrorDistance=0;
        AccVerticalErrorDistance=0;
        RollErrorMean=0;
        AccVerticalErrorMean=0;
        AccVerticalErrorSqMean=0;
        for SampleLoop=ErrorDelayIndex-ErrorDistanceNum:ErrorDelayIndex+ErrorDistanceNum
            RollErrorMean=RollErrorMean+RollErrorFilter(SampleLoop);
            AccVerticalErrorMean=AccVerticalErrorMean+AccVerticalErrorFilter(SampleLoop);
            AccVerticalErrorSqMean=AccVerticalErrorSqMean+AccVerticalErrorFilter(SampleLoop)*AccVerticalErrorFilter(SampleLoop);
        end
        RollErrorMean=RollErrorMean/(2*ErrorDistanceNum+1);
        AccVerticalErrorMean=AccVerticalErrorMean/(2*ErrorDistanceNum+1);
        AccVerticalErrorSqMean=AccVerticalErrorSqMean/(2*ErrorDistanceNum+1);
        AccVerticalErrorSqMeanView(i)=AccVerticalErrorSqMean;
        for SampleLoop=ErrorDelayIndex-ErrorDistanceNum:ErrorDelayIndex+ErrorDistanceNum
            RollErrorDistance=RollErrorDistance+(RollErrorMean-RollErrorFilter(SampleLoop))*(RollErrorMean-RollErrorFilter(SampleLoop));
            AccVerticalErrorDistance=AccVerticalErrorDistance+(AccVerticalErrorMean-AccVerticalErrorFilter(SampleLoop))*(AccVerticalErrorMean-AccVerticalErrorFilter(SampleLoop));
        end
        RollErrorDistance=sqrt(RollErrorDistance/(2*ErrorDistanceNum+1));
        RollErrorDistanceView(i)=RollErrorDistance;
        AccVerticalErrorDistance=sqrt(AccVerticalErrorDistance/(2*ErrorDistanceNum+1));
        AccVerticalErrorDistanceView(i)=AccVerticalErrorDistance;
        if i-CalculationDelay-HeelstrikeTime==FootInAfterHSMin
            RollErrorDistanceMax=RollErrorDistanceMaxDown+(GyroZMax-GyroZDown)*RollErrorDistanceMaxRatio;
            if RollErrorDistanceMax>RollErrorDistanceMaxUp
                RollErrorDistanceMax=RollErrorDistanceMaxUp;
            elseif RollErrorDistanceMax<RollErrorDistanceMaxDown
                RollErrorDistanceMax=RollErrorDistanceMaxDown;
            end
            AccVerticalErrorDistanceMax=RollErrorDistanceMax*AccVerticalRollErrorRatio;
            RollErrorFilterThreshold=RollErrorDistanceMax*1.5;
            AccVerticalErrorFilterThreshold=AccVerticalErrorDistanceMax*1.5;
            GyroZMax=0;
            OutFootFlat=0;
            FlatTimeMax=-1;
            ZeroVertical=0;
            AccVerticalErrorSqMeanMin=0;
        end
        if (OutFootFlat==0&&ZeroVelocityDetected~=0&&(RollErrorDistance>RollErrorFilterThreshold||RollErrorDistance>RollErrorDistanceFlatMin+RollErrorDistanceMax))||(OutFootFlat==0&&ZeroVelocityDetected~=0&&i==length(TrialData(:,1))-1)
            OutFootFlat=1;
            IFtoBU=1;
        end
        if OutFootFlat==0&&ZeroVelocityDetected~=0&&((GyroFilter(GyroFilterDelayIndex)<=SwingThreshold&&GyroFilter(GyroFilterDelayIndex+1)>SwingThreshold)||(i-CalculationDelay-HeelstrikeTime+1>GaitCycleMax))
            OutFootFlat=1;
            IFtoBU=1;
        end
        if IFtoBU==1
            IFtoBU=0;
            FlatTimeIF=IntegrationResetTimeIF-MidStanceStartTimeIF;
            if FlatTimeIF>FlatTimeMax
                FlatTimeMax=FlatTimeIF;
                q0BU=q0IF;
                q1BU=q1IF;
                q2BU=q2IF;
                q3BU=q3IF;
                VelocityFrontBU=VelocityFrontIF;
                VelocityVerticalBU=VelocityVerticalIF;
                VelocityLateralBU=VelocityLateralIF;
                DistanceFrontBU=DistanceFrontIF;
                DistanceVerticalBU=DistanceVerticalIF;
                DistanceLateralBU=DistanceLateralIF;
                IntegrationResetTime=IntegrationResetTimeIF;
                MidStanceStartTime=MidStanceStartTimeIF;
                ZeroStateEndRoll=ZeroStateEndRollIF;
                ZeroStateEndPitch=ZeroStateEndPitchIF;
                IntegrationVerticalResetTime=IntegrationVerticalResetTimeIF;
                MidStanceVerticalStartTime=MidStanceVerticalStartTimeIF;
                PreStance=PreStanceIF;
                PreVerticalStance=PreVerticalStanceIF;
                Swing=SwingIF;
                VelocityFrontError=VelocityFrontErrorIF;
                VelocityLateralError=VelocityLateralErrorIF;
                VelocityVerticalError=VelocityVerticalErrorIF;
                RollZ1=RollZ1IF;
                PitchZ1=PitchZ1IF;
                YawZ1=YawZ1IF;
                StrideLength=StrideLengthIF;
                StrideVLength=StrideVLengthIF;
                StrideLLength=StrideLLengthIF;
                FootHeight=FootHeightIF;
                RollRemove=RollRemoveIF;
                PitchRemove=PitchRemoveIF;
                YawRange=YawRangeIF;
                for ii=MidStanceStartTime+CalculationDelay+1:i
                    VelocityFrontBUView(ii)=VelocityFrontIFView(ii);
                    VelocityVerticalBUView(ii)=VelocityVerticalIFView(ii);
                    VelocityLateralBUView(ii)=VelocityLateralIFView(ii);
                    DistanceFrontBUView(ii)=DistanceFrontIFView(ii);
                    DistanceVerticalBUView(ii)=DistanceVerticalIFView(ii);
                    DistanceLateralBUView(ii)=DistanceLateralIFView(ii);
                    RollBUView(ii)=RollIFView(ii);
                    PitchBUView(ii)=PitchIFView(ii);
                    YawBUView(ii)=YawIFView(ii);
                end
            end
        end
        if RollErrorDistance<RollErrorDistanceMax
            if OutFootFlat==1
                ZeroVelocityDetected=0;
                OutFootFlat=0;
                ZeroVertical=0;
                AccVerticalErrorSqMeanMin=0;
            end
            ZeroVelocityDetected=ZeroVelocityDetected+1;
            if ZeroVelocityDetected==1||RollErrorDistanceFlatMin>RollErrorDistance
                RollErrorDistanceFlatMin=RollErrorDistance;
            end
            if ZeroVelocityDetected==1
                RollRemoveIF=0;
                PitchRemoveIF=0;
                RollIF=Roll;
                PitchIF=Pitch;
            end
            RollIF=RollIF-RollRemoveIF;
            RollRemoveIF=(RollRemoveIF*(ZeroVelocityDetected-1)+RollTrueFilter(ErrorDelayIndex)-RollIF)/ZeroVelocityDetected;
            RollIF=RollIF+RollRemoveIF;
            PitchIF=PitchIF-PitchRemoveIF;
            PitchRemoveIF=(PitchRemoveIF*(ZeroVelocityDetected-1)+PitchTrueFilter(ErrorDelayIndex)-PitchIF)/ZeroVelocityDetected;
            PitchIF=PitchIF+PitchRemoveIF;
            YawRangeIF=Yaw;
            YawIF=0;
            q0IF=cos(RollIF/2)*cos(PitchIF/2)*cos(YawIF/2)+sin(RollIF/2)*sin(PitchIF/2)*sin(YawIF/2);
            q1IF=sin(RollIF/2)*cos(PitchIF/2)*cos(YawIF/2)-cos(RollIF/2)*sin(PitchIF/2)*sin(YawIF/2);
            q2IF=cos(RollIF/2)*sin(PitchIF/2)*cos(YawIF/2)+sin(RollIF/2)*cos(PitchIF/2)*sin(YawIF/2);
            q3IF=-sin(RollIF/2)*sin(PitchIF/2)*cos(YawIF/2)+cos(RollIF/2)*cos(PitchIF/2)*sin(YawIF/2);
            if ZeroVelocityDetected==1
                VelocityFrontIF=-IMU2AnkleLength*GyroZ(GyroDelayIndex)*cos(RollIF);
                VelocityLateralIF=-IMU2AnkleLength*GyroY(GyroDelayIndex)*cos(PitchIF)-IMU2AnkleLength*GyroZ(GyroDelayIndex)*sin(RollIF)*sin(PitchIF);
                PreStanceIF=(i-CalculationDelay-HeelstrikeTime)/100.0;
                SwingIF=(HeelstrikeTime-ToeOffTime)/100.0;
                
                VelocityFrontErrorIF=VelocityFrontIF*cos(Yaw)+VelocityLateralIF*sin(Yaw)-VelocityFront;
                VelocityLateralErrorIF=VelocityLateralIF*cos(Yaw)-VelocityFrontIF*sin(Yaw)-VelocityLateral;
                
                DistanceFrontIF=-sin(RollIF)*IMU2AnkleLength;
                DistanceLateralIF=cos(RollIF)*IMU2AnkleLength*sin(PitchIF);
                VelocityFrontIFView(i)=VelocityFrontIF;
                VelocityLateralIFView(i)=VelocityLateralIF;
                DistanceFrontIFView(i)=DistanceFrontIF;
                DistanceLateralIFView(i)=DistanceLateralIF;
                RollIFView(i)=RollIF;
                PitchIFView(i)=PitchIF;
                YawIFView(i)=YawIF;
                RollZ1IF=Roll;
                PitchZ1IF=Pitch;
                YawZ1IF=Yaw;
                StrideLengthIF=DistanceFront;%-(-sin(RollIF)*IMU2AnkleLength*cos(Yaw)+cos(RollIF)*IMU2AnkleLength*sin(PitchIF)*sin(Yaw));
                StrideLLengthIF=DistanceLateral;%-(cos(RollIF)*IMU2AnkleLength*sin(PitchIF)*cos(Yaw)+sin(RollIF)*IMU2AnkleLength*sin(Yaw));
                FootHeightIF=FootHeightTem;
%                 FootHeight=FootHeight-StrideVLength*(FootHeightTime-IntegrationResetTime)/(i-CalculationDelay-IntegrationResetTime);
                IntegrationResetTimeIF=i-CalculationDelay;
                MidStanceStartTimeIF=i-CalculationDelay;
                ZeroStateEndRollIF=RollIF;
                ZeroStateEndPitchIF=PitchIF;
                %%%%Vertical
                VelocityVerticalIF=-IMU2AnkleLength*GyroZ(GyroDelayIndex)*sin(RollIF)*cos(PitchIF)+IMU2AnkleLength*GyroY(GyroDelayIndex)*sin(PitchIF);
                PreVerticalStanceIF=(i-CalculationDelay-HeelstrikeTime)/100.0;
                VelocityVerticalErrorIF=VelocityVerticalIF-VelocityVertical;
                DistanceVerticalIF=cos(RollIF)*IMU2AnkleLength*cos(PitchIF);
                VelocityVerticalIFView(i)=VelocityVerticalIF;
                DistanceVerticalIFView(i)=DistanceVerticalIF;
                StrideVLengthIF=DistanceVertical;%-DistanceVerticalIF;
                IntegrationVerticalResetTimeIF=i-CalculationDelay;
                MidStanceVerticalStartTimeIF=i-CalculationDelay;
            else
                VelocityFrontIF=-IMU2AnkleLength*GyroZ(GyroDelayIndex)*cos(RollIF);
                VelocityLateralIF=-IMU2AnkleLength*GyroY(GyroDelayIndex)*cos(PitchIF)-IMU2AnkleLength*GyroZ(GyroDelayIndex)*sin(RollIF)*sin(PitchIF);
                DistanceFrontIF=-sin(RollIF)*IMU2AnkleLength;
                DistanceLateralIF=cos(RollIF)*IMU2AnkleLength*sin(PitchIF);
                VelocityFrontIFView(i)=VelocityFrontIF;
                VelocityLateralIFView(i)=VelocityLateralIF;
                DistanceFrontIFView(i)=DistanceFrontIF;
                DistanceLateralIFView(i)=DistanceLateralIF;
                RollIFView(i)=RollIF;
                PitchIFView(i)=PitchIF;
                YawIFView(i)=YawIF;
                IntegrationResetTimeIF=i-CalculationDelay;
                ZeroStateEndRollIF=RollIF;
                ZeroStateEndPitchIF=PitchIF;
                %%%%Vertical
                VelocityVerticalIF=-IMU2AnkleLength*GyroZ(GyroDelayIndex)*sin(RollIF)*cos(PitchIF)+IMU2AnkleLength*GyroY(GyroDelayIndex)*sin(PitchIF);
                DistanceVerticalIF=cos(RollIF)*IMU2AnkleLength*cos(PitchIF);
                VelocityVerticalIFView(i)=VelocityVerticalIF;
                DistanceVerticalIFView(i)=DistanceVerticalIF;
                IntegrationVerticalResetTimeIF=i-CalculationDelay;
            end
%             if AccVerticalErrorDistance<AccVerticalErrorDistanceMax
%                 ZeroVertical=ZeroVertical+1;
%                 if ZeroVertical==1
%                     VelocityVerticalIF=-IMU2AnkleLength*GyroZ(GyroDelayIndex)*sin(RollIF)*cos(PitchIF)+IMU2AnkleLength*GyroY(GyroDelayIndex)*sin(PitchIF);
%                     PreVerticalStanceIF=(i-CalculationDelay-HeelstrikeTime)/100.0;
%                     VelocityVerticalErrorIF=VelocityVerticalIF-VelocityVertical;
%                     DistanceVerticalIF=cos(RollIF)*IMU2AnkleLength*cos(PitchIF);
%                     VelocityVerticalIFView(i)=VelocityVerticalIF;
%                     DistanceVerticalIFView(i)=DistanceVerticalIF;
%                     StrideVLengthIF=DistanceVertical-DistanceVerticalIF;
%                     IntegrationVerticalResetTimeIF=i-CalculationDelay;
%                     MidStanceVerticalStartTimeIF=i-CalculationDelay;
%                 else
%                     VelocityVerticalIF=-IMU2AnkleLength*GyroZ(GyroDelayIndex)*sin(RollIF)*cos(PitchIF)+IMU2AnkleLength*GyroY(GyroDelayIndex)*sin(PitchIF);
%                     DistanceVerticalIF=cos(RollIF)*IMU2AnkleLength*cos(PitchIF);
%                     VelocityVerticalIFView(i)=VelocityVerticalIF;
%                     DistanceVerticalIFView(i)=DistanceVerticalIF;
%                     IntegrationVerticalResetTimeIF=i-CalculationDelay;
%                 end
%             end
%             if ZeroVertical==0
%                 if AccVerticalErrorSqMean<AccVerticalErrorSqMeanMin||ZeroVelocityDetected==1
%                     AccVerticalErrorSqMeanMin=AccVerticalErrorSqMean;
%                     VelocityVerticalIF=-IMU2AnkleLength*GyroZ(GyroDelayIndex)*sin(RollIF)*cos(PitchIF)+IMU2AnkleLength*GyroY(GyroDelayIndex)*sin(PitchIF);
%                     PreVerticalStanceIF=(i-CalculationDelay-HeelstrikeTime)/100.0;
%                     VelocityVerticalErrorIF=VelocityVerticalIF-VelocityVertical;
%                     DistanceVerticalIF=cos(RollIF)*IMU2AnkleLength*cos(PitchIF);
%                     VelocityVerticalIFView(i)=VelocityVerticalIF;
%                     DistanceVerticalIFView(i)=DistanceVerticalIF;
%                     StrideVLengthIF=DistanceVertical-DistanceVerticalIF;
%                     IntegrationVerticalResetTimeIF=i-CalculationDelay;
%                     MidStanceVerticalStartTimeIF=i-CalculationDelay;
%                 end
%             end
        elseif ZeroVelocityDetected==0
            if RollErrorDistanceMin==0||RollErrorDistance<RollErrorDistanceMin
                RollErrorDistanceMin=RollErrorDistance;
                RollBU=RollTrueFilter(ErrorDelayIndex);
                PitchBU=PitchTrueFilter(ErrorDelayIndex);
                YawBU=0;
                YawRange=Yaw;
                q0BU=cos(RollBU/2)*cos(PitchBU/2)*cos(YawBU/2)+sin(RollBU/2)*sin(PitchBU/2)*sin(YawBU/2);
                q1BU=sin(RollBU/2)*cos(PitchBU/2)*cos(YawBU/2)-cos(RollBU/2)*sin(PitchBU/2)*sin(YawBU/2);
                q2BU=cos(RollBU/2)*sin(PitchBU/2)*cos(YawBU/2)+sin(RollBU/2)*cos(PitchBU/2)*sin(YawBU/2);
                q3BU=-sin(RollBU/2)*sin(PitchBU/2)*cos(YawBU/2)+cos(RollBU/2)*cos(PitchBU/2)*sin(YawBU/2);
                VelocityFrontBU=-IMU2AnkleLength*GyroZ(GyroDelayIndex)*cos(RollBU);
                VelocityLateralBU=-IMU2AnkleLength*GyroY(GyroDelayIndex)*cos(PitchBU)-IMU2AnkleLength*GyroZ(GyroDelayIndex)*sin(RollBU)*sin(PitchBU);
                VelocityVerticalBU=-IMU2AnkleLength*GyroZ(GyroDelayIndex)*sin(RollBU)*cos(PitchBU)+IMU2AnkleLength*GyroY(GyroDelayIndex)*sin(PitchBU);
                DistanceFrontBU=-sin(RollBU)*IMU2AnkleLength;
                DistanceLateralBU=cos(RollBU)*IMU2AnkleLength*sin(PitchBU);
                DistanceVerticalBU=cos(RollBU)*IMU2AnkleLength*cos(PitchBU);
                VelocityFrontBUView(i)=VelocityFrontBU;
                VelocityVerticalBUView(i)=VelocityVerticalBU;
                VelocityLateralBUView(i)=VelocityLateralBU;
                DistanceFrontBUView(i)=DistanceFrontBU;
                DistanceVerticalBUView(i)=DistanceVerticalBU;
                DistanceLateralBUView(i)=DistanceLateralBU;
                RollBUView(i)=RollBU;
                PitchBUView(i)=PitchBU;
                YawBUView(i)=YawBU;
                PreStance=(i-CalculationDelay-HeelstrikeTime)/100.0;
                PreVerticalStance=(i-CalculationDelay-HeelstrikeTime)/100.0;
                Swing=(HeelstrikeTime-ToeOffTime)/100.0;

                VelocityFrontError=VelocityFrontBU*cos(Yaw)+VelocityLateralBU*sin(Yaw)-VelocityFront;
                VelocityLateralError=VelocityLateralBU*cos(Yaw)-VelocityFrontBU*sin(Yaw)-VelocityLateral;
                VelocityVerticalError=VelocityVerticalBU-VelocityVertical;
                RollRemove=RollBU-Roll;
                PitchRemove=PitchBU-Pitch;
                
                RollZ1=Roll;
                PitchZ1=Pitch;
                YawZ1=Yaw;
                StrideLength=DistanceFront;%-(-sin(RollBU)*IMU2AnkleLength*cos(Yaw)+cos(RollBU)*IMU2AnkleLength*sin(PitchBU)*sin(Yaw));%%%%%190409
                StrideVLength=DistanceVertical;%-DistanceVerticalBU;
                StrideLLength=DistanceLateral;%-(cos(RollBU)*IMU2AnkleLength*sin(PitchBU)*cos(Yaw)+sin(RollBU)*IMU2AnkleLength*sin(Yaw));
                FootHeight=FootHeightTem;
%                 FootHeight=FootHeight-StrideVLength*(FootHeightTime-IntegrationResetTime)/(i-CalculationDelay-IntegrationResetTime);
                IntegrationResetTime=i-CalculationDelay;
                MidStanceStartTime=i-CalculationDelay;
                ZeroStateEndRoll=RollBU;
                ZeroStateEndPitch=PitchBU;
                IntegrationVerticalResetTime=i-CalculationDelay;
                MidStanceVerticalStartTime=i-CalculationDelay;
            end
        end
    end
  end
  ZeroVelocityDetectedView(i)=ZeroVelocityDetected;
  ZeroVerticalView(i)=ZeroVertical;
end
%%
for i=1:length(StepCountView)
    iii=1;
    if i==1
        CycleStart=InitialIntegrationView;
        CycleVerticalStart=InitialIntegrationView;
    else
        CycleStart=IntegrationResetTimeView(i-1);
        CycleVerticalStart=IntegrationVerticalResetTimeView(i-1);
    end
    ShiftAngle(i)=atan(StrideLLengthView(i)/StrideLengthView(i));
    for ii=CycleStart+1:MidStanceStartTimeView(i)
        iii=iii+1;
        timeiii=CycleStart+iii-1+CalculationDelay;
        AnkleFront=DistanceFrontView(timeiii)-(-sin(RollView(timeiii))*IMU2AnkleLength*cos(YawView(timeiii))+cos(RollView(timeiii))*IMU2AnkleLength*sin(PitchView(timeiii))*sin(YawView(timeiii)));
        AnkleLateral=DistanceLateralView(timeiii)-(cos(RollView(timeiii))*IMU2AnkleLength*sin(PitchView(timeiii))*cos(YawView(timeiii))+sin(RollView(timeiii))*IMU2AnkleLength*sin(YawView(timeiii)));
        HorizontalDisplacement(iii)=cos(ShiftAngle(i))*AnkleFront+sin(ShiftAngle(i))*AnkleLateral;
        LateralDisplacement(iii)=-sin(ShiftAngle(i))*AnkleFront+cos(ShiftAngle(i))*AnkleLateral;     
    end
    maxLateralDisplacement(i)=max(LateralDisplacement(1:iii));
    minLateralDisplacement(i)=min(LateralDisplacement(1:iii));
    maxSpeed(i)=max(diff(HorizontalDisplacement(2:iii)))/dt;
    if i==1
        HorizontalDisplacementView(CycleStart)=0;
        LateralDisplacementView(CycleStart)=0;
    else
        for ii=MidStanceStartTimeView(i-1)+1:CycleStart
            HorizontalDisplacementView(ii)=HorizontalDisplacementView(ii-1);
            LateralDisplacementView(ii)=LateralDisplacementView(ii-1);
        end
    end
    DisplacementViewi=1;
    for ii=CycleStart+1:MidStanceStartTimeView(i)
        DisplacementViewi=DisplacementViewi+1;
        HorizontalDisplacementView(ii)=HorizontalDisplacementView(CycleStart)+HorizontalDisplacement(DisplacementViewi);
        LateralDisplacementView(ii)=LateralDisplacementView(CycleStart)+LateralDisplacement(DisplacementViewi);
    end

    iii=1;
    for ii=CycleVerticalStart+1:MidStanceVerticalStartTimeView(i)
        iii=iii+1;
        timeiii=CycleVerticalStart+iii-1+CalculationDelay;
        AnkleVertical=DistanceVerticalView(timeiii)-cos(RollView(timeiii))*IMU2AnkleLength*cos(PitchView(timeiii));
        VerticalDisplacement(iii)=AnkleVertical-StrideVLengthView(i)*(iii-1)/(MidStanceVerticalStartTimeView(i)-CycleVerticalStart);
    end
    [maxHeight(i),maxHeightTime]=max(VerticalDisplacement(1:iii));
    ToeOffHeight(i)=VerticalDisplacement(max(1,ToeOffTimeView(i)-CycleVerticalStart+1));
    if maxHeightTime+CycleVerticalStart-CycleStart>0
        maxDistanceAtMaxHeight(i)=HorizontalDisplacement(maxHeightTime+CycleVerticalStart-CycleStart);
    else
        maxDistanceAtMaxHeight(i)=0;
    end
    if i==1
        VerticalDisplacementView(CycleVerticalStart)=0;
    else
        for ii=MidStanceVerticalStartTimeView(i-1)+1:CycleVerticalStart
            VerticalDisplacementView(ii)=0;
        end
    end
    DisplacementViewi=1;
    for ii=CycleVerticalStart+1:MidStanceVerticalStartTimeView(i)
        DisplacementViewi=DisplacementViewi+1;
        VerticalDisplacementView(ii)=VerticalDisplacementView(CycleVerticalStart)+VerticalDisplacement(DisplacementViewi);
    end
    swingAngleX=sum(GyroXView(ToeOffTimeView(i)+1:HeelstrikeTimeView(i)));
    swingAngleY=sum(GyroYView(ToeOffTimeView(i)+1:HeelstrikeTimeView(i)));
    swingAngleZ=sum(GyroZView(ToeOffTimeView(i)+1:HeelstrikeTimeView(i)));
    AngleXZ(i)=atan2(swingAngleX,swingAngleZ);
    AngleYZ(i)=atan2(swingAngleY,swingAngleZ*cos(AngleXZ(i))+swingAngleX*sin(AngleXZ(i)));
    
    GyroZIntegration(1)=0;
    GyroYIntegration(1)=0;
    for ii=2:HeelstrikeTimeView(i)-ToeOffTimeView(i)+1
        GyroZShift(ii)=((GyroZView(ii+ToeOffTimeView(i)-1)*cos(AngleXZ(i))+GyroXView(ii+ToeOffTimeView(i)-1)*sin(AngleXZ(i)))*cos(AngleYZ(i))+GyroYView(ii+ToeOffTimeView(i)-1)*sin(AngleYZ(i)));
        GyroZIntegration(ii)=GyroZIntegration(ii-1)+GyroZShift(ii)*dt;
        GyroYIntegration(ii)=GyroYIntegration(ii-1)+(-(GyroZView(ii+ToeOffTimeView(i)-1)*cos(AngleXZ(i))+GyroXView(ii+ToeOffTimeView(i)-1)*sin(AngleXZ(i)))*sin(AngleYZ(i))+GyroYView(ii+ToeOffTimeView(i)-1)*cos(AngleYZ(i)))*dt;
    end
    RomShank(i)=max(GyroZIntegration(1:ii))-min(GyroZIntegration(1:ii));
    RomShankPitch(i)=max(GyroYIntegration(1:ii))-min(GyroYIntegration(1:ii));
    maxGyroZ(i)=max(GyroZShift(2:ii));
    GyroZIntegrationStance(1)=0;
    if i~=1
        for ii=2:ToeOffTimeView(i)-HeelstrikeTimeView(i-1)+1
            GyroZIntegrationStance(ii)=GyroZIntegrationStance(ii-1)+((GyroZView(ii+HeelstrikeTimeView(i-1)-1)*cos(AngleXZ(i))+GyroXView(ii+HeelstrikeTimeView(i-1)-1)*sin(AngleXZ(i)))*cos(AngleYZ(i))+GyroYView(ii+HeelstrikeTimeView(i-1)-1)*sin(AngleYZ(i)))*dt;
        end
    else
        for ii=2:ToeOffTimeView(i)-1+1
            GyroZIntegrationStance(ii)=GyroZIntegrationStance(ii-1)+((GyroZView(ii+1-1)*cos(AngleXZ(i))+GyroXView(ii+1-1)*sin(AngleXZ(i)))*cos(AngleYZ(i))+GyroYView(ii+1-1)*sin(AngleYZ(i)))*dt;
        end
    end
    if isempty(ii)==0
        RomShankStance(i)=max(GyroZIntegrationStance(1:ii))-min(GyroZIntegrationStance(1:ii));
    else
        RomShankStance(i)=0;
    end
    
    if i==1
        gaitCycle(i)=HeelstrikeTimeView(i)-InitialIntegrationView;
        HorizontalDisplacementCycle=HorizontalDisplacementView(InitialIntegrationView:HeelstrikeTimeView(i)-1);
        LateralDisplacementCycle=LateralDisplacementView(InitialIntegrationView:HeelstrikeTimeView(i)-1);
        VerticalDisplacementCycle=VerticalDisplacementView(InitialIntegrationView:HeelstrikeTimeView(i)-1);
        PitchCycle=PitchView(InitialIntegrationView+CalculationDelay:HeelstrikeTimeView(i)-1+CalculationDelay);
        RollCycle=RollView(InitialIntegrationView+CalculationDelay:HeelstrikeTimeView(i)-1+CalculationDelay);
        YawCycle=YawView(InitialIntegrationView+CalculationDelay:HeelstrikeTimeView(i)-1+CalculationDelay);
        GyroZCycle=((GyroZView(InitialIntegrationView:HeelstrikeTimeView(i)-1)*cos(AngleXZ(i))+GyroXView(InitialIntegrationView:HeelstrikeTimeView(i)-1)*sin(AngleXZ(i)))*cos(AngleYZ(i))+GyroYView(InitialIntegrationView:HeelstrikeTimeView(i)-1)*sin(AngleYZ(i)));
    else
        gaitCycle(i)=HeelstrikeTimeView(i)-HeelstrikeTimeView(i-1);
        ShiftAngle2Cycle=-ShiftAngle(i-1)+ShiftAngle(i)-YawRangeView(i-1);
        HorizontalDisplacementCycle(i,1:gaitCycle(i))=HorizontalDisplacementView(HeelstrikeTimeView(i-1):HeelstrikeTimeView(i)-1)-HorizontalDisplacementView(MidStanceStartTimeView(i-1));
        LateralDisplacementCycle(i,1:gaitCycle(i))=LateralDisplacementView(HeelstrikeTimeView(i-1):HeelstrikeTimeView(i)-1);
        VerticalDisplacementCycle(i,1:gaitCycle(i))=VerticalDisplacementView(HeelstrikeTimeView(i-1):HeelstrikeTimeView(i)-1);
        AnkleFront=HorizontalDisplacementCycle(i,1:MidStanceStartTimeView(i-1)+1-HeelstrikeTimeView(i-1));
        AnkleLateral=LateralDisplacementCycle(i,1:MidStanceStartTimeView(i-1)+1-HeelstrikeTimeView(i-1));
        HorizontalDisplacementCycle(i,1:MidStanceStartTimeView(i-1)+1-HeelstrikeTimeView(i-1))=cos(ShiftAngle2Cycle)*AnkleFront+sin(ShiftAngle2Cycle)*AnkleLateral;
        LateralDisplacementCycle(i,1:MidStanceStartTimeView(i-1)+1-HeelstrikeTimeView(i-1))=-sin(ShiftAngle2Cycle)*AnkleFront+cos(ShiftAngle2Cycle)*AnkleLateral;
        PitchCycle(i,1:gaitCycle(i))=PitchView(HeelstrikeTimeView(i-1)+CalculationDelay:HeelstrikeTimeView(i)-1+CalculationDelay);
        RollCycle(i,1:gaitCycle(i))=RollView(HeelstrikeTimeView(i-1)+CalculationDelay:HeelstrikeTimeView(i)-1+CalculationDelay);
        YawCycle(i,1:gaitCycle(i))=YawView(HeelstrikeTimeView(i-1)+CalculationDelay:HeelstrikeTimeView(i)-1+CalculationDelay);
        YawCycle(i,IntegrationResetTimeView(i-1)+1-HeelstrikeTimeView(i-1)+1:gaitCycle(i))=YawCycle(i,IntegrationResetTimeView(i-1)+1-HeelstrikeTimeView(i-1)+1:gaitCycle(i))+ShiftAngle(i);
        YawCycle(i,1:IntegrationResetTimeView(i-1)+1-HeelstrikeTimeView(i-1))=YawCycle(i,1:IntegrationResetTimeView(i-1)+1-HeelstrikeTimeView(i-1))+ShiftAngle(i-1)+ShiftAngle2Cycle;
        GyroZCycle(i,1:gaitCycle(i))=((GyroZView(HeelstrikeTimeView(i-1):HeelstrikeTimeView(i)-1)*cos(AngleXZ(i))+GyroXView(HeelstrikeTimeView(i-1):HeelstrikeTimeView(i)-1)*sin(AngleXZ(i)))*cos(AngleYZ(i))+GyroYView(HeelstrikeTimeView(i-1):HeelstrikeTimeView(i)-1)*sin(AngleYZ(i)));
    end
    maxLateralDisplacementCycle(i)=max(LateralDisplacementCycle(i,1:gaitCycle(i)));
    minLateralDisplacementCycle(i)=min(LateralDisplacementCycle(i,1:gaitCycle(i)));
    maxSpeedCycle(i)=max(diff(HorizontalDisplacementCycle(i,2:gaitCycle(i))))/dt;
    [maxHeightCycle(i),maxHeightTimeCycle]=max(VerticalDisplacementCycle(i,1:gaitCycle(i)));
    maxDistanceAtMaxHeightCycle(i)=HorizontalDisplacementCycle(i,maxHeightTimeCycle);
end
gaitCycle=[[HeelstrikeTimeView(1:end)];gaitCycle]';
% outdata=[StepCountView;StrideLengthView;StrideLLengthView;StrideVLengthView;YawRangeView;MotionRangeShankView;UpstairStraightenAngleView;ShankStanceAngleRangeView;ToeOffTimeView;HeelstrikeTimeView;WalkingStatusView;StrideLengthOutView;GaitCycleView;StancePeriodView;FootHeightView;RollErrorDistanceMaxView;DiffVerticalVelocityView;StrideLengthCom1View;StrideLengthCom2View;VelocityFrontErrorView;PreStanceView;StrideVLengthCom1View;StrideVLengthCom2View;StrideVLengthCom3View;MidStanceStartTimeView;IntegrationResetTimeView;MidStanceVerticalStartTimeView;IntegrationVerticalResetTimeView;ZeroStateEndRollView;ZeroStateEndPitchView;RomShank;maxSpeed;maxHeight;maxDistanceAtMaxHeight;maxLateralDisplacement;minLateralDisplacement;RomShankPitch;RomShankStance;maxGyroZ;ToeOffHeight]';
outdata=[StepCountView;StrideLengthView;StrideLLengthView;StrideVLengthView;YawRangeView;MotionRangeShankView;UpstairStraightenAngleView;ShankStanceAngleRangeView;ToeOffTimeView;HeelstrikeTimeView;WalkingStatusView;StrideLengthOutView;GaitCycleView;StancePeriodView;FootHeightView;RollErrorDistanceMaxView;DiffVerticalVelocityView;StrideLengthCom1View;StrideLengthCom2View;VelocityFrontErrorView;PreStanceView;StrideVLengthCom1View;StrideVLengthCom2View;StrideVLengthCom3View;MidStanceStartTimeView;IntegrationResetTimeView;MidStanceVerticalStartTimeView;IntegrationVerticalResetTimeView;ZeroStateEndRollView;ZeroStateEndPitchView;RomShank;maxSpeedCycle;maxHeightCycle;maxDistanceAtMaxHeightCycle;maxLateralDisplacementCycle;minLateralDisplacementCycle;RomShankPitch;RomShankStance;maxGyroZ;ToeOffHeight]';
outParameter=[StepCountView;StrideLengthOutView;GaitCycleView;StancePeriodView;RomShank;maxSpeedCycle;maxHeightCycle;maxDistanceAtMaxHeightCycle;maxLateralDisplacementCycle;minLateralDisplacementCycle]';
end