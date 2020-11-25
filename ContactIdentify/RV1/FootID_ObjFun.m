function [obj] = FootID_ObjFun(X,auxdata)


N_cts=auxdata.N_cts;
BM=auxdata.BM;
BH=auxdata.BH;
FootLen=auxdata.FootLen;

osHS=2;
osTO=2;

% dhex    = X(1);
% dhey    = X(2);
% dtox    = X(3);
% dtoy    = X(4);
R1       = X(1);
R5       = X(2);
yc       = X(3);
k       = X(4:(4+N_cts-1))*1e4;
b       = X((4+N_cts):(4+N_cts+N_cts-1));
v0      = X(4+N_cts+N_cts);

%%
dhex=auxdata.PX_HE+R1;
dhey=-auxdata.PY_AK+R1;
dtox=auxdata.PX_TO-R5;
dtoy=-auxdata.PY_AK+R5;

ctsx =linspace(dhex, dtox,N_cts);
ctsy =linspace(dhey, dtoy,N_cts);
ctsR      = ctsy-(ctsy(1)-R1);
ctsk      = k.';
ctsc      = b.';
ctsud     = ones(1,N_cts)*0.8;
ctsV      = ones(1,N_cts)*v0;


pRAx=auxdata.pAJ(:,1);
pRAy=auxdata.pAJ(:,2)+yc;
pLAx=auxdata.pAJ(:,3);
pLAy=auxdata.pAJ(:,4)+yc;
qRF=auxdata.qFO(:,1);
qLF=auxdata.qFO(:,2);

pRAdotx=auxdata.pAJdot(:,1);
pRAdoty=auxdata.pAJdot(:,2);
pLAdotx=auxdata.pAJdot(:,3);
pLAdoty=auxdata.pAJdot(:,4);
qRFdot=auxdata.qFOdot(:,1);
qLFdot=auxdata.qFOdot(:,2);

xrcts=Fun_pFx(pRAx,qRF,ctsx,ctsy);
yrcts=Fun_pFy(pRAy,qRF,ctsx,ctsy);
xrctsdot=Fun_pFdotx(pRAy,qRF,pRAdotx,qRFdot,ctsx,ctsy);
yrctsdot=Fun_pFdoty(pRAy,qRF,pRAdoty,qRFdot,ctsx,ctsy);
xlcts=Fun_pFx(pLAx,qLF,ctsx,ctsy);
ylcts=Fun_pFy(pLAy,qLF,ctsx,ctsy);
xlctsdot=Fun_pFdotx(pLAy,qLF,pLAdotx,qLFdot,ctsx,ctsy);
ylctsdot=Fun_pFdoty(pLAy,qLF,pLAdoty,qLFdot,ctsx,ctsy);


[cts_FRx]=Fun_Contact_Sim_Fx(ctsR,ctsk,ctsc,ctsud,ctsV,yrcts,yrctsdot,xrctsdot);
[cts_FRy]=Fun_Contact_Sim_Fy(ctsR,ctsk,ctsc,           yrcts,yrctsdot);
[cts_FLx]=Fun_Contact_Sim_Fx(ctsR,ctsk,ctsc,ctsud,ctsV,ylcts,ylctsdot,xlctsdot);
[cts_FLy]=Fun_Contact_Sim_Fy(ctsR,ctsk,ctsc,           ylcts,ylctsdot);
Frgx_s=sum(cts_FRx,2);
Frgy_s=sum(cts_FRy,2);
Flgx_s=sum(cts_FLx,2);
Flgy_s=sum(cts_FLy,2);

COPRx_s=sum(cts_FRy.*xrcts,2)./Frgy_s;
COPLx_s=sum(cts_FLy.*xlcts,2)./Flgy_s;

%%
FrameCount=auxdata.FrameCount;
FrameCountDS=auxdata.FrameCountDS;

GRFRx=auxdata.GRF(:,1);
GRFRy=auxdata.GRF(:,2);
GRFLx=auxdata.GRF(:,3);
GRFLy=auxdata.GRF(:,4);

COPRx=auxdata.COP(:,1);
COPLx=auxdata.COP(:,3);

t_mkr=auxdata.t_mkr;

errCOPRx=COPRx_s-COPRx;
errCOPLx=COPLx_s(1:FrameCountDS)-COPLx(1:FrameCountDS);

errGRFRx=GRFRx-Frgx_s;
errGRFRy=GRFRy-Frgy_s;
errGRFLx=GRFLx-Flgx_s;
errGRFLy=GRFLy-Flgy_s;

errsum=[sum(errCOPRx(osHS:end).^2),sum(errCOPLx(1:(end-osTO)).^2),...
        sum(errGRFRx.^2),sum(errGRFRy.^2),...
        sum(errGRFLx.^2),sum(errGRFLy.^2)];

% fac=[1e8;1e8; 10;1; 10;1 ];
% fac=[1e6;1e6; 10;1; 10;1 ];
fac=[1e6;1e6; 1;1; 1;1 ];
% fac=[1e7;1e7; 1;1; 1;1 ];
% fac=[1e8;1e8; 1;1; 1;1 ];
obj=errsum*fac;

%

%%

if auxdata.plotflag==1
    %%
    objw=errsum.*fac.'
    
%     figure;
%     set(gcf,'Position',[100 100 900 400])
%     subplot(2,5,1);hold on;title('yrcts');
%     plot(t_mkr,yrcts);
%     xlim([t_mkr(1),t_mkr(end)]);
%     subplot(2,5,2);hold on;title('yrctsdot');
%     plot(t_mkr,yrctsdot);
%     xlim([t_mkr(1),t_mkr(end)]);
%     subplot(2,5,3);hold on;title('xrctsdot');
%     plot(t_mkr,xrctsdot);
%     xlim([t_mkr(1),t_mkr(end)]);
%     subplot(2,5,4);hold on;title('cts-FRx');
%     plot(t_mkr,cts_FRx);
%     plot(t_mkr,Frgx_s,'r');
%     xlim([t_mkr(1),t_mkr(end)]);
%     subplot(2,5,5);hold on;title('cts-FRy');
%     plot(t_mkr,cts_FRy);
%     plot(t_mkr,Frgy_s,'r');
%     xlim([t_mkr(1),t_mkr(end)]);
%     
%     subplot(2,5,6);hold on;title('ylcts');
%     plot(t_mkr,ylcts);
%     xlim([t_mkr(1),t_mkr(end)]);
%     subplot(2,5,7);hold on;title('ylctsdot');
%     plot(t_mkr,ylctsdot);
%     xlim([t_mkr(1),t_mkr(end)]);
%     subplot(2,5,8);hold on;title('xlctsdot');
%     plot(t_mkr,xlctsdot);
%     xlim([t_mkr(1),t_mkr(end)]);
%     subplot(2,5,9);hold on;title('cts-FLx');
%     plot(t_mkr,cts_FLx);
%     plot(t_mkr,Flgx_s,'r');
%     xlim([t_mkr(1),t_mkr(end)]);
%     subplot(2,5,10);hold on;title('cts-FLy');
%     plot(t_mkr,cts_FLy,'DisplayName','cts');
%     plot(t_mkr,Flgy_s,'r','DisplayName','Total');
%     xlim([t_mkr(1),t_mkr(end)]);
%     lgd=legend('show');
%     lgd.FontSize=7;
%     
%     figure;hold on;
%     set(gcf,'Position',[100 100 400 500],'defaultLineLineWidth',1.5)
%     subplot(3,2,1);hold on;title('COPRx');
%     line([t_mkr(1),t_mkr(end)],[0 0],'Color',[0.3 0.3 0.3]);
%     plot(t_mkr,COPRx_s-1.4);
%     plot(t_mkr,COPRx-1.4,'k');
%     plot(t_mkr,errCOPRx,'r');
%     xlim([t_mkr(1),t_mkr(end)]);
%     line(t_mkr(osHS)*[1 1],ylim(),'Color',[1 1 1]*0.6);
%     
%     subplot(3,2,2);hold on;title('COPLx');
%     line([t_mkr(1),t_mkr(end)],[0 0],'Color',[0.3 0.3 0.3]);
%     plot(t_mkr(1:FrameCountDS),COPLx_s(1:FrameCountDS)-1);
%     plot(t_mkr(1:FrameCountDS),COPLx(1:FrameCountDS)-1,'k');
%     plot(t_mkr(1:FrameCountDS),errCOPLx,'r');
%     xlim([t_mkr(1),t_mkr(end)]);
%     line(t_mkr(FrameCountDS-osTO)*[1 1],ylim(),'Color',[1 1 1]*0.6);
%     
%     subplot(3,2,3);hold on;title('Frgx');
%     line([t_mkr(1),t_mkr(end)],[0 0],'Color',[0.3 0.3 0.3]);
%     plot(t_mkr,Frgx_s);
%     plot(t_mkr,GRFRx,'k');
%     plot(t_mkr,errGRFRx,'r');
%     xlim([t_mkr(1),t_mkr(end)]);
%     
%     
%     subplot(3,2,4);hold on;title('Flgx');
%     line([t_mkr(1),t_mkr(end)],[0 0],'Color',[0.3 0.3 0.3]);
%     plot(t_mkr,Flgx_s);
%     plot(t_mkr,GRFLx,'k');
%     plot(t_mkr,errGRFLx,'r');
%     xlim([t_mkr(1),t_mkr(end)]);
%     
%     subplot(3,2,5);hold on;title('Frgy');
%     line([t_mkr(1),t_mkr(end)],[0 0],'Color',[0.3 0.3 0.3]);
%     plot(t_mkr,Frgy_s);
%     plot(t_mkr,GRFRy,'k');
%     plot(t_mkr,errGRFRy,'r');
%     xlim([t_mkr(1),t_mkr(end)]);
%     
%     subplot(3,2,6);hold on;title('Flgx');
%     line([t_mkr(1),t_mkr(end)],[0 0],'Color',[0.3 0.3 0.3]);
%     plot(t_mkr,Flgy_s);
%     plot(t_mkr,GRFLy,'k');
%     plot(t_mkr,errGRFLy,'r');
%     xlim([t_mkr(1),t_mkr(end)]);
    
    
%     figure;hold on;
%     subplot(1,2,1);hold on;title('Right foot');
%     axis equal
%     plot(pRAx,pRAy,'o-','MarkerSize',3);
%     plot(xrcts,yrcts,'ro','MarkerSize',3);
%     for i=[1,FrameCount]
%         line([pRAx(i),xrcts(i,1)],[pRAy(i),yrcts(i,1)],'Color','b')
%         line([pRAx(i),xrcts(i,end)],[pRAy(i),yrcts(i,end)],'Color','b')
%         line([xrcts(i,1),xrcts(i,end)],[yrcts(i,1),yrcts(i,end)],'Color','b')
% 
%                 for f1cti=1:N_cts
%                     rectangle('Position',[xrcts(i,f1cti)-ctsR(f1cti),...
%                         yrcts(i,f1cti)-ctsR(f1cti),2*ctsR(f1cti),2*ctsR(f1cti)],'Curvature',[1,1],'FaceColor',[0.1 0.1 0.1 0.3]);
%                 end
%      
%     end
%     
%     subplot(1,2,2);hold on;title('Left foot');
%     axis equal
%     plot(pLAx,pLAy,'o-','MarkerSize',3);
%     plot(xlcts,ylcts,'ro','MarkerSize',3);
%     
%     for i=[1,FrameCount]
%         line([pLAx(i),xlcts(i,1)],[pLAy(i),ylcts(i,1)],'Color','b')
%         line([pLAx(i),xlcts(i,end)],[pLAy(i),ylcts(i,end)],'Color','b')
%         line([xlcts(i,1),xlcts(i,end)],[ylcts(i,1),ylcts(i,end)],'Color','b')
%         
%                 for f1cti=1:N_cts
%                     rectangle('Position',[xlcts(i,f1cti)-ctsR(f1cti),...
%                         ylcts(i,f1cti)-ctsR(f1cti),2*ctsR(f1cti),2*ctsR(f1cti)],'Curvature',[1,1],'FaceColor',[0.1 0.1 0.1 0.3]);
%                 end
%         
%     end
    %% dissertation figure
    
%     
% pAx=[pRAx(4:end);pLAx(1:FrameCount)-pLAx(1)+pRAx(end)];
% pAy=[pRAy(4:end);pLAy(1:FrameCount)];
% xcts=[xrcts(4:end,:);xlcts(1:FrameCount,:)-pLAx(1)+pRAx(end)];
% ycts=[yrcts(4:end,:);ylcts(1:FrameCount,:)];
% t=[t_mkr(4:end);t_mkr(1:FrameCount)+t_mkr(end)];
% 
% GRFx=[GRFRx(4:end,:);GRFLx(1:FrameCount,:)];
% GRFy=[GRFRy(4:end,:);GRFLy(1:FrameCount,:)];
% Fgx_s=[Frgx_s(4:end,:);Flgx_s(1:FrameCount,:)];
% Fgy_s=[Frgy_s(4:end,:);Flgy_s(1:FrameCount,:)];
% 
% 
% t2=[t_mkr(osHS:end);t_mkr(1:FrameCountDS-osTO)+t_mkr(end)];
% COPx_s=[COPRx_s(osHS:end,:);COPLx_s(1:FrameCountDS-osTO,:)-pLAx(1)+pRAx(end)]-pRAx(1);
% COPx=[COPRx(osHS:end,:);COPLx(1:FrameCountDS-osTO,:)-pLAx(1)+pRAx(end)]-pRAx(1);
% 
% 
% 
% 
% 
% C1= [110 110 110]/3/255;
% C2= [255 48 48]/255;
% 
% figure;hold on;
% set(gcf,'Position',[100 100 600 200],'defaultLineLineWidth',1.5);
% subplot(1,3,1);hold on;%title('(a)')
% plot(t2,COPx,'Color',C1);
% plot(t2,COPx_s,'Color',C2);
% % plot(t2,COPx_s-COPx,'r');
% xlim([t(1) t(end)])
% xlabel('Time (s)');
% ylabel('pgx (m)');
% 
% subplot(1,3,2);hold on;%title('(b)')
% plot(t,GRFx ,'Color',C1);
% plot(t,Fgx_s,'Color',C2);
% % plot(t,Fgx_s-GRFx,'r');
% xlim([t(1) t(end)])
% xlabel('Time (s)');
% ylabel('Fgx (N)');
% 
% subplot(1,3,3);hold on;%title('(c)')
% plot(t,GRFy ,'Color',C1);
% plot(t,Fgy_s,'Color',C2);
% % plot(t,Fgy_s-GRFy,'r');
% xlim([t(1) t(end)])
% xlabel('Time (s)');
% ylabel('Fgy (N)');
% 
% legend({'Exp','Model'});% ,'Err'
% 
% 
% figure;hold on;title('(D)')
% set(gcf,'Position',[100 100 900 300]);
% line([1.2 3],[0 0],'Color','k');
% plot(pAx,pAy,'o-','MarkerSize',3);
% plot(xcts,ycts,'ro','MarkerSize',1);
% for i=[1,length(t)]
%     line([pAx(i),xcts(i,1)],[pAy(i),ycts(i,1)],'Color','b')
%     line([pAx(i),xcts(i,end)],[pAy(i),ycts(i,end)],'Color','b')
%     line([xcts(i,1),xcts(i,end)],[ycts(i,1),ycts(i,end)],'Color','b')
%     
%     for f1cti=1:N_cts
%         rectangle('Position',[xcts(i,f1cti)-ctsR(f1cti),...
%             ycts(i,f1cti)-ctsR(f1cti),2*ctsR(f1cti),2*ctsR(f1cti)],'Curvature',[1,1],'FaceColor',[0.1 0.1 0.1 0.3]);
%     end
%     
% end
% axis equal
% % ylim([-0.1 0.5])
% xlabel('x (m)');
% ylabel('y (m)');
% 
% 
% tab=[ctsx;ctsy;ctsR;ctsk;ctsc;ctsud;ctsV;];

%% paper figure
    
    
pAx=[pRAx(4:end);pLAx(1:FrameCount)-pLAx(1)+pRAx(end)];
pAy=[pRAy(4:end);pLAy(1:FrameCount)];
xcts=[xrcts(4:end,:);xlcts(1:FrameCount,:)-pLAx(1)+pRAx(end)];
ycts=[yrcts(4:end,:);ylcts(1:FrameCount,:)];
t=[t_mkr(4:end);t_mkr(1:FrameCount)+t_mkr(end)];

GRFx=[GRFRx(4:end,:);GRFLx(1:FrameCount,:)];
GRFy=[GRFRy(4:end,:);GRFLy(1:FrameCount,:)];
Fgx_s=[Frgx_s(4:end,:);Flgx_s(1:FrameCount,:)];
Fgy_s=[Frgy_s(4:end,:);Flgy_s(1:FrameCount,:)];


t2=[t_mkr(osHS:end);t_mkr(1:FrameCountDS-osTO)+t_mkr(end)];
COPx_s=[COPRx_s(osHS:end,:);COPLx_s(1:FrameCountDS-osTO,:)-pLAx(1)+pRAx(end)]-pRAx(1);
COPx=[COPRx(osHS:end,:);COPLx(1:FrameCountDS-osTO,:)-pLAx(1)+pRAx(end)]-pRAx(1);



% t_cycle=t_mkr/t_mkr(end)*50;
t_cycle=t_mkr;

C1= [110 110 110]/3/255;
C2= [255 48 48]/255;
copx_offset=min([COPRx(osHS:end,:);COPLx(1:FrameCountDS-osTO,:)]);
figure;hold on;
% set(gcf,'Position',[100 100 600 200],'defaultLineLineWidth',1.5);
set(gcf,'Position',[100 100 700 250],'defaultLineLineWidth',2);
subplot(1,3,1,'FontSize',11);hold on;%title('(a)')
plot(t_cycle(osHS:end),COPRx(osHS:end,:)-copx_offset,'Color',C1);
plot(t_cycle(osHS:end),COPRx_s(osHS:end,:)-copx_offset,'Color',C2);
plot(t_cycle(1:FrameCountDS-osTO),COPLx(1:FrameCountDS-osTO,:)-copx_offset,'-.','Color',C1);
plot(t_cycle(1:FrameCountDS-osTO),COPLx_s(1:FrameCountDS-osTO,:)-copx_offset,'-.','Color',C2);
% plot(t2,COPx_s-COPx,'r');
xlim([t_cycle(1) t_cycle(end)])
ylim([-0.2,max(ylim())]);
% xlabel('Gait cycle (%)');
xlabel('Time (s)');
ylabel('Fore-aft COP (m)');


subplot(1,3,2,'FontSize',11);hold on;%title('(b)')
plot(t_cycle(1:end),GRFRx(1:end,:)/BM/9.8 ,'Color',C1); % /BM
plot(t_cycle(1:end),Frgx_s(1:end,:)/BM/9.8,'Color',C2);
plot(t_cycle(1:FrameCount),GRFLx(1:FrameCount,:)/BM/9.8,'-.' ,'Color',C1);
plot(t_cycle(1:FrameCount),Flgx_s(1:FrameCount,:)/BM/9.8,'-.','Color',C2);
% plot(t,Fgx_s-GRFx,'r');
xlim([t_cycle(1) t_cycle(end)])
% ylim([-200 300])
% xlabel('Gait cycle (%)');
xlabel('Time (s)');
ylabel('Fore-aft GRF (/BW)');

subplot(1,3,3,'FontSize',11);hold on;%title('(c)')
plot(t_cycle(1:end),GRFRy(1:end,:)/BM/9.8 ,'Color',C1);
plot(t_cycle(1:end),Frgy_s(1:end,:)/BM/9.8,'Color',C2);
plot(t_cycle(1:FrameCount),GRFLy(1:FrameCount,:)/BM/9.8,'-.' ,'Color',C1);
plot(t_cycle(1:FrameCount),Flgy_s(1:FrameCount,:)/BM/9.8,'-.','Color',C2);
% plot(t,Fgy_s-GRFy,'r');
xlim([t_cycle(1) t_cycle(end)])
ylim([-0.1 max(ylim())])
% xlabel('Gait cycle (%)');
xlabel('Time (s)');
ylabel('Vertical GRF (/BW)');

h=legend({'MCS-R','Model-R','MCS-L','Model-L'},'Box','off','FontSize',9);% ,'Err'


% figure;hold on;title('(D)')
% set(gcf,'Position',[100 100 900 300]);
% line([1.2 3],[0 0],'Color','k');
% plot(pAx,pAy,'o-','MarkerSize',3);
% plot(xcts,ycts,'ro','MarkerSize',1);
% for i=[1,length(t)]
%     line([pAx(i),xcts(i,1)],[pAy(i),ycts(i,1)],'Color','b')
%     line([pAx(i),xcts(i,end)],[pAy(i),ycts(i,end)],'Color','b')
%     line([xcts(i,1),xcts(i,end)],[ycts(i,1),ycts(i,end)],'Color','b')
%     
%     for f1cti=1:N_cts
%         rectangle('Position',[xcts(i,f1cti)-ctsR(f1cti),...
%             ycts(i,f1cti)-ctsR(f1cti),2*ctsR(f1cti),2*ctsR(f1cti)],'Curvature',[1,1],'FaceColor',[0.1 0.1 0.1 0.3]);
%     end
%     
% end
% axis equal
% % ylim([-0.1 0.5])
% xlabel('x (m)');
% ylabel('y (m)');
% 
% 
tab=[ctsx;ctsy;ctsR;ctsk;ctsc;ctsud;ctsV;];

%% controllable snapshot----
% figure;hold on;
% set(gcf,'Position',[100 100 1100 400])
% title('gait cycle')
% sld=uicontrol('Style','slider','Min',1,'Max',FrameCount,'Value',1,...
%     'SliderStep',[1/(FrameCount-1) 1/(FrameCount-1)],'Units','normalized','Position',[0.4 0.1 0.25 0.05],...
%     'Callback',@posti);
% sld.CreateFcn=@posti;

end


    function posti(source,event)
        gridi=round(source.Value);
        subplot(1,2,1);hold on;title(['n=',num2str(gridi),' t=',num2str(t_mkr(gridi),'%.03f'),'s'])
        cla;
        %         gridi=(source.Value);
        plot(pRAx,pRAy,'o-','MarkerSize',3);
        plot(xrcts,yrcts,'ro','MarkerSize',3);
%         line([1.3 1.8],[0 0],'Color','k');
        line([min(pRAx)-0.1 max(pRAx)+0.3],[0 0],'Color','k');
        line([pRAx((gridi)),xrcts((gridi),1)],[pRAy(gridi),yrcts(gridi,1)],'Color','b')
        line([pRAx(gridi),xrcts(gridi,end)],[pRAy(gridi),yrcts(gridi,end)],'Color','b')
        line([xrcts(gridi,1),xrcts(gridi,end)],[yrcts(gridi,1),yrcts(gridi,end)],'Color','b')
        
        for f1cti=1:N_cts
            rectangle('Position',[xrcts(gridi,f1cti)-ctsR(f1cti),...
                yrcts(gridi,f1cti)-ctsR(f1cti),2*ctsR(f1cti),2*ctsR(f1cti)],'Curvature',[1,1],'FaceColor',[0.1 0.1 0.1 0.3]);
        end
        
        axis equal;
%         xlim([1.3 1.8]);
%         ylim([-0.15 0.15]);
        
        
        %         grid minor
        
        subplot(1,2,2);hold on;
        cla;
        plot(pLAx,pLAy,'o-','MarkerSize',3);
        plot(xlcts,ylcts,'ro','MarkerSize',3);
%         line([0.6 2.4],[0 0],'Color','k');
        line([min(pLAx)-0.1 max(pLAx)+0.3],[0 0],'Color','k');
        line([pLAx((gridi)),xlcts((gridi),1)],[pLAy(gridi),ylcts(gridi,1)],'Color','b')
        line([pLAx(gridi),xlcts(gridi,end)],[pLAy(gridi),ylcts(gridi,end)],'Color','b')
        line([xlcts(gridi,1),xlcts(gridi,end)],[ylcts(gridi,1),ylcts(gridi,end)],'Color','b')
        
        for f1cti=1:N_cts
            rectangle('Position',[xlcts(gridi,f1cti)-ctsR(f1cti),...
                ylcts(gridi,f1cti)-ctsR(f1cti),2*ctsR(f1cti),2*ctsR(f1cti)],'Curvature',[1,1],'FaceColor',[0.1 0.1 0.1 0.3]);
        end
        
        axis equal;
%         xlim([0.6 2.4]);
%         ylim([-0.15 0.35]);
        
    end

end
