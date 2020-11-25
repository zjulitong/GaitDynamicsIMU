function [ TrialOut ] = getDataSegment( Trial,nosegment,findbreak )
time=Trial(:,1);
segmentCount=0;
segmentStart=1;
segmentEnd=0;
for i=1:length(time)-2
    if time(i+2)-time(i)>1000&&time(i+1)-time(i)>1000&&time(i+1)-time(i-1)>1000
        segmentCount=segmentCount+1;
        if segmentCount==nosegment
            segmentEnd=i;
            break;
        else
            segmentStart=i+1;
        end
    end
end
if segmentEnd==0&&segmentCount==nosegment-1
    segmentEnd=length(time);
end
Trial=Trial(segmentStart:segmentEnd,:);

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
TrialOut=Trial;
end

