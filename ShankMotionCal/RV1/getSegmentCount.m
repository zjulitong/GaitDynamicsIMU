function [ segmentCount ] = getSegmentCount( Trial )
time=Trial(:,1);
segmentCount=1;
for i=1:length(time)-2
    if time(i+2)-time(i)>1000&&time(i+1)-time(i)>1000&&time(i+1)-time(i-1)>1000
        segmentCount=segmentCount+1;
    end
end
