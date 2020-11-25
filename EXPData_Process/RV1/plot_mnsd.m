function [] = plot_mnsd(t,mn,sd,flag,facecolor,linecolor,linestyle,linewidth)
%GAITCYCLE Summary of this function goes here
%   mn: mean value curve
%   sd: standard deviation curve

mn=squeeze(mn);
sd=squeeze(sd);

if size(t,1)~=size(mn,1)
    mn=reshape(mn, [size(mn,2),size(mn,1)]);
    sd=reshape(sd, [size(sd,2),size(sd,1)]);
end

if flag==1
    harea=area(t,[mn-sd,2*sd],'HandleVisibility','off');
    set(harea(1),'FaceColor','none','EdgeColor','none');
    set(harea(2),'FaceColor',facecolor,'FaceAlpha',0.9,'EdgeColor','none');
    
else
    plot(t,mn,'Color',linecolor,...
        'LineStyle',linestyle,'LineWidth',linewidth);
end

end

        