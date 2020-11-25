function [] = plot_sd(t,xmin,xmax,facecolor)
%GAITCYCLE Summary of this function goes here
%   mn: mean value curve
%   sd: standard deviation curve

% mn=squeeze(mn);
% sd=squeeze(sd);

if size(t,1)~=size(xmin,1)
    xmin=reshape(xmin, [size(xmin,2),size(xmin,1)]);
    xmax=reshape(xmax, [size(xmax,2),size(xmax,1)]);
end

% if flag==1
    harea=area(t,[xmin,xmax-xmin],'HandleVisibility','off');
    set(harea(1),'FaceColor','none','EdgeColor','none');
    set(harea(2),'FaceColor',facecolor,'FaceAlpha',0.99,'EdgeColor','none');
%     
% % else
%     plot(t,mn,'Color',linecolor,...
%         'LineStyle',linestyle,'LineWidth',linewidth);
% end

end

        