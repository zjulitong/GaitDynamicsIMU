function xdot=trans_diff(x,t)

delta=t(2)-t(1);
len=length(x);

[fb_mkr, fa_mkr] = butter(2,2*6/150);

xdotm = (x(3:end,:)-x(1:end-2,:))/(delta*2);

xdotmf = filtfilt(fb_mkr,fa_mkr,xdotm);

xdot=interp1((2:len-1).',xdotmf,(1:len).','spline');



