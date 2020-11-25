function xdot=TLdiff(x,t,peri_flag)

%-------------------------- Euler
t=t*ones(1,size(x,2));
xdot=(x(2:end,:)-x(1:end-1,:))./(t(2:end,:)-t(1:end-1,:));

if peri_flag==1 % if assume x is periodical and output xdot that is also periodical
    xdot=[xdot;xdot(1,:)]; % xdot is with the same length as x
end
% else % xdot is short than x in length by one, same to the function "diff"

end