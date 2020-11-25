function z=TLtrapz(t,x)

t=t*ones(1,size(x,2));
z=sum((x(1:end-1,:)+x(2:end,:))/2.*...
      (t(2:end,:)-t(1:end-1,:)));

% z=sum((x(1:end-1,:)+x(2:end,:))/2.*...
%       (t(2:end)-t(1:end-1)));
end
