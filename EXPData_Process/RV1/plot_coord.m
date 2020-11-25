function [varargout] = plot_coord(T,l,sty)

if nargin==1
  l = 1;
  sty='-';
elseif nargin==2
  sty='-';
end

start = [T(1:3,4), T(1:3,4), T(1:3,4)];
stop = start + l*[T(1:3,1), T(1:3,2), T(1:3,3)];
% stop = start + l*[T(1,1:3).', T(2,1:3).', T(3,1:3).'];

X = [start(1,:);stop(1,:)];
Y = [start(2,:);stop(2,:)];
Z = [start(3,:);stop(3,:)];

if nargout <= 1
%   h = line(X,Y,Z,'LineWidth',2);
line(X(:,1),Y(:,1),Z(:,1),'LineWidth',2,'Color','c','LineStyle',sty);  
line(X(:,2),Y(:,2),Z(:,2),'LineWidth',2,'Color','g','LineStyle',sty);  
line(X(:,3),Y(:,3),Z(:,3),'LineWidth',2,'Color','r','LineStyle',sty);  
varargout{1} = 1;
  
else
  varargout{1} = X';
  varargout{2} = Y';
  varargout{3} = Z';
end
