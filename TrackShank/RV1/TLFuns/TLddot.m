function [ yddot ] = TLddot(y,q,qdot,qddot)

% Function to compute the second derivatives of an expression.
% the input are a vector with symbolic variables X, Xdot and Xddot.

if size(qdot,2)>1
    q=q.';
    qdot=qdot.';
    qddot=qddot.';
end

ydot=TLdot(y,q,qdot);
yddot=jacobian(ydot,[q;qdot])*[qdot;qddot];

end

