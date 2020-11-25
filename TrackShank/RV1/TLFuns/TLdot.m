function [ yddot ] = TLdot(y,q,qdot)

% Function to compute the second derivatives of an expression.
% the input are a vector with symbolic variables X, Xdot and Xddot.

if size(qdot,2)>1
    qdot=qdot.';
end

yddot=jacobian(y,q)*qdot;

end

