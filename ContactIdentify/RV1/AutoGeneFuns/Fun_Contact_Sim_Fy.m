function Fy = Fun_Contact_Sim_Fy(R,k,c,y,ydot)
%FUN_CONTACT_SIM_FY
%    FY = FUN_CONTACT_SIM_FY(R,K,C,Y,YDOT)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    18-Nov-2020 01:52:20

t2 = R-y;
t3 = k.^(2.0./3.0);
Fy = t3.*(t2.^2+1.0e-5).^(3.0./4.0).*(tanh(ydot.*5.0e1-(1.0e2./3.0)./c).*(1.0./2.0)-5.000000000000001e-1).*(c.*ydot.*(3.0./2.0)-1.0).*(tanh(R.*3.0e2-y.*3.0e2).*(1.0./2.0)+5.000000000000001e-1).*sqrt(R.*t3.*(1.0./2.0)).*(2.0./3.0);