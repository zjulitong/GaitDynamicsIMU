function pFx = Fun_pFx(pAx,qF,dFx,dFy)
%FUN_PFX
%    PFX = FUN_PFX(PAX,QF,DFX,DFY)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    18-Nov-2020 01:52:23

pFx = pAx+dFx.*cos(qF)-dFy.*sin(qF);
