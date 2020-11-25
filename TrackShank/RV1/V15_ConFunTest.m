function y=V15_ConFunTest(X,auxdata,flag)

[c,ceq,~,~] = V15_ConFun(X,auxdata);
switch flag
    case 1
        y=c;
    case 2
        y=ceq;
end

end



