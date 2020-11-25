function [Xopt,fval,exitflag,output]=V15_Optimize_IPOPT(X0,lb,ub,auxdata)


global test_count test_divide plot_flag; %#ok<NUSED>
global fixstep_flag fixLstep; %#ok<NUSED>

%% using IPOPT

ip_opts=ipoptset('dual_inf_tol',1e-0,'constr_viol_tol',1e-6,'compl_inf_tol',1e-4,...
   'derivative_test_print_all','no');

switch auxdata.ChkGradFlag
    case true
        deCheck='on';
    case false
        deCheck='off';
end

opts=optiset('solver','ipopt','display','iter',...
    'tolafun',1,'tolrfun',1,...
    'maxtime',2e+3,'derivCheck',deCheck,'maxiter',1000,'maxfeval',1e+6,...
    'solverOpts',ip_opts);

if auxdata.logiter
    iter_table=[];
    opts.iterfun=@myCallback;
end

[myc,myceq,mycJac,myceqJac] = V15_ConFun(X0,auxdata);

nlrhs=zeros(length(myc)+length(myceq),1);
nle=[-1*ones(length(myc),1);zeros(length(myceq),1)];
jacstr=@() (auxdata.Grad_CONS.nlcJX_strc);

%% Run optimization
if auxdata.ObjGradFlag==true && auxdata.ConGradFlag==true
    Problem=opti('fun',@(x) objfun(x,auxdata),'x0',X0,'bounds',lb,ub,...
        'options',opts,...
        'A',auxdata.lcneqA,'b',auxdata.lcneqb,...
        'Aeq',auxdata.lceqA,'beq',auxdata.lceqb,...
        'nlmix',@(x) confun(x,auxdata),nlrhs,nle,...
        'f',@(x) gradfun(x,auxdata),...
        'nljac',@(x) jacfun(x,auxdata),...
        'nljacstr',jacstr);

elseif  auxdata.ObjGradFlag==false && auxdata.ConGradFlag==false
    Problem=opti('fun',@(x) objfun(x,auxdata),'x0',X0,'bounds',lb,ub,...
        'options',opts,...
        'A',auxdata.lcneqA,'b',auxdata.lcneqb,...
        'Aeq',auxdata.lceqA,'beq',auxdata.lceqb,...
        'nlmix',@(x) confun(x,auxdata),nlrhs,nle);
end

[Xopt,fval,exitflag,output] = solve(Problem);

%% Save results 
plot_flag=10; %#ok<*NASGU>
[Result,myGrad] = V15_ObjFun(Xopt,auxdata); %#ok<*ASGLU>
[myc,myceq,mycJac,myceqJac] = V15_ConFun(Xopt,auxdata);
plot_flag=0;

Result.exitflag=exitflag;
Result.fval=fval;
Result.output=output;

t_stop=datetime('now','format','yyyyMMdd_HHmmss');

Result.auxdata.t_stop= t_stop;
Result.auxdata.t_use= Result.auxdata.t_stop-auxdata.t_start;
disp(['Time used:',num2str(seconds(Result.auxdata.t_use)),' sec']);

save(auxdata.ResultFileName,'Result');

%% nested functions
    function y = objfun(x,auxdata)
            [y,~] = V15_ObjFun(x,auxdata);
    end
    function [grad] = gradfun(x,auxdata)
            [~,grad] = V15_ObjFun(x,auxdata);
        
    end
    function [cceq] = confun(x,auxdata)
            [c,ceq,~,~] = V15_ConFun(x,auxdata);
        cceq = [c;ceq];
    end

    function [cceqJac] = jacfun(x,auxdata)
            [~,~,cJac,ceqJac] = V15_ConFun(x,auxdata);
        cceqJac = [cJac;ceqJac]; % mycJac is the combination of cJac and ceqJac
    end

    function stop = myCallback(iter,fval,x) 
        stop = false;
        iter_table{iter+1,1}=x;
        iter_table{iter+1,2}=fval;
    end
end


