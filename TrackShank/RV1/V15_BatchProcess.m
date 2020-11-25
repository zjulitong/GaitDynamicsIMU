clear;clc;

set(0,'DefaultFigureVisible', 'off')
% set(0,'DefaultFigureVisible', 'on')
%%

ObjType= 10 ; % [0 simple,1=simple with periodical, 10=all settings]
% ResultFolder='BatchSimplest';
% ResultFolder='BatchSimple';
% ResultFolder='BatchAll'; % use Guess10 first Flgyswingmax=0.01*BM*g;
ResultFolder='BatchAll2'; % use Guess10 first  no swing constraint

Results_collect=[];
idx=0;

CTest=6;
Subj_pool=[1:12];


%%
for subi= Subj_pool
for typei= 1 :5
    for trii= 1:CTest
        GuessFlag=10;
        for ri=1:5
            [Result]=V15_Main(subi,typei,trii,ResultFolder,GuessFlag,ObjType);
            if ((Result.obj_qSegErr+Result.obj_pJoiErr)<1e-2) && (Result.exitflag>0)
                break;
            else
                GuessFlag=10;
            end
        end
        idx=idx+1;
        Results_collect{idx,1}=subi;
        Results_collect{idx,2}=typei;
        Results_collect{idx,3}=trii;
        Results_collect{idx,4}=Result.auxdata.ResultFileName;
        Results_collect{idx,5}=ri;
        Results_collect{idx,6}=Result.fval;
        Results_collect{idx,7}=Result.StepL_Err*100;
        
    end
end
end
Results_table=cell2table(Results_collect,'VariableNames',...
    {'subi','typei','trii','Result','ri','fval','StepLerr'});


% return;
%% save results

save(['ResAll_',ResultFolder,'_',char(datetime('now','format','yyyyMMdd_HHmmss'))],'Results_table');
set(0,'DefaultFigureVisible', 'on')

return;

