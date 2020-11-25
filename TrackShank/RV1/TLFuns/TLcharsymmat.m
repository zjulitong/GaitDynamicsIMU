function [mat_charOld,mat_charNew]=TLcharsymmat(mat)

% Function: change the sym mat into char for subs
% mat should be like '[q1, q2,...]' or [q1_1,q1_2,...;q2_1,q2_2,...;...]






num_row=size(mat,1);
num_col=size(mat,2);

mat_charOld=cell(numel(mat),1);
mat_charNew=cell(numel(mat),1);
mat1_1_name=char(mat(1,1));

if num_row>1&&num_col>1
    mat_name=mat1_1_name(1:end-3);
    for j=1:num_col
        for i=1:num_row
            
            idx=sub2ind(size(mat),i ,j);
            mat_charOld{idx}=[mat_name,'',num2str(i),'_',num2str(j)];
            mat_charNew{idx}=[mat_name,'(',num2str(i),',',num2str(j),')'];
        end
        
    end
elseif num_row==1||num_col==1
    mat_name=mat1_1_name(1:end-1);
    for k=1:length(mat)
        mat_charOld{k}=[mat_name,'',num2str(k)];
        mat_charNew{k}=[mat_name,'(',num2str(k),')'];
    end
    
end





