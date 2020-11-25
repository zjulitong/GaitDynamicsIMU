function [ch]=TLmat2char(mat)

num_row=size(mat,1);
num_col=size(mat,2);

ch='[';

for row=1:num_row
    for col=1:num_col
        mat_ij=mat(row,col);
        
        if isa(mat_ij,'double')
            ch=[ch,num2str(mat_ij)];
        elseif isa(mat_ij,'sym')
            ch=[ch,char(mat_ij)];
        end
        
        if (col==num_col) && (row<num_row)
            ch=[ch,';'];
        elseif (col<num_col)
            ch=[ch,','];
        end
    end
%     disp(row);
end
ch=[ch,']'];

end
