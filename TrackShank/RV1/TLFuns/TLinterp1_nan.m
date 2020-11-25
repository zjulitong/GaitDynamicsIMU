function yout=TLinterp1_nan(t1,y1,t2,method)


if size(t1,2)>1
    disp('t1 is not vertical vector');
    yout=0;
    return;
end

n_col=size(y1,2);
y2=zeros(length(t2),n_col);

for i=1:n_col
    if nnz(isnan(y1(:,i)))>2
        y2(:,i)=nan;
    else
        y2(:,i)=interp1(t1,y1(:,i),t2,method);
    end
end
yout=y2;

end
