function coef=TLcorrcoef(A,B)

sizeA=size(A);
sizeB=size(B);

if min(sizeA==sizeB)==0
    disp('A and B have different size');
end

coef=zeros(1,sizeA(2));
for i=1:sizeA(2)
    temp=corrcoef(A(:,i),B(:,i));
    coef(i)=temp(1,2);
end

end
