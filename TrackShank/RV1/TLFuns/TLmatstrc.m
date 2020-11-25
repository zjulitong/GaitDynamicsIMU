function matOut=TLmatstrc(matIn)

[nzrow, nzcol]=find(matIn);
matOut=sparse(nzrow,nzcol,ones(length(nzrow),1),size(matIn,1),size(matIn,2));



end