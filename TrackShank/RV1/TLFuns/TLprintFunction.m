function TLprintFunction(FolderName,FuncName,Args,Expr)


fid=fopen([FolderName,'/',FuncName,'.m'],'w+');
fprintf(fid,['function FunOut=',FuncName,'(',Args,')\n']);
fprintf(fid,['FunOut=',TLmat2char(Expr),';\n']);
fprintf(fid,'\nend\n\n');
fclose(fid);


end

