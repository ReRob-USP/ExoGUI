clc
close all
clear all


files = {'incertezas' , 'model' , 'control_parameters' };
files = {'incertezas_cdc' , 'model_cdc' , 'control_parameters_cdc','loop_externo_cdc' };
for idx = 1:numel(files)
   load([files{idx} '.mat']) ;
end
clear files
clear idx

vars = whos();
vars = {vars.name};


NK = 1;
P = repmat(P,1,NK,1);


TAB = 13;
NL = 10;
folder = 'out/';

for idx = 1:numel(vars)
    var = eval(vars{idx});
    if(length(size(var)) == 2)
        header = 'ARMA_MAT_TXT_FN008';
        [f,r] = size(var);
        str = header;
        str = [str NL sprintf('%d %d ',f,r) NL ];
        
        for ff = 1:f
            for rr = 1:r
                str = [ str sprintf('   %3.16e',var(ff,rr)) ];
            end
            str = [str NL];
        end
        
        fileName = [folder vars{idx} '.dat'];
        fileobj = fopen(fileName,'W');
        fprintf(fileobj,'%s',str);
        fclose(fileobj);
        
    else
        header = 'ARMA_CUB_TXT_FN008';
        [f,r,z] = size(var);
        str = header;
        str = [str NL sprintf('%d %d %d',f,r,z) NL ];
        
        for zz = 1:z
            for ff = 1:f
            for rr = 1:r
                str = [ str sprintf('   %3.16e',var(ff,rr,zz)) ];
            end
            str = [str NL];
        end
         
        end
        
        fileName = [ folder vars{idx} '.dat'];
        fileobj = fopen(fileName,'W');
        fprintf(fileobj,'%s',str);
        fclose(fileobj);
        
    end
end

str = '';
for idx = 1:numel(vars)
    var = eval(vars{idx});
    if(length(size(var)) == 2)
        str = [str 'mat ' vars{idx} ';' NL];
        str = [str vars{idx} '.load("' vars{idx} '.dat");' NL];
    else
        str = [str 'cube ' vars{idx} ';' NL];
        str = [str vars{idx} '.load("' vars{idx} '.dat");' NL];
    end
%         str = [str vars{idx} '.print("' vars{idx} '");' NL];
end
fileName = [folder 'vars.cpp'];
fileobj = fopen(fileName,'W');
fprintf(fileobj,'%s',str);
fclose(fileobj);

