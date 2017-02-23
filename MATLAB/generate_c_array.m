h=rand(1,5); % Filter coefficients
fd=fopen('myheader.h','wt');
fprintf(fd,'float h[%d]={\n\t%.2g',length(h),h(1));
fprintf(fd,',\n\t%.2g',h(2:end));
fprintf(fd,'\n};\n');
fclose(fd)