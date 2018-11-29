function B=rowmove(A,x)
%x>0,”““∆x
%x<0,◊Û“∆x
[m,n]=size(A);
if x>0
   C=A(:,1:n-x);
   D=zeros(m,x);
   B=cat(2,D,C);
else if x<0
        C=A(:,abs(x)+1:n);
        D=zeros(m,abs(x));
        B=cat(2,C,D);
    end
end
end