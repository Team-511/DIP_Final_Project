%{
I = vl_impattern('river1');
image(I);
I = single(rgb2gray(I));
[f,d] = vl_sift(I);
perm = randperm(size(f,2));
sel = perm(1:50);
h1 = vl_plotframe(f(:,sel));
h2 = vl_plotframe(f(:,sel));

set(h1,'color','k','linewidth',3);
set(h2,'color','y','linewidth',2);
%}

Ia = im2single(vl_impattern('river1'));
%figure;
%image(Ia);
Ib = im2single(vl_impattern('river2'));
%figure;
%image(Ib);

[fa,da] = vl_sift(Ia);
[fb,db] = vl_sift(Ib);
[matches,scores] = vl_ubcmatch(da,db);

