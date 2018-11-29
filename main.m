function linear_hom=main(ima1,ima2)
global fitfn resfn degenfn psize numpar  
fitfn = 'homography_fit';   %fit function
resfn = 'homography_res';   
degenfn = 'homography_degen';
psize   = 4;
numpar  = 9;

M     = 500;  % Number of hypotheses for RANSAC.
thr   = 0.1;  % RANSAC threshold.

scale = 0.25;    % Scale of input images (maybe for large images you would like to use a smaller scale).

%-------
% Paths.
%-------
addpath('modelspecific');
addpath('mexfiles');
addpath('multigs');


cd multigs;
if exist('computeIntersection','file')~=3  
    mex computeIntersection.c;
end
cd ..;

cd mexfiles;
if exist('imagewarping','file')~=3
    mex ../imagewarping.cpp; 
end
if exist('wsvd','file')~=3
    mex ../wsvd.cpp; % We make use of eigen3's SVD in this file.
end
cd ..;

%----------------------
% Setup VLFeat toolbox.  
%----------------------
cd vlfeat-0.9.21/toolbox;
feval('vl_setup');
cd ../..;


%-------------
% Read images.
%-------------

fprintf('Read images and SIFT matching\n');tic;
fprintf('> Reading images...');tic;
img1 = imresize(imread(ima1),scale);
img2 = imresize(imread(ima2),scale);
fprintf('done (%fs)\n',toc);

  
C1=[1;1;1]; 
C2=[size(img2,2);1;1];
C3=[1;size(img2,1);1];
C4=[size(img2,2);size(img2,1);1];
%--------------------------------------
% SIFT keypoint detection and matching.
%--------------------------------------


fprintf('  Keypoint detection and matching...');tic;
% [image, descriptors, locs] = sift(imageFile)
%     descriptors: a K-by-128 matrix, where each row gives an invariant
%         descriptor for one of the K keypoints.  The descriptor is a vector
%         of 128 values normalized to unit length.
%     locs: K-by-4 matrix, in which each row has the 4 values for a
%         keypoint location (row, column, scale, orientation).  The 
%         orientation is in the range [-PI, PI] radians.


[ kp1,ds1 ] = sift(single(rgb2gray(img1))); 
[ kp2,ds2 ] = sift(single(rgb2gray(img2)));


%[ kp1,ds1 ] = vl_sift(single(rgb2gray(img1)),'PeakThresh', 0,'edgethresh',3);
%[ kp2,ds2 ] = vl_sift(single(rgb2gray(img2)),'PeakThresh', 0,'edgethresh',3);

kp1=kp1(:,1:2);     
[kp1,i,~]=unique(kp1,'rows','stable'); 
ds1=ds1(i,:);    
kp2=kp2(:,1:2);   
[kp2,i,~]=unique(kp2,'rows','stable');
ds2=ds2(i,:);
%disp(ds2); % 4x183
matches = match(ds1,ds2); 
%kp:K-by-4,(y, x, scale, orientation)
kp1=kp1'; 
kp2=kp2';


matches   = vl_ubcmatch(ds1,ds2);
[matches, scores]= vl_ubcmatch(ds1,ds2);
numBestPoints = 100;
[~,indices] = sort(scores);
%// Get the numBestPoints best matched features
bestMatches = matches(:,indices(1:numBestPoints));
%}

fprintf('done (%fs)\n',toc);

% Normalise point distribution.
fprintf('  Normalising point distribution...');tic;

%data_orig = [ kp1(1:2,matches(1,:)) ; ones(1,size(matches,2)) ; kp2(1:2,matches(2,:)) ; ones(1,size(matches,2)) ];

data_orig = [ kp1(2,matches(1,:)) ; 
              kp1(1,matches(1,:));
              ones(1,size(matches,2)) ; 
              kp2(2,matches(2,:)) ; 
              kp2(1,matches(2,:)) ;
              ones(1,size(matches,2)) ];

data_orig=data_orig';
data_orig=unique(data_orig,'rows','stable');
data_orig=data_orig';


[ dat_norm_img1,T1 ] = normalise2dpts(data_orig(1:3,:));
%dat_norm_img1=T1*data_orig(1:3,:)
[ dat_norm_img2,T2 ] = normalise2dpts(data_orig(4:6,:));
data_norm = [ dat_norm_img1 ; dat_norm_img2 ];
fprintf('done (%fs)\n',toc);

%-----------------
% Outlier removal.
%-----------------
fprintf('Outlier removal\n');tic;
% Multi-GS
rng(0);
% M     = 500;  % Number of hypotheses for RANSAC.
% thr   = 0.1;  % RANSAC threshold.

[ ~,res,~,~ ] = multigsSampling(100,data_norm,M,10);  
con = sum(res<=thr);  
[ ~, maxinx ] = max(con);
inliers = find(res(:,maxinx)<=thr);

%inliers=inliers(1:2:end);


if size(img1,1) == size(img2,1)
    % Show results of RANSAC.
    fprintf('  Showing results of RANSAC...');tic;
%     figure;
%     imshow([img1 img2]);
%     hold on;
    %plot(data_orig(1,:),data_orig(2,:),'ro','LineWidth',2);
    %plot(data_orig(4,:)+size(img1,2),data_orig(5,:),'ro','LineWidth',2);
    for i=1:length(inliers)
     %   plot(data_orig(1,inliers(i)),data_orig(2,inliers(i)),'go','LineWidth',2);
     %   plot(data_orig(4,inliers(i))+size(img1,2),data_orig(5,inliers(i)),'go','LineWidth',2);
        %plot([data_orig(1,inliers(i)) data_orig(4,inliers(i))+size(img1,2)],[data_orig(2,inliers(i)) data_orig(5,inliers(i))],'g-');
    end
   % title('Ransac''s results');
    fprintf('done (%fs)\n',toc);
end


keypoint_img1=data_orig(1:2,inliers);
keypoint_img2=data_orig(4:5,inliers);
keypoint_norm=data_norm(:,inliers);

%-------------------------------------------------------------
% Step2.Finding Best-fitting Global homography (H).   best-fitting
%-------------------------------------------------------------
penalty = zeros(length(inliers),1); %seed penaltyֵ
flag=zeros(length(inliers),1); 
%Hg_fitness_threshold=2
Hg_fitness_threshold=3;
sum_threshold=0.01;             

edge_cost=1;
Hg_stack=[];           
Hg_fitness_stack=[];   
edge_cost_stack=[];    %edge_cost
while edge_cost>0.05
    if mean(flag)==1   
        break;
    end    
    rng('shuffle');                     
    seednum=randi(length(inliers));    
    
    
    if flag(seednum)==1
       continue;
    else
       flag(seednum)=1; 
    end   
    if penalty(seednum)>mean(penalty)
        continue;
    end

    seedfpt=data_orig(4:5,inliers(seednum));    %seed feature point
    index=knnsearch(keypoint_img2',seedfpt','K',length(inliers));  
    %idx_features_group
    features_group_idx=index(1:9); 
    
    is_Hg_exist=0;
    for i=10:length(index)
        %grouping features
        features_group_idx=[features_group_idx,index(i)]; 
        
        [ h,A,D1,D2 ] = feval(fitfn,keypoint_norm(:,features_group_idx));
        Hg = T2\(reshape(h,3,3)*T1);    
        Hg_fitness=fitness(A,h);
        Hg_fitness;
        if Hg_fitness>Hg_fitness_threshold 
            if i==10  
                is_Hg_exist=0;
                break;
            else3
                features_group_idx=features_group_idx(:,1:i-1);
                [ h,A,D1,D2 ] = feval(fitfn,keypoint_norm(:,features_group_idx));
                Hg = T2\(reshape(h,3,3)*T1); 
                Hg_fitness=fitness(A,h);
                is_Hg_exist=1;
                break;
            end
        end
        if i==length(index)
            is_Hg_exist=1;
        end
    end
    
    if is_Hg_exist==0
        continue;
    end
    
    penalty(features_group_idx)=penalty(features_group_idx)+1;
     %obtaining the four corners 
    TL = Hg\[1;1;1];
    TL = round([ TL(1)/TL(3) ; TL(2)/TL(3) ]);
    BL = Hg\[1;size(img2,1);1];     
    BL = round([ BL(1)/BL(3) ; BL(2)/BL(3) ]);
    TR = Hg\[size(img2,2);1;1];     
    TR = round([ TR(1)/TR(3) ; TR(2)/TR(3) ]);
    BR = Hg\[size(img2,2);size(img2,1);1];  
    BR = round([ BR(1)/BR(3) ; BR(2)/BR(3) ]);
    %{
    if TL(1)>TR(1) || TL(2)>BL(2) || BR(1)<BL(1) || BR(2)<TR(2)
        continue;
    end
    %}
    % Setting the Canvas size.
    cw = max([1 size(img1,2) TL(1) BL(1) TR(1) BR(1)]) - min([1 size(img1,2) TL(1) BL(1) TR(1) BR(1)]) + 1; 
    ch = max([1 size(img1,1) TL(2) BL(2) TR(2) BR(2)]) - min([1 size(img1,1) TL(2) BL(2) TR(2) BR(2)]) + 1; 

    % Offset for left image.
    off = [ 1 - min([1 size(img1,2) TL(1) BL(1) TR(1) BR(1)]) + 1 ; 1 - min([1 size(img1,1) TL(2) BL(2) TR(2) BR(2)]) + 1 ];
    % Warping source image with global homography 
    warped_img1 = uint8(zeros(ch,cw,3));
    warped_img1(off(2):(off(2)+size(img1,1)-1),off(1):(off(1)+size(img1,2)-1),:) = img1; 
    warped_img2 = imagewarping(double(ch),double(cw),double(img2),Hg,double(off));
    warped_img2 = reshape(uint8(warped_img2),size(warped_img2,1),size(warped_img2,2)/3,3); 
    
%     imwrite(warped_img1,'warped_img1.png','png');
%     imwrite(warped_img2,'warped_img2.png','png');
%     
    
 
    %Homography Screening    
    % C1=[1;1;1]; 
    % C2=[size(img2,2);1;1];
    % C3=[1;size(img2,1);1];
    % C4=[size(img2,2);size(img2,1);1];
    C1bar=Hg*C1;
    C1bar=round([ C1bar(1)/C1bar(3) ; C1bar(2)/C1bar(3) ]);
    C2bar=Hg*C2;
    C2bar=round([ C2bar(1)/C2bar(3) ; C2bar(2)/C2bar(3) ]);
    C3bar=Hg*C3;
    C3bar=round([ C3bar(1)/C3bar(3) ; C3bar(2)/C3bar(3) ]);
    C4bar=Hg*C4;
    C4bar=round([ C4bar(1)/C4bar(3) ; C4bar(2)/C4bar(3) ]);        
    save('corners.mat','C1bar','C2bar','C3bar','C4bar','img2','img1')
    %||Hs*Ci-Cibar||^2=0
    %Hs'Hs*Ci-Hs'Cibar=0
    A_hs=[C1(1),-C1(2),1,0;
          C1(2),C1(1),0,1;
          C2(1),-C2(2),1,0;
          C2(2),C2(1),0,1;
          C3(1),-C3(2),1,0;
          C3(2),C3(1),0,1;
          C4(1),-C4(2),1,0;
          C4(2),C4(1),0,1];
    b_hs=[C1bar(1);
          C1bar(2);
          C2bar(1);
          C2bar(2);
          C3bar(1);
          C3bar(2);
          C4bar(1);
          C4bar(2);];
    X_hs=(A_hs'*A_hs)\A_hs'*b_hs;
    dist_hs=A_hs*X_hs-b_hs;      
    Hs=[X_hs(1),-X_hs(2),X_hs(3);X_hs(2),X_hs(1),X_hs(4)];  
    
    C1s=Hs*[1;1;1]; 
    C2s=Hs*[size(img1,2);1;1]; 
    C3s=Hs*[1;size(img1,1);1];
    C4s=Hs*[size(img1,2);size(img1,1);1]; 
    Sum=norm(C1bar-C1s)+norm(C2bar-C2s)+norm(C3bar-C3s)+norm(C4bar-C4s);
    img1_size=size(img1,1)*size(img1,2);
    norm_sum=Sum/img1_size;
    if norm_sum>sum_threshold 
       continue;
    end
    
    
    w1 = imfill(im2bw(uint8(warped_img1), 0),'holes');
    w2 = imfill(im2bw(uint8(warped_img2), 0),'holes');
    
    mask = w1 & w2;
    
    border1=getborder(w1);
    border2=getborder(w2);
    border=border1&border2;
    border1=border1-border;   
    source=border2&mask;
    sink=border1&mask;  
    %[i,j]=find(A)
    [gy, gx] = find(mask); 
    miny = min(gy);
    minx = min(gx);
    maxy = max(gy);
    maxx = max(gx);
    
    kp_img=zeros(ch,cw);
    for i=1:length(index)
        point1=keypoint_img1(:,index(i)); %img1
        point2=keypoint_img2(:,index(i)); %img2
        point2=pt_after_H(point2,Hg);
        kp_img(uint8(point1(2))+off(2)-1,uint8(point1(1))+off(1)-1)=1;
        kp_img(point2(2)+off(2)-1,point2(1)+off(1)-1)=1;
    end
    kp_img=mask&kp_img;
    kp_img=kp_img(miny:maxy,minx:maxx);
    [kpy,kpx]=find(kp_img);
    kp=[kpy,kpx];
    
    mask=mask(miny:maxy,minx:maxx);
    
    wd=maxx-minx+1;
    ht=maxy-miny+1;

    all_one=ones(ht,wd);
    [i,j]=find(all_one);
    ps=[i j];
    fc_dim1=fc(ps,kp);%1*m -- alignment confidence function
    fc_img=reshape(fc_dim1,ht,wd);
    fc_img=fc_img.*mask;
    
  %  imwrite(fc_img,'fc_img.png','png');
    
    gray_warped_img1=rgb2gray(warped_img1);  
    gray_warped_img2=rgb2gray(warped_img2);
    warped_img1_edge = edge(gray_warped_img1,'canny'); % getting the edge map by Canny edge detector
    warped_img2_edge = edge(gray_warped_img2,'canny'); %getting the edge map by Canny edge detector
    Ed=bitxor(warped_img1_edge,warped_img2_edge); %difference map between the edge maps of warped and ref img
    Ed=Ed(miny:maxy,minx:maxx);
    
%     figure;
%     imshow(Ed);

    fced_img=fc_img.*Ed;
    cost_x=fced_img+rowmove(fced_img,-1)+1e-6; %cost function as mentioned in the report
    infinity=ones(ht,wd)*1e10;  
    cost_x=cost_x.*mask+rowmove(infinity,-1).*(~mask);  
    cost_x(:,wd)=0;
 
    cost_y=fced_img+colmove(fced_img,-1)+1e-6; %cost function as mentioned in the report
    cost_y=cost_y.*mask+colmove(infinity,-1).*(~mask);
    cost_y(ht,:)=0;
    
    
    source=source(miny:maxy,minx:maxx);
    sink=sink(miny:maxy,minx:maxx);
    source=source+~mask;
    source=source*1e10; 
    sink=sink*1e10; 
    source=source'; 
    sink=sink';
    dc=[source(:)';sink(:)'];
     
    Ix=(1:wd*ht-1)';
    Jx=(2:wd*ht)';
    Sx=cost_x';
    Sx=Sx(:);
    Sx=Sx(1:wd*ht-1);

    Iy=(1:wd*(ht-1))';
    Jy=(1+wd:wd*ht)';
    Sy=cost_y';
    Sy=Sy(:);
    Sy=Sy(1:wd*(ht-1));

    I=[Ix;Iy];
    J=[Jx;Jy];
    S=[Sx;Sy];
    nb=sparse(I,J,S,wd*ht,wd*ht); 
    hinc = BK_Create(wd*ht,2*wd*ht);   
    BK_SetUnary(hinc,dc);              
    BK_SetNeighbors(hinc,nb);          
    edge_cost=BK_Minimize(hinc);      
    
    
    L=BK_GetLabeling(hinc);           
    maskone=zeros(wd*ht,1);
    maskone=im2uint8(maskone)+1;
    B=L-maskone;    
    C=reshape(B,wd,ht);
    
    C=C';                  
    Cc=C.*255;
    D=uint8(~C);
    
    %imwrite(C,'warped_img1.png','png');
    
    
    Hg_stack=[Hg_stack;Hg];
    edge_cost_stack=[edge_cost_stack;edge_cost];
    Hg_fitness_stack=[Hg_fitness_stack;Hg_fitness];
    %features group
    if size(img1,1) == size(img2,1)
        fprintf('  Showing results of features group');tic;
       % figure;
      %  imshow([img1 img2]);
       % hold on;   
     %   plot(keypoint_img1(1,index),keypoint_img1(2,index),'ro','LineWidth',2);
     %   plot(keypoint_img2(1,index)+size(img1,2),keypoint_img2(2,index),'ro','LineWidth',2);

     %   plot(keypoint_img1(1,features_group_idx),keypoint_img1(2,features_group_idx),'go','LineWidth',2);
      %  plot(keypoint_img2(1,features_group_idx)+size(img1,2),keypoint_img2(2,features_group_idx),'go','LineWidth',2);

      %  title('features group''s results');
        fprintf('done (%fs)\n',toc);
    end
end
[~,min_index]=min(edge_cost_stack);
Hg=Hg_stack(3*min_index-2:3*min_index,:);



%{
if size(img1,1) == size(img2,1)
    fprintf('  Showing results of features group');tic;
    figure;
    imshow([img1 img2]);
    hold on;    
    %{
    plot(data_orig(1,:),data_orig(2,:),'ro','LineWidth',2);
    plot(data_orig(4,:)+size(img1,2),data_orig(5,:),'ro','LineWidth',2);
    %}
    
    plot(keypoint_img1(1,index),keypoint_img1(2,index),'ro','LineWidth',2);
    plot(keypoint_img2(1,index)+size(img1,2),keypoint_img2(2,index),'ro','LineWidth',2);
   
    plot(keypoint_img1(1,features_group_idx),keypoint_img1(2,features_group_idx),'go','LineWidth',2);
    plot(keypoint_img2(1,features_group_idx)+size(img1,2),keypoint_img2(2,features_group_idx),'go','LineWidth',2);
   
    title('features group''s results');
    fprintf('done (%fs)\n',toc);
end
%}
TL = Hg\[1;1;1]; 
TL = round([ TL(1)/TL(3) ; TL(2)/TL(3) ]);
BL = Hg\[1;size(img2,1);1]; 
BL = round([ BL(1)/BL(3) ; BL(2)/BL(3) ]);
TR = Hg\[size(img2,2);1;1];
TR = round([ TR(1)/TR(3) ; TR(2)/TR(3) ]);
BR = Hg\[size(img2,2);size(img2,1);1]; 
BR = round([ BR(1)/BR(3) ; BR(2)/BR(3) ]);
cw = max([1 size(img1,2) TL(1) BL(1) TR(1) BR(1)]) - min([1 size(img1,2) TL(1) BL(1) TR(1) BR(1)]) + 1;
ch = max([1 size(img1,1) TL(2) BL(2) TR(2) BR(2)]) - min([1 size(img1,1) TL(2) BL(2) TR(2) BR(2)]) + 1;
off = [ 1 - min([1 size(img1,2) TL(1) BL(1) TR(1) BR(1)]) + 1 ; 1 - min([1 size(img1,1) TL(2) BL(2) TR(2) BR(2)]) + 1 ];
warped_img1 = uint8(zeros(ch,cw,3));
warped_img1(off(2):(off(2)+size(img1,1)-1),off(1):(off(1)+size(img1,2)-1),:) = img1;
%mask=mask(miny:maxy,minx:maxx)
warped_img1(miny:maxy,minx:maxx,1)=warped_img1(miny:maxy,minx:maxx,1).*C;
warped_img1(miny:maxy,minx:maxx,2)=warped_img1(miny:maxy,minx:maxx,2).*C;
warped_img1(miny:maxy,minx:maxx,3)=warped_img1(miny:maxy,minx:maxx,3).*C;

 
 
warped_img2 = imagewarping(double(ch),double(cw),double(img2),Hg,double(off));
warped_img2 = reshape(uint8(warped_img2),size(warped_img2,1),size(warped_img2,2)/3,3);
% 
% 
warped_img2(miny:maxy,minx:maxx,1)=warped_img2(miny:maxy,minx:maxx,1).*D;
warped_img2(miny:maxy,minx:maxx,2)=warped_img2(miny:maxy,minx:maxx,2).*D;
warped_img2(miny:maxy,minx:maxx,3)=warped_img2(miny:maxy,minx:maxx,3).*D;


% fprintf('  Showing results of warped imgs');tic;
% figure;
% imshow([warped_img1 warped_img2]);




%-------------------------------------------------------------
% Step4.Employ the optimal homography to pre-align images
%-------------------------------------------------------------


% Blending images by simple average (linear blending)
fprintf('  Homography linear image blending (averaging)...');tic;
linear_hom = imageblending(warped_img1,warped_img2);
fprintf('done (%fs)\n',toc);
% figure;
% imshow(linear_hom);
% title('Image Stitching with global homography');
imwrite(linear_hom,'GraphCutBlend.png','png');
%{
figure;
label=BK_GetLabeling(hinc);
result=reshape(label,[wd,ht]);
imagesc(result');
drawnow;
%}


