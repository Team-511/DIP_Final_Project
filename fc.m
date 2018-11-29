function confidence=fc(ps,kp)
    load('corners.mat','img1')
    gamma=0.01*ones(1,size(ps,1));%1*m
    sigma=0.05*min(size(img1,1),size(img1,2));
    
    distance=pdist2(kp,ps);%n*m
    distance=gaussmf(distance,[sigma 0]);%n*m
    confidence=ones(1,size(ps,1))./(sum(distance)+gamma);  %1*m
end