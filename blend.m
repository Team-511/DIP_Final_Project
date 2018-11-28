function canvas = imageBlending(img1,img2)

    w1 = imfill(im2bw(uint8(img1), 0),'holes');
    w2 = imfill(im2bw(uint8(img2), 0),'holes');

    w1 = mat2gray(w1);
    w2 = mat2gray(w2);

    img1 = double(img1);
    img2 = double(img2);
    canvas(:,:,1) = ((img1(:,:,1).*w1)+(img2(:,:,1).*w2))./(w1+w2);
    canvas(:,:,2) = ((img1(:,:,2).*w1)+(img2(:,:,2).*w2))./(w1+w2);
    canvas(:,:,3) = ((img1(:,:,3).*w1)+(img2(:,:,3).*w2))./(w1+w2);
    canvas = uint8(canvas);
%   imwrite(img1,'simple_blending.png','png');