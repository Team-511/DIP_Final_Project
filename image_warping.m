function [warped_img2] = image_warping(image1,H)
    ch = 0;
    cw = 0;
    img = image1;
    Hg = H;


    ch1canv = 0;
    ch2canv = canvn*canvm;
    ch3canv = canvn*canvm*2;

    ch1img2 = 0;
    ch2img2 = img2n*img2m;
    ch3img2 = img2n*img2m*2;

    %/* Start computations. */
    if (nrhs == 5)%/*We stitch Global Homography (DLT). */

        %/* For each pixel in the target image... */
        for i = 1:canvm
            for i = 1:canvn

                %/* Get projective transformation for current source point (i,j). */
                posa = round(((H((Hm*0)+0) * (j-off(0)+1)) + (H((Hm*1)+0) * (i-off(1)+1)) + H((Hm*2)+0)) / ((H((Hm*0)+2) * (j-off(0)+1)) + (H((Hm*1)+2) * (i-off(1)+1)) + H((Hm*2)+2)));
                posb = round(((H((Hm*0)+1) * (j-off(0)+1)) + (H((Hm*1)+1) * (i-off(1)+1)) + H((Hm*2)+1)) / ((H((Hm*0)+2) * (j-off(0)+1)) + (H((Hm*1)+2) * (i-off(1)+1)) + H((Hm*2)+2)));

                %/* Find if the current pixel/point in the (warped) source image falls inside canvas. */
                if ((posa > 1)&&(posa < img2n)&&(posb>1)&&(posb<img2m))

                    % /* If the current pixel in the source image hits the target (i.e., is inside the canvas)
                    %  * we get its corresponding position in the 2D array. */
                    cidx = ((j-1)*canvm)+(i-1);
                    sidx = ((posa-1)*img2m)+(posb-1);

                    %/* Warping pixel in source image to canvas. */
                    if (cidx+ch1canv >= 0 && cidx+ch3canv < canvm*canvn*3 && sidx+ch1img2 >= 0 && sidx+ch3img2 < img2m*img2n*3)

                        warped_img2(cidx+ch1canv) = img2(sidx+ch1img2);
                        warped_img2(cidx+ch2canv) = img2(sidx+ch2img2);
                        warped_img2(cidx+ch3canv) = img2(sidx+ch3img2);
                    end;
                end;
            end;
        end;
    end;
    else if(nrhs == 7)
        %/* For each point in the grid. */
         for i = 1:canvm

             for i = 1:canvn

                %/* Get grid point for current pixel. */
                for(xinx=0; xinx < xn && j >= X(xinx); xinx++);
                for(yinx=0; yinx < yn && i >= Y(yinx); yinx++);

                inx = yinx + xinx*yn;

                posa = round(((H(inx+(Hm*0)) * (j-off(0)+1)) + (H(inx+(Hm*3)) * (i-off(1)+1)) + H(inx+(Hm*6))) / ((H(inx+(Hm*2)) * (j-off(0)+1)) + (H(inx+(Hm*5)) * (i-off(1)+1)) + H(inx+(Hm*8))));
                posb = round(((H(inx+(Hm*1)) * (j-off(0)+1)) + (H(inx+(Hm*4)) * (i-off(1)+1)) + H(inx+(Hm*7))) / ((H(inx+(Hm*2)) * (j-off(0)+1)) + (H(inx+(Hm*5)) * (i-off(1)+1)) + H(inx+(Hm*8))));

                if ((posa>0)&&(posa<img2n)&&(posb>0)&&(posb<img2m))
                    cidx = ((j-1)*canvm)+(i-1);
                    sidx = ((posa-1)*img2m)+(posb-1);

                    if ((cidx+ch1canv > 0) && (cidx+ch3canv < canvm*canvn*3) && (sidx+ch1img2 > 0) && (sidx+ch3img2 < img2m*img2n*3))

                        warped_img2(cidx+ch1canv) = img2(sidx+ch1img2);
                        warped_img2(cidx+ch2canv) = img2(sidx+ch2img2);
                        warped_img2(cidx+ch3canv) = img2(sidx+ch3img2);
                    end;
                end
            end;
        end;
    end;
