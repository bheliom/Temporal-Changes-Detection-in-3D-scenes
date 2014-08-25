function output = getPSM(directory, noOfImages, K , outdir)
tic
for i = 0:noOfImages-1
    count = 1;
    tmp_dir = [directory '/' num2str(i) '/'];
    newImg = imread([tmp_dir num2str(i) 'new.jpg']);
    new_img = rgb2gray(newImg);
    img1vec = new_img(:);
    
    inputMat = zeros(size(img1vec,1),K+1);
    inputMat(:,1) = img1vec;
    count = count + 1;
    
    tmp_str = [num2str(i) '0'];
    tmp_dir = [tmp_dir tmp_str];
    for j = 0 : K - 1                        
        if(exist([tmp_dir 'old.jpg'],'file'));
            old = imread([tmp_dir 'old.jpg']);
            old_gray = rgb2gray(old);        

            img2vec = old_gray(:);
            inputMat(:,count) = img2vec;            
            count = count + 1;      
        end
        tmp_dir = [tmp_dir num2str(j+1)];
    end
    
    lambda = 1.5/sqrt(size(img1vec,1));
    i
    [A E iter] = inexact_alm_rank1soft(inputMat,lambda);
    
    aImg1 = reshape(A(:,1),size(new_img,1),size(new_img,2),size(new_img,3));
    eImg1 = reshape(E(:,1),size(new_img,1),size(new_img,2),size(new_img,3));
    
    mask_img = zeros(size(eImg1));
    mask_img(eImg1>0) = 255;
    
    imwrite(mask_img, [outdir '/mask' num2str(i) '.jpg'], 'JPG');
end

output = toc;