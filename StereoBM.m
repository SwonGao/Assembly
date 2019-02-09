clc,clear;
left=imread('.\im6.png');
left=rgb2gray(left);
right=imread('.\im2.png');
right=rgb2gray(right);
left   = im2double(left);
right  = im2double(right);
Block=3;HalfBlock=(Block-1)/2;
DisparityRange = 120;
[column,row]=size(left); %i行 j列
D=zeros(column,row);%D是记录深度信息。

for i=1+HalfBlock:column-HalfBlock
    for j=1+HalfBlock:row-HalfBlock
        R1=inf*ones(2,DisparityRange);
        if j<=DisparityRange/2+HalfBlock
            for k=1+HalfBlock:DisparityRange+HalfBlock
                lefty=left(i-HalfBlock:i+HalfBlock,j-HalfBlock:j+HalfBlock);
                righty=right(i-HalfBlock:i+HalfBlock,k-HalfBlock:k+HalfBlock);
                R1(1,k-HalfBlock)=sum(sum(abs((lefty-righty))));
                R1(2,k-HalfBlock)=k;
            end
        end
        
        if (j>DisparityRange/2+HalfBlock && j<row-DisparityRange/2-HalfBlock)
            temp=1;
            for k= (j-DisparityRange/2) :(j+DisparityRange/2)
                lefty=left(i-HalfBlock:i+HalfBlock,j-HalfBlock:j+HalfBlock);
                righty=right(i-HalfBlock:i+HalfBlock,k-HalfBlock:k+HalfBlock);
                R1(1,temp)=sum(sum(abs((lefty-righty))));
                R1(2,temp)=k;
                temp=temp+1;
            end
        end
        
        if (j>=row-DisparityRange/2-HalfBlock)
            temp=1;
           for k= (row-HalfBlock-DisparityRange) : (row-HalfBlock)
                lefty=left(i-HalfBlock:i+HalfBlock,j-HalfBlock:j+HalfBlock);
                righty=right(i-HalfBlock:i+HalfBlock,k-HalfBlock:k+HalfBlock);
                R1(1,temp)=sum(sum(abs((lefty-righty))));
                R1(2,temp)=k;
                temp=temp+1;
           end
        end
        %三种情况
        MIN=100;x_min=0;
        for k=1:DisparityRange
            if (R1(1,k)<MIN) 
                MIN = R1(1,k);
                x_min=R1(2,k);
            end
        end
        D(i-HalfBlock,j-HalfBlock)=(j-x_min);
    end
end
D=D+(-min(min(D)));
D= D*2/( max(max(D)) );
imshow(D);

