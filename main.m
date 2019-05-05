%% Proyecto final: Correlacion cruzada normalizada
%% Garcia Azano David Kiyoshi
%% Evolucion diferencial y penalizacion (metodo 1)

clear all
close all
clc

img = imread('Image_2.bmp');
temp = imread('Template.bmp');

img_g = rgb2gray(img);
temp_g = rgb2gray(temp);

[img_H,img_W] = size(img_g);
[temp_H,temp_W] = size(temp_g);

val_max = -1;
xp = 0;
yp = 0;

F = 0.8;
CR = 0.5; 
G = 100;
N = 70;
D = 2;

x = round(zeros(D,N));
v = zeros(D,N); 
u = round(zeros(D,N));
best = -1;

%limites 
xl = [1; 1];
xu = [img_H - temp_H; img_W - temp_W];

% vector mutado
for i=1:N
    x(:,i) = xl + (xu-xl).*rand(D,1);
end

for gen=1:G 
    for i=1:N 
        
        r1 = randi([1,N]);
        r2 = randi([1,N]);
        r3 = randi([1,N]);
        
        while(r1 == r2 || r2 == r3 || r1 == i || r2 == i || r3 == i)
            r1 = randi([1,N]);
            r2 = randi([1,N]);
            r3 = randi([1,N]);
        end
        
        v(:,i) = x(:,r1) + F *((x(:,r2)) - (x(:,r3)));        
        ra = rand;
        
        for j=1:D      
            if(ra <= CR)
                u(j,i) = v(j,i);
            else
                u(j,i) = x(j,i);
            end
        end
        
        % Penalizacion
        for j=1:D
            if(xl(j)<= u(j,i) && xu(j)>= u(j,i))
                u(j,i) = u(j,i);
            else
                u(j,i) = xl(j) + (xu(j)-xl(j))*rand();
            end
        end

        %Comparacion con penalizacion.
        ncc1 = NCC(img_g,temp_g,round(u(2,i)),round(u(1,i)));
        if(ncc1 > best)
            best=ncc1;
            yp = u(1,i);
            xp = u(2,i);
        end
    end
end

figure
hold on

imshow(img)

line([xp xp+temp_W], [yp yp],'Color','g','LineWidth',3);
line([xp xp], [yp yp+temp_H],'Color','g','LineWidth',3);
line([xp+temp_W xp+temp_W], [yp yp+temp_H],'Color','g','LineWidth',3);
line([xp xp+temp_W], [yp+temp_H yp+temp_H],'Color','g','LineWidth',3);