clc 
clear all
close all



load("My_map_little.mat");



% Consider unknown spaces to be unoccupied
omap3D.FreeThreshold = omap3D.OccupiedThreshold;


mapWidth = 50;
mapLength = 50;
maxheight = 40;



q_i = [3 3 3];

q_f = [45 25 30];
%%
figure
show(omap3D);
hold on
scatter3(q_i(1),q_i(2),q_i(3),30,"green","filled")
scatter3(q_f(1),q_f(2),q_f(3),30,"red","filled")

sphere = collisionSphere(0.5);

G=q_i;

N=10;
Max_numb_of_it=200;

delta=20;


path_found=0;
numb_of_it=0;

while path_found==0 && numb_of_it<=Max_numb_of_it
for i=1:N
    numb_of_it=numb_of_it+1;
    if numb_of_it<=Max_numb_of_it

    ind_1 = randi(mapLength);
    ind_2 = randi(mapWidth);
    ind_3= randi(maxheight);
    qrand=[ind_1 ind_2 ind_3];

            vec_dist=[];
    for j=1:length(G(:,1))
        
        dist=norm(qrand-G(j,:));
        vec_dist=[vec_dist dist];
    end
    [min_dis,ind]=min(vec_dist);
    if min_dis>=delta
        q_near=G(ind(1),:);
      
        qnew=q_near+(delta/norm(qrand-q_near))*(qrand-q_near);

        Rect=retta(q_near,qnew);
                flag=1;
                for k=1:length(Rect(:,1))
                    
                    sphere.Pose = trvec2tform(Rect(k,:));
                    if checkMapCollision(omap3D,sphere)==1
                        flag=0;
                        break
                    end
                end
            
                if flag==1
                    G=[G; qnew];
                    hold on 
                    plot3(Rect(:,1),Rect(:,2),Rect(:,3),'LineWidth',2)
                    hold on
                    scatter3(qnew(1),qnew(2),qnew(3),30,"magenta","filled")
                end 
        
    end
    else
        disp('FAILURE: PATH NOT FOUND!')
        break
    end
end

% trying to connect qf to the tree starting from q_i
fin_dis=[];
for j=1:length(G(:,1))
        
        dist=norm(q_f-G(j,:));
        fin_dis=[fin_dis dist];
end
[min_dis,ind]=min(fin_dis);

Rect=retta(q_near,q_f);

 flag=1;
 for k=1:length(Rect(:,1))
                    
   sphere.Pose = trvec2tform(Rect(k,:));
   if checkMapCollision(omap3D,sphere)==1
        flag=0;
         break
   end
end

if flag==1
    path_found=1;
    hold on 
    plot3(Rect(:,1),Rect(:,2),Rect(:,3),'LineWidth',2)
    disp(['SUCCESS! PATH FOUND IN ' num2str(numb_of_it) ' ITERATIONS'])
end

end
