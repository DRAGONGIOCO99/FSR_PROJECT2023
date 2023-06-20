clc 
clear all
close all



load("My_map_little.mat");



% Consider unknown spaces to be unoccupied
omap3D.FreeThreshold = omap3D.OccupiedThreshold;


mapWidth = 50;
mapLength = 50;
maxheight = 40;



q_i = [3 3 3 0 0]; %flag 0 stand for tree from start

q_f = [45 25 30 1 0]; %flag 1 stand for tree from goal

 
path=[q_i;q_f];

trees=q_i;
treeg=q_f;


N=10;
offset = 0.5;
Max_numb_of_it=150;
delta=20;

figure
show(omap3D);
hold on
scatter3(q_i(1),q_i(2),q_i(3),30,"green","filled")
scatter3(q_f(1),q_f(2),q_f(3),30,"red","filled")

sphere = collisionSphere(0.5);

path_found=0;
numb_of_it=0;

while path_found==0 && numb_of_it<=Max_numb_of_it
for i=1:N
    numb_of_it=numb_of_it+1;
    if numb_of_it<=Max_numb_of_it
    ind_1 = randi(mapLength);
    ind_2 = randi(mapWidth);
    ind_3 = randi(maxheight);

    ind2_1 = randi(mapLength);
    ind2_2=randi(mapWidth);
    ind2_3=randi(maxheight);

    qrand=[ind_1,ind_2,ind_3];
    qrand2=[ind2_1,ind2_2,ind2_3];
%           hold on
%           plot(qrand(2),qrand(1),'y.','MarkerSize',15)
            vec_dist=[];
            vec_dist2=[];
    for j=1:length(treeg(:,1))
        dist2=norm(qrand2-treeg(j,1:3));
        vec_dist2=[vec_dist2 dist2];
    end

    for k=1:length(trees(:,1))
        dist=norm(qrand-trees(k,1:3));
        vec_dist=[vec_dist dist];
    end
    [min_dis,ind]=min(vec_dist);
    [min_dis2,ind2]=min(vec_dist2);

    if min_dis>=delta
        q_near=trees(ind(1),:);

        qnew=q_near(1:3)+(delta/norm(qrand-q_near(1:3)))*(qrand-q_near(1:3));
      

        Rect=retta(q_near(1:3),qnew);
        
                flag=1;
                for k=1:length(Rect(:,1))
                    sphere.Pose = trvec2tform(Rect(k,:));
                    if checkMapCollision(omap3D,sphere)==1
                        flag=0;
                        break
                    end
                end
            
                if flag==1
                        qnew(end+1)=0;                                                               
                        result = ismember(path, q_near, 'rows');% Use ismember to find the vector in the matrix
                        indices = find(all(result, 2)); % Find the indices of the rows containing the vector
                        qnew(end+1) = indices;
                        trees=[trees; qnew];
                        path=[path; qnew];
                        hold on 
                        plot3(Rect(:,1),Rect(:,2),Rect(:,3),'LineWidth',2)
                        hold on
                        scatter3(qnew(1),qnew(2),qnew(3),30,"magenta","filled")
                end 
        
    end

   if min_dis2>=delta
        q_near2=treeg(ind2(1),:);

            
            
        qnew2=q_near2(1:3)+(delta/norm(qrand2-q_near2(1:3)))*(qrand2-q_near2(1:3));
        

        Rect2=retta(q_near2(1:3),qnew2);
        
                flag=1;
                for k=1:length(Rect2(:,1))
                    sphere.Pose = trvec2tform(Rect2(k,:));
                    if checkMapCollision(omap3D,sphere)==1
                        flag=0;
                        break
                    end
                end
            
                if flag==1
                        qnew2(end+1)=1;
                        result = ismember(path, q_near2, 'rows');% Use ismember to find the vector in the matrix
                        indices = find(all(result, 2)); % Find the indices of the rows containing the vector
                        qnew2(end+1) = indices;
                        treeg=[treeg; qnew2];

                    path=[path; qnew2];
                    hold on  
                    plot3(Rect2(:,1),Rect2(:,2),Rect2(:,3),'LineWidth',2)
                    hold on
                    scatter3(qnew2(1),qnew2(2),qnew2(3),30,"yellow","filled")
                end 
      
   end
    else
        disp('FAILURE: PATH NOT FOUND!')
        break
    end

end
% trying to connect Trees to Treeg to find a path

vec_of_min_dis=[];
pos_of_nod_in_treg=[];
 for j=1:length(trees(:,1))
     fin_dis=[];
        for k=1:length(treeg(:,1))
         dist_fromj=norm(treeg(k,1:3)-trees(j,1:3));
         fin_dis=[fin_dis dist_fromj];
        end
 [min_dis,ind]=min(fin_dis);
 vec_of_min_dis=[vec_of_min_dis min_dis];
 pos_of_nod_in_treg=[ pos_of_nod_in_treg ind(1)];
 end

 [abs_min_dis,indj]=min(vec_of_min_dis);
 


Rect_fin=retta(trees(indj(1),1:3),treeg(pos_of_nod_in_treg(indj(1)),1:3));

 flag=1;
 for k=1:length(Rect_fin(:,1))
     sphere.Pose = trvec2tform(Rect2(k,:));
     if checkMapCollision(omap3D,sphere)==1
         flag=0;
         disp('CONNECTION OF THE TREES NOT POSSIBLE, INCREASING NUMBER OF ITERATIONS')
         break
     end
 end


if flag==1
    path_found=1;
%      result = ismember(path, treeg(pos_of_nod_in_treg(indj(1)),:), 'rows');% Use ismember to find the vector in the matrix
%      indices = find(all(result, 2)); % Find the indices of the rows containing the vector
%     result2 = ismember(path, trees(indj(1),:), 'rows');% Use ismember to find the vector in the matrix
%     indices2 = find(all(result2, 2)); % Find the indices of the rows containing the vector
%     path(indices2,5)=indices;
    hold on 
    plot3(Rect_fin(:,1),Rect_fin(:,2),Rect_fin(:,3),'LineWidth',2)
    disp(['SUCCESS! PATH FOUND IN ' num2str(numb_of_it) ' ITERATIONS'])
end
    
    

end

%% enumerate nodes
for i=1:length(path)
text(path(i,1)+offset, path(i,2)+offset,path(i,3)+offset, num2str(i), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle','Color','red');
end

%% adjacent matrix

A_adj=zeros(length(path),length(path));
for k=1:length(path)
    ind_adj=find(path(:,5)==k);
    if ~isempty(ind_adj)
        for j=ind_adj
            A_adj(k,j)=1;
            A_adj(j,k)=1;
        end
    end
end

result = ismember(path, treeg(pos_of_nod_in_treg(indj(1)),:), 'rows');% Use ismember to find the vector in the matrix
indices = find(all(result, 2)); % Find the indices of the rows containing the vector
result2 = ismember(path, trees(indj(1),:), 'rows');% Use ismember to find the vector in the matrix
indices2 = find(all(result2, 2)); % Find the indices of the rows containing the vector

A_adj(indices,indices2)=1;
A_adj(indices2,indices)=1;

%% path with DFS

Track_path=DFS(A_adj,2,path);


%% PLOT THE REAL PATH
figure
show(omap3D);
hold on
scatter3(Track_path(1,1),Track_path(1,2),Track_path(1,3),30,"green","filled")
RETTA=retta(Track_path(1,:),Track_path(2,:));
hold on
plot3(RETTA(:,1),RETTA(:,2),RETTA(:,3),'LineWidth',2,'Color','yellow')

for j=2:length(Track_path(:,1))-2
hold on
scatter3(Track_path(j,1),Track_path(j,2),Track_path(j,3),30,"magenta","filled")
hold on
scatter3(Track_path(j+1,1),Track_path(j+1,2),Track_path(j+1,3),30,"magenta","filled")
RETTA=retta(Track_path(j,:),Track_path(j+1,:));
hold on
plot3(RETTA(:,1),RETTA(:,2),RETTA(:,3),'LineWidth',2,'Color','yellow')

end

hold on
scatter3(Track_path(end,1),Track_path(end,2),Track_path(end,3),30,"red","filled")
RETTA=retta(Track_path(end-1,:),Track_path(end,:));
hold on
plot3(RETTA(:,1),RETTA(:,2),RETTA(:,3),'LineWidth',2,'Color','yellow')


