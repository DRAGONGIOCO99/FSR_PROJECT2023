function [track_path] = DFS(A_adj,i_goal,path)

num_nodes=length(A_adj(:,1)); 

Vis_nodes=zeros(num_nodes);
Vis_nodes(1)=1;

Nodes=zeros(num_nodes);
for i=1:num_nodes
    Nodes(i)=i;
end

Open=[1 ; 0];
parent_memory=[]; %  to have memory of the connection in the three
p=[];
Ng=Nodes(i_goal);
num_iteration=0;

Ng_is_in_open=0; % end condition

%% start of algoritm 
tree=digraph();

while (Vis_nodes(i_goal)~=1 && not(isempty(Open)))
        i=find(Nodes==Open(1,end));
        if Open(1,end)==Nodes(i)
            parent=Open(2,end);
            parent_memory=[parent_memory Open(:,end)];
            Open(:,end)=[];
            if num_iteration~=0
            tree=addedge(tree,Nodes(parent),Nodes(i));
            figure
            plot(tree,'LineWidth',2);
            end
            num_iteration=num_iteration+1;
            index=find(A_adj(i,:)==1);
            for j=1:length(index)
                if Vis_nodes(index(j))==0
                Open(:,end+1)=[Nodes(index(j));i];
                Vis_nodes(index(j))=1;
                end
            end
        end
end

% add the goal node to the graph
i_goal_in_open=find(Open(1,:)==Ng);
parent=Open(2,i_goal_in_open);
tree=addedge(tree,Nodes(parent),Nodes(i_goal)); % adding Ng to the tree 
figure()
plot(tree,'LineWidth',2)
parent_memory=[parent_memory Open(:,i_goal_in_open)];

disp("the tree is done")


%% searching the real Path in the tree
    
now_node=Ng;
track_path=[path(now_node,1:3)]; % Path=[Ns---------NG]


while true
    i_parent=find(parent_memory(1,:)==now_node);
    parent_now=parent_memory(2,i_parent);
        if parent_now==0
            break
        end
    now_node=Nodes(parent_now);
    track_path=[path(Nodes(parent_now),1:3); track_path];
end



% path plot

% Path_G=digraph();
% for i=1:length(Path)-1
%     Path_G=addedge(Path_G,Path(i),Path(i+1));
% end
% figure
% 
% plot(Path_G)




end