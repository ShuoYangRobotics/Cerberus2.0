bar_height = 0.5;
l1_l = 0;
l2_l = 0.5;
l3_l = 1.0;
l4_l = 1.5;
stand_time = 0.05;

figure(3);clf;
set(gcf,'Color','w')
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

tiledlayout(1,3,'Padding','compact');
%%
nexttile;
rectangle('Position',[0 l1_l 0.5+stand_time bar_height],'FaceColor',[0 .0 .0],'EdgeColor',[1 1 1])
rectangle('Position',[0.5 l2_l 0.5 bar_height],'FaceColor',[0 .0 .0],'EdgeColor',[1 1 1])
rectangle('Position',[0 l3_l 0.5+stand_time bar_height],'FaceColor',[0 .0 .0],'EdgeColor',[1 1 1])
rectangle('Position',[0.5 l4_l 0.5 bar_height],'FaceColor',[0 .0 .0],'EdgeColor',[1 1 1])
% additional standing
rectangle('Position',[0 l2_l stand_time bar_height],'FaceColor',[0 .0 .0],'EdgeColor',[1 1 1])
rectangle('Position',[0 l4_l stand_time bar_height],'FaceColor',[0 .0 .0],'EdgeColor',[1 1 1])

axis([0 1 0 2])
xticks([0 0.5 1])
xticklabels({'0%','Gait Phase','100%'})
yticks([l1_l+bar_height/2 l2_l+bar_height/2 l3_l+bar_height/2 l4_l+bar_height/2])
yticklabels({'Leg1','Leg2','Leg3','Leg4'})
title("Standing Trot")
pos = get(gca, 'position')
arrow_x1 = pos(1)+(pos(3))/1.95;
arrow_x2 = pos(1)+(pos(3))/1.066;
arrow_y1 = pos(2)+(pos(4))/8;
annotation('textarrow',[arrow_x1+pos(3)/15 arrow_x1],[arrow_y1 arrow_y1],'String',' Gait time  ','FontSize',10,'Linewidth',2)
annotation('textarrow',[arrow_x2-pos(3)/15 arrow_x2],[arrow_y1 arrow_y1],'String','','FontSize',10,'Linewidth',2)
%%
nexttile;
rectangle('Position',[0 l1_l 0.5 bar_height],'FaceColor',[0 .0 .0],'EdgeColor',[0 0 0])
rectangle('Position',[0.5 l2_l 0.5 bar_height],'FaceColor',[0 .0 .0],'EdgeColor',[0 0 0])
rectangle('Position',[0 l3_l 0.5 bar_height],'FaceColor',[0 .0 .0],'EdgeColor',[0 0 0])
rectangle('Position',[0.5 l4_l 0.5 bar_height],'FaceColor',[0 .0 .0],'EdgeColor',[0 0 0])
axis([0 1 0 2])
xticks([0 0.5 1])
xticklabels({'0%','Gait Phase','100%'})
yticks([l1_l+bar_height/2 l2_l+bar_height/2 l3_l+bar_height/2 l4_l+bar_height/2])
yticklabels({'Leg1','Leg2','Leg3','Leg4'})
title("Trot")
pos = get(gca, 'position')
arrow_x1 = pos(1)+(pos(3))/2.0;
arrow_x2 = pos(1)+(pos(3))/1;
arrow_y1 = pos(2)+(pos(4))/8;
annotation('textarrow',[arrow_x1+pos(3)/10 arrow_x1],[arrow_y1 arrow_y1],'String',' Gait time  ','FontSize',10,'Linewidth',2)
annotation('textarrow',[arrow_x2-pos(3)/10 arrow_x2],[arrow_y1 arrow_y1],'String','','FontSize',10,'Linewidth',2)

%%
nexttile;
rectangle('Position',[0 l1_l 0.5-stand_time bar_height],'FaceColor',[0 .0 .0],'EdgeColor',[1 1 1])
rectangle('Position',[0.5+stand_time l2_l 0.5 bar_height],'FaceColor',[0 .0 .0],'EdgeColor',[1 1 1])
rectangle('Position',[0 l3_l 0.5-stand_time bar_height],'FaceColor',[0 .0 .0],'EdgeColor',[1 1 1])
rectangle('Position',[0.5+stand_time l4_l 0.5 bar_height],'FaceColor',[0 .0 .0],'EdgeColor',[1 1 1])
axis([0 1 0 2])
xticks([0 0.5 1])
xticklabels({'0%','Gait Phase','100%'})
yticks([l1_l+bar_height/2 l2_l+bar_height/2 l3_l+bar_height/2 l4_l+bar_height/2])
yticklabels({'Leg1','Leg2','Leg3','Leg4'})
title("Flying Trot")
pos = get(gca, 'position')
arrow_x1 = pos(1)+(pos(3))/1.8;
arrow_x2 = pos(1)+(pos(3))/1;
arrow_y1 = pos(2)+(pos(4))/8;
annotation('textarrow',[arrow_x1+pos(3)/15 arrow_x1],[arrow_y1 arrow_y1],'String',' Gait time  ','FontSize',10,'Linewidth',2)
annotation('textarrow',[arrow_x2-pos(3)/15 arrow_x2],[arrow_y1 arrow_y1],'String','','FontSize',10,'Linewidth',2)


addpath(genpath("../matlab2tikz/"))

matlab2tikz(strcat('tro_gait_pattern.tex'), 'height', '\fheighta', 'width', '\fwidtha');