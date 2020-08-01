clear all
close all
clc

%% Specify files to load
vfile = '../data/killian-v.dat';
efile = '../data/killian-e.dat';

%%

vfid = fopen(vfile);            
if (vfid < 0)
    fprintf('Fail to open %s\n.', vfile);
    return
end
vertices = fscanf(vfid, 'VERTEX2 %d %f %f %f\n', [4 Inf]);

efid = fopen(efile);
            if (efid < 0)
                fprintf('Fail to open %s\n.', vfile);
                return
            end
            edges = fscanf(efid,...
                'EDGE2 %d %d %f %f %f %f %f %f %f %f %f \n',[11 Inf]);
% figure()
% plot(vertices(2,:), vertices(3,:))
%% 
figure()
pg = PoseGraph();
pg.readGraph(vfile, efile);
% Do 5 iteration with visualization
pg.optimize(3, true);

