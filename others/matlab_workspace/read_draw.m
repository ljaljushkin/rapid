arr = csvread('output/rvec_and_tvec.txt');
r_arr = csvread('output/ransac_rvec_and_tvec.txt');

rx = arr( : , 1 );
ry = arr( : , 2 );
rz = arr( : , 3 );

tx = arr( : , 4 );
ty = arr( : , 5 );
tz = arr( : , 6 );

r_rx = r_arr( : , 1 );
r_ry = r_arr( : , 2 );
r_rz = r_arr( : , 3 );

t_tx = r_arr( : , 4 );
t_ty = r_arr( : , 5 );
t_tz = r_arr( : , 6 );

a = -5;
b = 5;

figure('name','Delta rotation vectors')
scatter3(rx,ry,rz,...
        'MarkerEdgeColor','k',...
        'MarkerFaceColor',[.75 .75 0])   
hold on
    
ra = linspace(r_rx, 0, 100);
rb = linspace(r_ry, 0, 100);
rc = linspace(r_rz, 0, 100);

plot3(ra, rb, rc, 'LineWidth', 3, 'Color', 'r')
plot3(linspace(a, b, 100),linspace(0, 0, 100),linspace(0, 0, 100),'Color', 'b','LineWidth', 1) % for x-axis
plot3(linspace(0, 0, 100), linspace(a, b, 100),linspace(0, 0, 100),'Color', 'b','LineWidth', 1) % for y-axis
plot3(linspace(0, 0, 100),linspace(0, 0, 100),linspace(a, b, 100),'Color', 'b','LineWidth', 1) % for z-axis

axis([a,b,a,b,a,b])
view(-30,10)
hold off

figure('name','Delta translation vectors')
scatter3(tx,ty,tz,...
        'MarkerEdgeColor','k',...
        'MarkerFaceColor',[0 .75 .75])
hold on

ta = linspace(t_tx, 0, 100);
tb = linspace(t_ty, 0, 100);
tc = linspace(t_tz, 0, 100);

a = -500;
b = 0;

plot3(ta, tb, tc,'LineWidth', 3, 'Color', 'r')
plot3(linspace(a, b, 100),linspace(0, 0, 100),linspace(0, 0, 100),'Color', 'b','LineWidth', 1) % for x-axis
plot3(linspace(0, 0, 100), linspace(a, b, 100),linspace(0, 0, 100),'Color', 'b','LineWidth', 1) % for x-axis
plot3(linspace(0, 0, 100),linspace(0, 0, 100),linspace(a, b, 100),'Color', 'b','LineWidth', 1) % for z-axis

axis([a,b,a,b,a,b])
view(-30,10)
hold off

