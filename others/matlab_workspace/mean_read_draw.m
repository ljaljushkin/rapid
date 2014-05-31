arr = csvread('mean_shift_rvec_and_tvec.txt');
m_arr = csvread('center_rvec_and_tvec.txt');
r_arr = csvread('ransac_rvec_and_tvec.txt');

rx = arr( : , 1 ); ry = arr( : , 2 ); rz = arr( : , 3 );
tx = arr( : , 4 ); ty = arr( : , 5 ); tz = arr( : , 6 );

r_rx = r_arr( : , 1 ); r_ry = r_arr( : , 2 ); r_rz = r_arr( : , 3 );
t_tx = r_arr( : , 4 ); t_ty = r_arr( : , 5 ); t_tz = r_arr( : , 6 );

m_rx = m_arr( : , 1 ); m_ry = m_arr( : , 2 ); m_rz = m_arr( : , 3 );
m_tx = m_arr( : , 4 ); m_ty = m_arr( : , 5 ); m_tz = m_arr( : , 6 );

a = -8;
b = 8;

figure('name','Delta rotation vectors')
scatter3(rx,ry,rz,...
        'MarkerEdgeColor','k',...
        'MarkerFaceColor',[.75 .75 0])   
hold on
    
ra = linspace(r_rx, a, 100);
rb = linspace(r_ry, b, 100);
rc = linspace(r_rz, a, 100);

plot3(ra, rb, rc, 'LineWidth', 3, 'Color', 'r')
plot3(linspace(a, b, 100),linspace(0, 0, 100),linspace(0, 0, 100),'Color', 'b','LineWidth', 1) % for x-axis
plot3(linspace(0, 0, 100), linspace(a, b, 100),linspace(0, 0, 100),'Color', 'b','LineWidth', 1) % for y-axis
plot3(linspace(0, 0, 100),linspace(0, 0, 100),linspace(a, b, 100),'Color', 'b','LineWidth', 1) % for z-axis

ma = linspace(m_rx, b, 100);
mb = linspace(m_ry, a, 100);
mc = linspace(m_rz, a, 100);
plot3(ma, mb, mc, 'LineWidth', 3, 'Color', 'g')

axis([a,b,a,b,a,b])
view(-30,10)
hold off

figure('name','Delta translation vectors')
scatter3(tx,ty,tz,...
        'MarkerEdgeColor','k',...
        'MarkerFaceColor',[0 .75 .75])
hold on

a = -500;
b = 200;

ta = linspace(t_tx, a, 100);
tb = linspace(t_ty, b, 100);
tc = linspace(t_tz, a, 100);

plot3(ta, tb, tc,'LineWidth', 3, 'Color', 'r')
plot3(linspace(a, b, 100),linspace(0, 0, 100),linspace(0, 0, 100),'Color', 'b','LineWidth', 1) % for x-axis
plot3(linspace(0, 0, 100), linspace(a, b, 100),linspace(0, 0, 100),'Color', 'b','LineWidth', 1) % for x-axis
plot3(linspace(0, 0, 100),linspace(0, 0, 100),linspace(a, b, 100),'Color', 'b','LineWidth', 1) % for z-axis

ma = linspace(m_tx, b, 100);
mb = linspace(m_ty, a, 100);
mc = linspace(m_tz, a, 100);
plot3(ma, mb, mc, 'LineWidth', 3, 'Color', 'g')


axis([a,b,a,b,a,b])
view(-30,10)
hold off

