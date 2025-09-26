figure;
hold on;
axis equal;
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3);

% クワッドローターのアーム長
arm_length = 0.2; 
prop_radius = 0.05;

% アームの座標 (X, Y, Z)
arm1_x = [-arm_length, arm_length];
arm1_y = [0, 0];
arm1_z = [0, 0];

arm2_x = [0, 0];
arm2_y = [-arm_length, arm_length];
arm2_z = [0, 0];

% アームのプロット
plot3(arm1_x, arm1_y, arm1_z, 'k', 'LineWidth', 2);
plot3(arm2_x, arm2_y, arm2_z, 'k', 'LineWidth', 2);

% プロペラの位置
prop_positions = [
    arm_length, 0, 0;
   -arm_length, 0, 0;
    0, arm_length, 0;
    0, -arm_length, 0];

% プロペラの描画
theta = linspace(0, 2*pi, 50);
for i = 1:4
    x = prop_positions(i,1) + prop_radius * cos(theta);
    y = prop_positions(i,2) + prop_radius * sin(theta);
    z = zeros(size(theta));
    plot3(x, y, z, 'b', 'LineWidth', 1.5);
end

% % 軸の描画
quiver3(0,0,0,0.1,0,0,'r','LineWidth',2); % X軸
quiver3(0,0,0,0,0.1,0,'g','LineWidth',2); % Y軸
quiver3(0,0,0,0,0,0.1,'b','LineWidth',2); % Z軸

title('Quadcopter 3D Model');
