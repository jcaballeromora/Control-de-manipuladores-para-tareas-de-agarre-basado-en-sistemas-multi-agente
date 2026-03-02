clear
clc
close all

% POSPROCESAMIENTO DE EXPERIMENTO DE AGARRE %
data = readtable('13datos_experimento.xlsx');

% Tiempo
t = data.current_real_time;
dt = data.dt;

% Fuerza
fx1 = data.Fx_Fil1;     fx2 = data.Fx_Fil2;
xr1 = data.xr1;         xr2 = data.xr2;

% Posiciones Cartesianas EF - R1
x1 = data.px1; y1 = data.py1; z1 = data.pz1;

% Posiciones Cartesianas EF - R2
x2 = data.px2; y2 = data.py2; z2 = data.pz2;

% Defino Fuerza deseada
for i=1:length(t)
    fd(i)=20;
end

%% Iteraciones
figure()
plot(dt)
xlabel('Iteración')
ylabel('Tiempo de paso [s]')
grid on
title('Latencia')
%% Gráficas de Fuerza
figure()
plot(t, fx1,'b')
hold on
plot(t, fx2,'r')
hold on
plot(t,fd,'--g')
legend('F_{R1}', 'F_{R2}','F_d')
title('Fuerzas R1 , R2')
xlabel('Tiempo [s]')
ylabel('Fuerza [N]')
grid on

%% Gráficas de señal de control de Fuerza
figure()
plot(t, xr1,'b')
hold on
plot(t, xr2,'r')
legend('xr_{1}', 'xr_{2}')
title('Señales de control')
xlabel('Tiempo [s]')
ylabel('Controlador [m]')
grid on

%% Gráficias EF - R1
figure()
subplot(3,1,1)
plot(t, x1)
ylabel('Posicion x [m]')
xlabel('Tiempo [s]')
title('Posicion cartesiana EF R1 - x')
grid on
subplot(3,1,2)
plot(t, y1)
ylabel('Posicion y [m]')
xlabel('Tiempo [s]')
title('Posicion cartesiana EF R1 - y')
grid on
subplot(3,1,3)
plot(t, z1)
ylabel('Posicion z [m]')
xlabel('Tiempo [s]')
title('Posicion cartesiana EF R1 - z')
grid on

%% Gráficias EF - R2
figure()
subplot(3,1,1)
plot(t, x2,'r')
ylabel('Posicion x [m]')
xlabel('Tiempo [s]')
title('Posicion cartesiana EF R2 - x')
grid on
subplot(3,1,2)
plot(t, y2,'r')
ylabel('Posicion y [m]')
xlabel('Tiempo [s]')
title('Posicion cartesiana EF R2 - y')
grid on
subplot(3,1,3)
plot(t, z1,'r')
ylabel('Posicion z [m]')
xlabel('Tiempo [s]')
title('Posicion cartesiana EF R1 - z')
grid on

%% Gráficas de robot y objeto
plot3(x1,y1,z1,'ob')
hold on
plot3(x2,y2,z2,'or')

%% Animación R1
clc
figure()
pause(2)
n=length(t);
for i=1:n
%     clc

    plot3(x1(i),y1(i),z1(i),'og','MarkerSize',16)
%     hold on
%     plot3(x2(i),y2(i),z2(i),'ob','MarkerSize',16)
%     hold on
%     plot3([x1(i) x2(i)],[y1(i) y2(i)],[z1(i) z2(i)],'-.r')
%     hold on
%     plot3([r1_x(i) r5_x(i)],[r1_y(i) r5_y(i)],[r1_z(i) r5_z(i)],'-.r')
%     hold on
    % plot3(0,0,0,'*b')
    % hold on
    % plot3(1,0,0,'*b')
    grid on
    axis([-1,1,-1,1,0,1])
    title('Comportamiento')
    xlabel('Eje x [m]')
    ylabel('Eje y [m]')
    zlabel('Eje z [m]')
    legend('r1')
    pause(0.01);
    hold off
    fprintf('Tiempo de simulacion: %.2f\n',t(i))
end



%% Animación dos robots

% Simulación Condiciones 
% close all
clc
figure()
pause(2)
n=length(t);
for i=1:n
%     clc

    plot3(x1(i),y1(i),z1(i),'og','MarkerSize',16)
    hold on
    plot3(x2(i),y2(i),z2(i),'ob','MarkerSize',16)
    hold on
    plot3([x1(i) x2(i)],[y1(i) y2(i)],[z1(i) z2(i)],'-.r')
%     hold on
%     plot3([r1_x(i) r5_x(i)],[r1_y(i) r5_y(i)],[r1_z(i) r5_z(i)],'-.r')
%     hold on
    % plot3(0,0,0,'*b')
    % hold on
    % plot3(1,0,0,'*b')
    grid on
    axis([-1,1,-1,1,0,1])
    title('Comportamiento')
    xlabel('Eje x [m]')
    ylabel('Eje y [m]')
    zlabel('Eje z [m]')
    legend('r1','r2')
    pause(0.01);
    hold off
    fprintf('Tiempo de simulacion: %.2f\n',t(i))
end
