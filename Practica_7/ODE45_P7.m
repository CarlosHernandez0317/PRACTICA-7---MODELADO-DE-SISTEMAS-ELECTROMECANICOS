Ra = 2;
Kt = 0.01;
b  = 0.0012;
La = 0.023;
Ke = 0.01;
J  = 0.001;
va = 24;

% Condiciones iniciales
ia0 = 0;
w0 = 0;    
theta0 = 0;
x0 = [ia0; w0; theta0];

% Tiempo de simulación
tspan = [0 2];

f = @(t,x) [
    (-Ra*x(1) - Ke*x(2) + va) / La;
    (Kt*x(1) - b*x(2)) / J;
    x(2)
];

% Integrar con ode45
opts = odeset('RelTol',1e-6,'AbsTol',1e-8);
[t, x] = ode45(f, tspan, x0, opts);

ia    = x(:,1);
w     = x(:,2);
theta = x(:,3);

rpm = w * 60 / (2*pi);

% Graficas
figure('Name','Respuesta del motor DC','NumberTitle','off','Units','normalized','Position',[0.1 0.2 0.6 0.6]);

subplot(3,1,1);
plot(t, ia, 'LineWidth',1.3);
grid on;
ylabel('Corriente i_a (A)');
title('Corriente de armadura');

subplot(3,1,2);
plot(t, w, 'LineWidth',1.3);
hold on;
yyaxis right
plot(t, rpm, '--','LineWidth',1.0);
yyaxis left
grid on;
ylabel('Velocidad \omega (rad/s)');
xlabel('Tiempo (s)');
legend('\omega (rad/s)','rpm','Location','best');
title('Velocidad angular');

subplot(3,1,3);
plot(t, theta, 'LineWidth',1.3);
grid on;
ylabel('Posición \theta (rad)');
xlabel('Tiempo (s)');
title('Posición angular');

fprintf('Valores al final de la simulación (t = %.3f s):\n', t(end));
fprintf('  Corriente ia = %.6f A\n', ia(end));
fprintf('  Velocidad w = %.6f rad/s (%.2f rpm)\n', w(end), rpm(end));
fprintf('  Posición theta = %.6f rad\n', theta(end));

A = [ -Ra/La,  -Ke/La, 0;
       Kt/J,   -b/J,   0;
       0,       1,     0 ];
B = [ 1/La; 0; 0 ];
C = eye(3);
D = zeros(3,1);
