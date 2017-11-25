kolor = color(255,128,0);

function alpha=plot_arc(x_c, y_c, x_s, y_s, x_f, y_f, R)
    alpha_1 = atan(y_s - y_c, x_s - x_c);
    disp(alpha_1)
    alpha_2 = atan(y_f - y_c, x_f - x_c);
     disp(alpha_2)
    delta_alpha = alpha_2 - alpha_1;
    if delta_alpha > %pi then
        delta_alpha = delta_alpha - 2*%pi;
    elseif delta_alpha < -%pi then
        delta_alpha = delta_alpha + 2*%pi;
    end
    disp(delta_alpha)
    alphas = alpha_1:sign(delta_alpha)*0.01:(alpha_1+delta_alpha);
    x = x_c + R*cos(alphas);
    y = y_c + R*sin(alphas);
    plot2d(x,y,kolor)
    alpha = alpha_2+%pi/2;
endfunction

//workabilty check only for cases when y_f != y_c2
function answ = contact_point(x_f, y_f, x_c2, y_c2, R)
    function [y] = f1(x)
        y(1) = (x(1) - x_f)^2 + (x(2) - y_f)^2 - R^2;
        y(2) = (x(1) - x_f)*(x_c2 - x(1)) + (x(2) - y_f)*(y_c2 - x(2));
    endfunction
    answ = fsolve([x_f+R/2, y_f-R/2], f1);
endfunction

///////////////////// BODY ////////////////////////////
x_s = 3;
y_s = 1;
x_c1 = 1;
y_c1 = 0;
x_c2 = 2.0;
y_c2 = 0;
delta_1 = 0.1;
delta_2 = 0.3;
d = 0.1;
R = 0.4;
plot(x_c1, y_c1,'g.');
plot(x_c2, y_c2,'g.')
plot(x_s, y_s,'g.')

x_f = x_c1 + delta_1;
y_f = y_c1 - d;
answ = contact_point(x_f, y_f + R, x_c2-delta_2, y_c2+delta_2, R)
x_t = answ(1);
y_t = answ(2);
alpha = plot_arc(x_f, y_f + R, x_f, y_f, x_t, y_t, R)
k = tan(alpha);
b = k*(y_t-x_t);
h = R / cos(alpha);
x2 = 1/k*(y_s - R - b + h);
y2 = y_s - R;
disp("--")
//plot(x2, y2,'go')
plot(3.1,1.1)
plot(0.9,-0.2)
plot2d([x_t:0.01:x2-R*sin(alpha)], k*[x_t:0.01:x2-R*sin(alpha)]+b,kolor)
plot_arc(x2, y2, x2-R*sin(alpha), y2+R*cos(alpha), x2, y_s, R)
plot2d([x2:0.01:x_s], y_s*ones([x2:0.01:x_s]), kolor)
xgrid()
