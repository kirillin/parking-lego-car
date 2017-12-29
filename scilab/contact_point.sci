// for search of contact point of bottom arc and inclined line
function answ = contact_point(point_7, point_4, R)

    x_7 = point_7(1);
    y_7 = point_7(2);
    x_4 = point_4(1);
    y_4 = point_4(2);

    function [y] = f1(x)
        y(1) = (x(1) - x_7)^2 + (x(2) - y_7)^2 - R^2;
        y(2) = (x(1) - x_7)*(x_4 - x(1)) + (x(2) - y_7)*(y_4- x(2));
    endfunction

    answ = fsolve([x_7 + R/2, y_7 - R/2], f1);

endfunction
