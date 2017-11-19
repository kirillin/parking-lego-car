L = 0.20;
PHI_MAX = %pi / 4;
w = 4.8 / 1;
KP1 = w^2;
KP2 = w^2;
KD1 = 2*w;
KD2 = 2*w;
path = get_absolute_file_path('control_1.sce');
scheme_name = path + 'control_1.zcos';
importXcosDiagram(scheme_name);
xcos_simulate(scs_m, 4);

plot2d(ref.values(:,1), ref.values(:,2));
plot2d(car.values(:,1), car.values(:,2), 5);
scf();
plot2d(car.time, car.values(:,3));
