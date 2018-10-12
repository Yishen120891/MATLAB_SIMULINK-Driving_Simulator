function norma=calc_distance(ind,x,y)
global route

x_route = route.centrex(ind);
y_route = route.centrey(ind);

norma=(x_route-x)^2+(y_route-y)^2;
norma=sqrt(norma);
Route=[x_route ; y_route];
Vehicule=[x;y];
norma=norm(Route-Vehicule);

