use <balancer.scad>;

balancer_bottom_plate();

for(j=[-1:2:1])
for(i=[-1:2:1])
translate([55*i,50*j,0.4/2-5/2-37])
cylinder(d=10, h=0.4, center=true);
