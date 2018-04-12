use <balancer.scad>;

balancer_top_plate();

for(j=[-1:2:1])
for(i=[-1:2:1])
translate([50*i,50*j,0.4/2-5/2])
cylinder(d=10, h=0.4, center=true);