use <balancer.scad>;

for(i=[0:3])
translate([i*8,0,0])
balancer_gear_axle_top();

for(i=[0:1])
translate([i*8,8,0])
balancer_gear_axle_top(cutoff=1);
