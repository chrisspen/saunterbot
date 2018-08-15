use <leg_spring2.scad>;

difference(){
    translate([0,0,4/2-0.125])
    rotate([45+180,0,0])
    make_leg_spring_upper(holes=1);

    translate([0,0,-100/2])
    cube([100,100,100], center=true);
}
