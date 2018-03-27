use <openscad-extra/src/rounded.scad>;
use <openscad-extra/src/countersink.scad>;
use <openscad-extra/src/nut.scad>;

use <motors_alturn.scad>;
use <screw.scad>;
use <sensor_rotary.scad>;

module leg_lower(){
    
    difference(){
        // main mass
        translate([-100/2,-30/2,-5/2])
        rounded_cube_2d([100, 30, 5], r=14.9, center=false, $fn=100);
        
        // bottom most hole
        color("blue")
        translate([-100/2+10,0,10])
        cylinder(d=3, h=50, center=true, $fn=50);
        //make_countersink(d1=2.5, d2=5, outer=10, inner=20, $fn=25);
        
        // middle hole
        color("blue")
        translate([-100/2+10+20,0,10])
        cylinder(d=3, h=50, center=true, $fn=50);
        //make_countersink(d1=2.5, d2=5, outer=10, inner=20, $fn=25);
        
        // spring cutout middle
        color("red")
        translate([-20,0,0])
        rotate([0,0,45])
        translate([-10/2,-10/2,-2.5/2])
        rounded_cube_2d([50, 50, 2.5], r=10/2, center=false, $fn=100);
        
        // spring cutout outer
        difference(){
            color("red")
            translate([-20,0,0])
            rotate([0,0,45])
            translate([-10/2,-10/2,-10/2])
            rounded_cube_2d([50, 50, 10], r=10/2, center=false, $fn=100);
            color("blue")
            translate([-20,0,0])
            cylinder(d=15, h=200, center=true, $fn=100);
        }
        
        // mount holes
        for(j=[0:4])
        for(i=[0:8])
        color("green")
        translate([i*5,j*5-10,0])
        cylinder(d=2.5, h=10, center=true, $fn=50);
        
        for(i=[0:8])
        color("green")
        translate([-i*5,0*5-10,0])
        cylinder(d=2.5, h=10, center=true, $fn=50);
        
    }
    
    
    
}

leg_lower();
