use <openscad-extra/src/rounded.scad>;
use <openscad-extra/src/countersink.scad>;
use <openscad-extra/src/nut.scad>;

use <motors_alturn.scad>;
use <screw.scad>;
use <sensor_rotary.scad>;

module leg_upper(show_axle=1){
    
    difference(){
        // main mass
        translate([-100/2,-30/2,-5/2])
        rounded_cube_2d([100, 30, 5], r=14.9, center=false, $fn=100);
        
        //placed so a 5mm long, 2mm d screw can attach to it
        color("red")
        translate([100/2-15,0,3.75])
        rotate([180,0,0])
        make_motors_alturn_abrs_5314htg_hv_horn(buffer=0.5);
        
        //if a screw with 2mm head thickness and 5mm shaft thickness needs to go through 2.5mm of horn and 5mm of leg
        //then it needs to be countersunk by 2.5mm
        // screws are 14mm diameter apart
        
        color("blue")
        translate([100/2-15,0,2.5+2.5])
        for(j=[0:3])
        for(i=[-1:2:1])
        rotate([0,0,45*j])
        translate([0,14/2*i,-5])
        rotate([180,0,0])
        screw_2sd_5l_4hd(extra_length=50);
        
        // servo horn center hole
        color("blue")
        translate([100/2-15,0,0])
        cylinder(d=7, h=50, center=true, $fn=100);

        // bottom most hole
        color("blue")
        translate([-100/2+10,0,10])
        cylinder(d=4.5, h=50, center=true, $fn=50);
        //make_countersink(d1=2.5, d2=5, outer=10, inner=20, $fn=25);
        
        // middle hole
        color("blue")
        translate([-100/2+10+20,0,10])
        cylinder(d=3, h=50, center=true, $fn=50);
        //make_countersink(d1=2.5, d2=5, outer=10, inner=20, $fn=25);

        // sensor slot cutout
        mirror([0,0,1])
        scale([1,1,1.005])
        translate([-100/2+10,0,-1.3])
        rotate([0,0,45])
        rotate([0,0,0])
        make_rotary_sensor_3382G_bb(buffer=0.5, hole=0, extra_length=30);
        
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
        for(i=[0:4])
        color("green")
        translate([i*5,j*5-10,0])
        cylinder(d=2.5, h=10, center=true, $fn=50);
        
    }
    
    if(show_axle)
    color("green")
    translate([100/2-15,0,2.5+2.5])
    translate([0,0,-5])
    rotate([180,0,0])
    cylinder(d=1, h=100, center=true, $fn=50);

}

leg_upper();
