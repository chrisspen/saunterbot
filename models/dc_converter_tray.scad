//include <../settings.scad>;
//use <tray.scad>;
use <openscad-extra/src/countersink.scad>;

module make_dc_converter_holes(height=20, d=3){
    offset_x = 53.5/2+3/2;
    offset_y = 21/2;
    translate([-offset_x,offset_y,0])
    cylinder(d=d, h=height, center=true, $fn=100);
    translate([-offset_x,-offset_y,0])
    cylinder(d=d, h=height, center=true, $fn=100);
    translate([offset_x,offset_y,0])
    cylinder(d=d, h=height, center=true, $fn=100);
    translate([offset_x,-offset_y,0])
    cylinder(d=d, h=height, center=true, $fn=100);
}


module make_dc_converter_tray(){
    d = 2.5;
    height = 20;

    mount_x = 5*5;
    mount_y = 5*3.5;
    mount_z = 0.25;

    thickness = 5*0.5;

    difference(){
        union(){
            cube([5*14, 40, thickness], center=true);
            //cube([80+5*2, 5*4, thickness], center=true);
    
            translate([0,0,1.5])
            make_dc_converter_holes(d=5, height=5);
        }

        // mount holes for dc board
        make_dc_converter_holes(d=2);

        // mount holes of tray to frame
        for(i=[0:6]) for(j=[-1:2:1])
        translate([mount_x - 10*i + 5,mount_y*j,1.2])
        color("green")
        make_countersink();
        
        // wire cutout
        color("red")
        for(j=[-1:2:1])
        translate([35*j,0,0])
        hull()
        for(i=[-1:2:1])
        translate([0,5*i,0])
        cylinder(d=10+2.5, h=height*2, center=true);
    
        // center bulk cutout
        color("red")
        hull()
        for(j=[-1:2:1])
        translate([15*j,0,0])
        for(i=[-1:2:1])
        translate([0,5*i,0])
        cylinder(d=10+2.5, h=height*2, center=true);
        
    }// end diff
}

make_dc_converter_tray();
