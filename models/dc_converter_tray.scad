//include <../settings.scad>;
//use <tray.scad>;
use <openscad-extra/src/countersink.scad>;

//TODO:bring in mount holes for power board, too far out

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

module mock_servo_board(){
    difference(){
        color("orange")
        cube([60, 15, 2], center=true);
            
        color("red")
        for(i=[-1:2:1])
        translate([(60/2-1.7)*i,0,0])
        cylinder(d=2.3, h=20, center=true, $fn=100);
    }
}

module make_servo_board_holes(height=20, d=2){
    offset_x = 60/2-1.7;
    translate([offset_x,0,0])
    cylinder(d=d, h=height, center=true, $fn=100);
    translate([-offset_x,0,0])
    cylinder(d=d, h=height, center=true, $fn=100);
}

module make_dc_converter_tray(){
    d = 2.5;
    height = 20;

    mount_x = 5*5;
    //mount_y = 5*4;
    mount_y = 25+5/2;
    mount_z = 0.25;
    
    servo_board_y = 14;

    thickness = 5*0.5;

    difference(){
        union(){
            cube([5*14, 60, thickness], center=true);
            //cube([80+5*2, 5*4, thickness], center=true);
            
            translate([0,-10,0])
            translate([0,0,1.5])
            make_dc_converter_holes(d=5, height=5);
        
            // servo board mount stubs
            translate([0,servo_board_y,0])    
            translate([0,0,1.5])
            make_servo_board_holes(d=5, height=5);
        }

        // mount holes for dc board
        translate([0,-10,0])
        make_dc_converter_holes(d=2);

        // mount holes for servo board
        translate([0,servo_board_y,0])
        make_servo_board_holes(d=2);
        
        // mount holes of tray to frame
        for(i=[0:6]) for(j=[-1:2:1])
        translate([mount_x - 10*i + 5, mount_y*j, 1.2])
        color("green")
        make_countersink();
        
        // wire cutout
        color("red")
        for(j=[-1:2:1])
        translate([37.5*j,0,0])
        hull()
        for(i=[-1:2:1])
        translate([0,18.75*i,0])
        cylinder(d=10+2.5, h=height*2, center=true);
    
        // center bulk cutout
        color("red")
        hull()
        for(j=[-1:2:1])
        translate([19*j,0,0])
        for(i=[-1:2:1])
        translate([0,18.75*i,0])
        cylinder(d=10+2.5, h=height*2, center=true);
        
    }// end diff
}

if(0){
    //mock servo power board
    //60mm x 15mm with 2.3mm holes inset by 1.7mm
    
    translate([0,14,10])
    mock_servo_board();

    //mock regulator
    color("brown")
    translate([0,-10,10]){
        cube([62, 27, 2], center=true);
    }
}

if(0)
color("blue")
translate([0,0,-5])
import("printable/pelvis_cross_support.stl");
make_dc_converter_tray();
