use <openscad-extra/src/countersink.scad>;

// mega needs 3mm of clearance below it
module arduino_mega_mount(){
    difference(){
        for(i=[-1:2:1])
        translate([0,7.5,(25+2.5)*i])
        cube([110,10,5], center=true);
        
        translate([-1.15,7.65,-.75])
        color("orange")
        arduino_mega_board(buffer=0.5);

        color("red"){
            // top mount holes
            for(i=[0:2])
            translate([40-40*i,12.5,30-2.5])
            rotate([-90,0,0])
            make_countersink(d1=2.5, d2=5, inner=20);
            
            // bottom mount holes
            for(i=[0:2])
            translate([40-40*i,5,-(30-2.5)])
            rotate([-90,0,0])
            make_countersink(d1=2.5, d2=5, inner=20);
        }
    }
    
}

module arduino_mega_board(buffer=0){
    // main board
    color("green")
    cube([99.2+buffer,1.7+buffer,53.4+buffer], center=true);
    
    // electronics bounding box
    color("red")
    translate([0,6,0])
    cube([99.2-3+2+buffer,1.7+10.5+buffer,53.4-3+0.5+buffer], center=true);
    
    // bottom elec bb
    color("red")
    translate([2-1.8,-2.86+1/2+0.02,0])
    cube([95+buffer,2+buffer,50.5+0.5+buffer], center=true);
}

//mega_offset_y = -4; // flush
//mega_offset_y = -4+3; // minimum clearance
mega_offset_y = -4+4; // minimum clearance + extra for screw

if(0){
color("gray")
translate([-109.5,mega_offset_y,100])
rotate([-90,0,0])
import("electronics/arduino-mega-2560/Arduino_MEGA2560.stl");

translate([-109.5+99.2/2,-1.7/2+mega_offset_y,100-53.4/2])
arduino_mega_board(buffer=0.5);
}

if(0)
color("blue")
translate([-58.75, -66-5+55+5+2.5, 4.5+70])
//pelvis_cross_support();
rotate([90,0,0])
import("printable/pelvis_cross_support.stl");

translate([-58.75, -66-5+55+5+2.5, 4.5+70])
arduino_mega_mount();
