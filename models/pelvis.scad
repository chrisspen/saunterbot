use <openscad-extra/src/countersink.scad>;
use <openscad-extra/src/strut.scad>;

use <motors_alturn.scad>;

module pelvis_motor_face_mount_holes(){
        // horizontal holes
        for(j=[-1:2:1])
        for(i=[0:9])
        translate([1.2,10*i-110/2+10,(60/2-5/2)*j])
        rotate([0,90,0])
        make_countersink(d1=2.5, d2=5, inner=10, outer=10);
        
        // vertical holes
        for(j=[-1:2:1])
        for(i=[0:4])
        translate([1.2,(110/2-5/2)*j,10*i-60/2+10])
        rotate([0,90,0])
        make_countersink(d1=2.5, d2=5, inner=10, outer=10);
}

module pelvis_motor_face(){
    difference(){
        union(){
            // servo mount face
            color("green")
            cube([2.5, 110, 60], center=true);
            
            // main bulkhead rim
            // horizontal bar
            color("red")
            for(i=[-1:2:1])
            translate([-5/2/2,0,(60/2-5/2)*i])
            cube([5, 110, 5], center=true);
            
            // vertical bar
            color("red")
            for(i=[-1:2:1])
            translate([-5/2/2,(110/2-5/2)*i,0])
            cube([5, 5, 60], center=true);
            
            // middle bar
            translate([-5/2/2,0,0])
            cube([5, 15, 60], center=true);
        }
        
        //color("blue"){
        
            color("purple")
            translate([1.25,18.5-2.5+10,0])
            rotate([0,90,0])
            rotate([0,0,90])
            make_motors_alturn_abrs_5314htg_hv_holes(axle=1, buffer=0.5, flip_holes=0);
        
            color("orange")
            translate([1.25,-18.5+2.5-10,0])
            rotate([0,-90,0])
            rotate([0,0,90])
            make_motors_alturn_abrs_5314htg_hv_holes(axle=1, buffer=0.5, flip_holes=1);
            
            // chassis mounting holes
            pelvis_motor_face_mount_holes();
            
            // wire thru
            for(j=[-1:2:1])
            translate([0,(100/2-10)*0,(60/2-10-7.5)*j])
            scale([1,1,2])
            rotate([0,90,0])
            cylinder(d=10, h=50, center=true, $fn=100);

        //}
    }//end diff
}

module pelvis_cross_support(){

    
    //cube([100,5,60], center=true);
    
    rotate([0,0,90]){
    
        difference(){
            color("blue")
            make_cross_plate(
                w=100,
                h=60,
                t=5,
                cross_type=1
            );

            // vertical holes
            for(j=[-1:2:1])
            for(i=[0:3])
            translate([2.5,(110/2-5/2-5)*j,10*i-60/2+10+5])
            rotate([0,90,0])
            make_countersink(d1=2.5, d2=5, inner=10, outer=10);

            // horizontal holes
            for(j=[-1:2:1])
            for(i=[0:8])
            translate([2.5,10*i-110/2+10+5,(60/2-5/2)*j])
            rotate([0,90,0])
            make_countersink(d1=2.5, d2=5, inner=10, outer=10);
            
            // vertical holes
            color("red")
            for(j=[-1:2:1])
            for(i=[0:4])
            translate([0,(110/2-5/2-5+7.5)*j,10*i-60/2+10])
            rotate([0,0,90*j])
            rotate([0,90,0])
            make_countersink(d1=2.5, d2=5, inner=11, outer=10);
        }
        
    }
    
}
