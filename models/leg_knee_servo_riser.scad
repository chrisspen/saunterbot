use <gears/parametric_involute_gear_v5.0.scad>;
use <screw.scad>;
use <motors_alturn.scad>;
use <openscad-extra/src/torus.scad>;

module leg_knee_servo_riser_divets(buffer=0){
        // screw mount holes divets
        color("blue")
        translate([0,0,-1/2])
        for(j=[0:3])
        for(i=[-1:2:1])
        rotate([0,0,45*j])
        translate([0,14/2*i,-5])
        rotate([180,0,0])
        //screw_2sd_5l_4hd(extra_length=5, shaft_buffer=1);
        cylinder(d1=2+buffer, d2=3+buffer, h=1, center=true, $fn=50);
}

module leg_knee_servo_riser(){
    
    difference(){
        
        // main mass
        //translate([0,0,-5/2])
        cylinder(d=20, h=15, center=true, $fn=100);

        // screw mount holes
        color("blue")
        translate([0,0,-10])
        for(j=[0:3])
        for(i=[-1:2:1])
        rotate([0,0,45*j])
        translate([0,14/2*i,-5])
        rotate([180,0,0])
        screw_2sd_5l_4hd(extra_length=50);

        // servo horn center hole
        color("blue")
        translate([0,0,0])
        cylinder(d=8, h=50, center=true, $fn=100);
     
        //placed so a 5mm long, 2mm d screw can attach to it
        if(1)
        color("red")
        translate([0,0,5/2])
        translate([0,0,6.25])
        rotate([180,0,0])
        make_motors_alturn_abrs_5314htg_hv_horn(buffer=0.5);
    
        // divit cutouts
        if(1)
        translate([0,0,-5/2])
        translate([0,0,+0.9])
        leg_knee_servo_riser_divets(buffer=0);
        
    }
}

module leg_knee_servo_spindle(){
    //C = pi*d => d = C/pi
    //130mm extended
    //94mm retracted
    //36mm full pull
    //pi = 3.141592653589793;
    //spindle_d = 36/pi;
    screw_offset = 6;
    
    difference(){
    
        union(){
            difference(){
                union(){
                    // smaller main mass
                    if(0)
                    translate([0,0,+5/2])
                    cylinder(d=20, h=5, center=true, $fn=50);
                    
                    // main spindle mass
                    translate([0,0,+5/2-5])
                    cylinder(d=25, h=5, center=true, $fn=100);
                        
                    // divit nubs
//                    translate([0,0,10+1-5])
//                    rotate([0,0,0])
//                    leg_knee_servo_riser_divets(buffer=-0.5);
                    
                                        
                    color("green")
                    translate([0,0,-2])
                    cylinder(d=27, h=2, $fn=50);
                    
                    color("green")
                    translate([0,0,-5])
                    cylinder(d=27, h=2, $fn=50);
                }

                // servo horn center hole
                color("blue")
                translate([0,0,0])
                cylinder(d=8, h=50, center=true, $fn=100);
                
                // wire cutout
                color("green")
                translate([0,0,-5/2])
                torus(r1=1, r2=25/2+1/2);
    
                // screw mount holes
                if(1)
                color("blue")
                translate([0,0,2.5])
                for(j=[0:3])
                for(i=[-1:2:1])
                rotate([0,0,45*j])
                translate([0,14/2*i,-5])
                rotate([180,0,0])
                screw_2sd_5l_4hd(extra_length=50, head_buffer=.5);
                
                if(1)
                rotate([0,0,-45/2]){
                        
                    // cutout for crimp mount screw
                    color("red")
                    rotate([0,0,0])
                    translate([-12.5,0,-5/2])
                    cube([6,20,5], center=true);
                    
                    // cutout for crimp mount screw
                    color("red")
                    rotate([0,0,0])
                    translate([-12.5,0-2.5,-5/2])
                    cube([6+3,11,5], center=true);
                    
                    // cutout for crimp
                    if(0)
                    color("red")
                    rotate([0,0,30])
                    translate([-12.5,0,-5/2])
                    cube([6,10,5], center=true);
                    
                }
                
                if(0)
                rotate([0,0,-45/2]){
                        
                    // cutout for wire axle hole
                    translate([-10-5/2,0,-5/2])
                    rotate([0,90,0])
                    cylinder(d=2, h=10, center=true, $fn=50);
                    
                }
           
           
            }// end diff

            rotate([0,0,-45/2]){
                    
                // mass for wire axle mount
                if(1)
                translate([-10-5/2+2+1,screw_offset,-5/2])
                rotate([0,90,0])
                cylinder(d=3, h=2.5, center=true, $fn=50);
                
            }
        }
    
        rotate([0,0,-45/2]){
                
            // cutout for wire axle hole
            if(1)
            translate([-10-5/2+5,screw_offset,-5/2])
            rotate([0,90,0])
            cylinder(d=2, h=10, center=true, $fn=50);
            
        }
        
        
    }// end diff
        
}

if(0)
leg_knee_servo_riser();

if(1)
translate([0,0,-15-10])
leg_knee_servo_spindle();

if(1)
color("orange")
translate([0,0,-50])
cylinder(d=27, h=1, $fn=50);
