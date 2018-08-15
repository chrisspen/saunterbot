use <gears/parametric_involute_gear_v5.0.scad>;
use <screw.scad>;
use <motors_alturn.scad>;
use <openscad-extra/src/wedge.scad>;

module leg_hip_gear_main_herringbone_gear(teeth=51){
    
    bore_d = 7;
    //teeth = 51;//17;
    pressure_angle=30;
    twist=200;
    height=2.5;
    
    for(i=[0:1])
    mirror([0,0,i])
    gear (
        number_of_teeth=teeth,
        //circular_pitch=700-295,
        circular_pitch=250,
        pressure_angle=pressure_angle,
        clearance = 0.2,
        gear_thickness = height,
        rim_thickness = height,
        rim_width = 5,
        hub_thickness = height,
        hub_diameter=10,
        bore_diameter=bore_d,
        circles=0,
        twist=twist/teeth);

}

module leg_hip_gear_main(teeth=51){
    
    //bb
    if(0)
    translate([0,0,-30]){
        color("red")
        translate([0,0,100])
        cylinder(d=20, h=10, center=true);
        
        color("blue")
        cylinder(d=30, h=10, center=true);
    }
    

    intersection(){
        difference(){
            rotate([0,0,0])
            union(){
                leg_hip_gear_main_herringbone_gear(teeth=teeth);
                translate([0,0,-5])
                leg_hip_gear_main_herringbone_gear(teeth=teeth);
            }

            // screw mount holes
            color("blue")
            translate([0,0,-10])
            for(j=[0:3])
            for(i=[-1:2:1])
            rotate([0,0,45*j])
            translate([0,14/2*i,-5])
            rotate([180,0,0])
            screw_2sd_5l_4hd(extra_length=50);

            // bearing cutout
            color("green")
            translate([0,0,-4])
            cylinder(d=22.25, h=7.1, center=true, $fn=50);
    
            // servo horn center hole
            color("blue")
            translate([0,0,0])
            cylinder(d=9.5, h=50, center=true, $fn=100);
         
            //placed so a 5mm long, 2mm d screw can attach to it
            if(0)
            color("red")
            translate([0,0,3.75])
            rotate([180,0,0])
            make_motors_alturn_abrs_5314htg_hv_horn(buffer=0.5);
            
            // oval mass cutout
            color("red")
            translate([0,21+2,0])
            scale([1.8,1,1])
            cylinder(d=15, h=50, center=true, $fn=50);
         
            
        }
        
        translate([0,0,10])
        color("red"){
            wedge(r=50, h=50, a=120);
            cylinder(d=30, h=50, $fn=50, center=true);
        }
        
    }
    
    
}

rotate([0,180,0])
leg_hip_gear_main(teeth=51);
//leg_hip_gear_main(teeth=34);
