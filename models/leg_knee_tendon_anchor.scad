use <openscad-extra/src/countersink.scad>;
use <openscad-extra/src/nut.scad>;

module leg_knee_tendon_anchor_slot(width=3, width_buffer=0, height=1){
    linear_extrude(height=30)
    polygon([
        [(width+width_buffer)/2,0],
        [(width+width_buffer)/2/4,height],
        [-(width+width_buffer)/2/4,height],
        [-(width+width_buffer)/2,0]]);
}

module leg_knee_tendon_anchor_top_base(buffer=0){
    
    // track cutout
    translate([-1.5,0,-1+.15])
    rotate([0,90,0])
    rotate([0,0,90])
    leg_knee_tendon_anchor_slot(width_buffer=buffer*2, height=2);
    
    // slot cutout
    translate([20/2-1.5,0,2.5+5/2])
    cube([20, 4+buffer*2, 10], center=true);
    
}

module leg_knee_tendon_anchor_top(){
    
    wire_x = 12+5;
    wire_z = 6-0.1;
    wire_d = 6.46;
    
    difference(){
        union(){
            intersection(){
                
                // track mass
                if(1)
                translate([0,0,1.5])
                leg_knee_tendon_anchor_top_base(buffer=-0.25);
                
                // bounding box
                if(1)
                color("blue")
                translate([5+2,0,4.125])
                cube([15,10,10], center=true);
            }

            // top mass
            color("blue")
            translate([5+2,0,5.5+.25+.05+0.15/2])
            cube([15,5,6.5], center=true);
                
            // tendon mount mass
            hull(){
                translate([wire_x,0,wire_z])
                rotate([90,0,0])
                cylinder(d=wire_d, h=5, center=true, $fn=50);
                
                translate([6,0,wire_z])
                rotate([90,0,0])
                cylinder(d=wire_d, h=5, center=true, $fn=50);
            }
            
        }
        
        // small hux nut cutout
        if(0)
        color("red")
        hull()
        for(i=[0:1])
        translate([2,i*5,5])
        rotate([0,90,0])
        rotate([0,0,30])
        make_hex_nut(w=5, h=2, d=0);

        // hex nut cutout
        hull(){
            for(i=[0:3])
            translate([4,i,5])
            rotate([0,90,0])
            rotate([0,0,60/2])
            make_hex_nut(w=5.1, h=4, d=0);
        }

        // axle hole
        translate([0,0,5])
        rotate([0,90,0])
        cylinder(d=2.5, h=100, center=true, $fn=50);
            
        // wire slot cutout
        color("red")
        translate([wire_x,0,wire_z])
        rotate([90,0,0])
        cylinder(d=8, h=2, center=true, $fn=50);
    
        // wire screw hole
        color("red")
        translate([wire_x,0,wire_z])
        rotate([90,0,0])
        cylinder(d=2, h=20, center=true, $fn=50);

    }
    
}

module leg_knee_tendon_anchor_bottom(){
    
    axle_offset_z = -.5+3;
    axle_mass_h = 7.5+2;
    
    difference(){
        union(){
            // main mass
            if(0)
            translate([0,0,-2.5/2])
            cube([25, 5, 5+2.5], center=true);
            
            // axle mount mass
            color("blue")
            translate([-10/2-5/2-2.5/2+1-5/2, 0, axle_mass_h/2/2+axle_offset_z-1.5])
            cube([12.5-5, 5, axle_mass_h], center=true);
            
            if(1)
            hull()
            translate([0,0,-2.5])
            for(i=[-1:2:1])
            translate([2.5*4*i,0,0])
            rotate([90,0,0])
            cylinder(d=8, h=5, center=true, $fn=50);
        }
        
        // mount holes
        translate([0,0,-2.5])
        for(i=[-1:2:1])
        color("red")
        translate([2.5*4*i,-2.6,0])
        rotate([90,0,0])
        make_countersink(d1=3);
        
        // flattening cutout
        if(0)
        translate([7,0,5+5/2])
        cube([25, 6, 10], center=true);
        
        // hex nut cutout
        if(0)
        translate([2,0,-4+axle_offset_z])
        hull(){
            for(i=[0:3])
            translate([-7.5,i,5+.5])
            rotate([0,90,0])
            rotate([0,0,60/2])
            make_hex_nut(w=5.1, h=4, d=0);
        }
    
        // main axle cutout
        translate([-14,0,1.5+axle_offset_z])
        rotate([0,-90,0])
        //make_countersink(d1=3, inner=40);
        cylinder(d=2.5, h=40, center=true, $fn=50);
        
        translate([-5,0,.5]){
            //color("red")
            leg_knee_tendon_anchor_top_base();
        }
            
    }
    
}

if(1)
//color("green")
translate([0,0,-1])
leg_knee_tendon_anchor_top();

leg_knee_tendon_anchor_bottom();

color("green")
translate([0,0,1+3])
rotate([0,90,0])
cylinder(d=1, h=100, center=true, $fn=50);

if(0)
color("red")
for(i=[0:10])
translate([5*i-20,0,0])
rotate([90,0,0])
cylinder(d=1, h=100, center=true, $fn=50);
