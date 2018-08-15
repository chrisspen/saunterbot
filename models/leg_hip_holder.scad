use <openscad-extra/src/countersink.scad>;

module leg_hip_holder(show_axle=0){
    axle_offset_z = -22+0.2;
    
    difference(){
        hull(){
            cube([10, 100+10, 5], center=true);
            
            translate([0,0,axle_offset_z])
            cube([10, 30, 30], center=true);
        }
        
        // notch cutout
        color("orange")
        translate([-5/2,0,0])
        cube([5.5, 110, 5.5], center=true);

        // bottom brace cutout
        color("blue")
        translate([0,0,axle_offset_z-30/2+5/2])
        cube([15, 30, 5.5], center=true);

        // body mount holes
        color("red")
        for(i=[0:9])
        translate([0,10*i-45,0])
        rotate([0,90,0])
        cylinder(d=2.5, h=50, center=true, $fn=50);

        // bottom brace mount holes
        color("red")
        for(j=[-1:2:1])
        for(i=[0:2])
        translate([2.5*j,10-10*i,-41.5+5])
        rotate([0,0,0])
        cylinder(d=2.5, h=20, center=true, $fn=50);

        // mass cutouts
        color("red")
        for(i=[-1:2:1]){
            hull(){
            translate([0,15*i,-17])
            rotate([0,90,0])
            // big mass cutouts
            cylinder(d=20, h=50, center=true, $fn=50);
            
            translate([0,35*i,-9.5])
            rotate([0,90,0])
            // small mass cutouts
            cylinder(d=5, h=50, center=true, $fn=50);
            }
        }
        
        // end cutouts
        color("orange")
        for(i=[-1:2:1])
        translate([0,(50+2.5)*i,0])
        cube([30, 5.5, 5.5], center=true);

        // bottom mount mass
        color("green")
        translate([0,0,axle_offset_z])
        translate([-5/2,0,0])
        cube([5.5,8.6,8.6], center=true);

        // axle mounting hole
        translate([0,0,axle_offset_z])
        translate([5,0,0])
        rotate([0,90,0])
        make_countersink(d1=2.5, d2=5, outer=10, inner=40, $fn=25);

    }// end diff

    // mock axle
    if(show_axle)
    translate([0,0,axle_offset_z])        
    rotate([0,90,0])
    color("red")
    cylinder(d=1, h=100, center=true, $fn=50);
    
    
}

module leg_hip_axle(show_axle=0){
    axle_offset_z = -22+0.2;

    translate([0,0,axle_offset_z]){

        difference(){
            union(){
                difference(){
                    union(){
                        rotate([0,90,0]){
                            translate([0,0,-10/2+0.05]){
                                // bearing axle
                                translate([0,0,-11/2])
                                cylinder(d=7.9-0.5+0.5, h=11, center=true, $fn=50);
                                // leg axle
                                color("blue")
                                translate([0,0,-(11+7)/2])
                                cylinder(d=6, h=11+7, center=true, $fn=50);
                            }
                        }
                        // hip cap mount
                        color("purple")
                        translate([-(4+11+7)/2-10-3.5,0,0])
                        cube([4,4,4], center=true);
                    }
                    
                    // screw hole
                    rotate([0,90,0])
                    color("red")
                    cylinder(d=2, h=100, center=true, $fn=50);
                }
                
                // bottom mount mass
                color("green")
                translate([-5/2,0,0])
                cube([5,8,8], center=true);
            }
        
            // axle mounting hole
            translate([5,0,0])
            rotate([0,90,0])
            make_countersink(d1=2.5, d2=5, outer=10, inner=15, $fn=25);
        }
    }
    
}

module leg_hip_axle_round(show_axle=0){
    axle_offset_z = -22+0.2;

    translate([0,0,axle_offset_z]){

        difference(){
            union(){
                difference(){
                    union(){
                        rotate([0,90,0]){
                            translate([0,0,-10/2+0.05]){
                                // bearing axle
                                translate([0,0,-11/2])
                                cylinder(d=7.9-0.5+0.5, h=11, center=true, $fn=50);
                                // leg axle
                                color("blue")
                                translate([0,0,-(11+7)/2])
                                cylinder(d=6, h=11+7, center=true, $fn=50);
                            }
                        }
                        // hip cap mount
                        if(0)
                        color("purple")
                        translate([-(4+11+7)/2-10-3.5,0,0])
                        cube([4,4,4], center=true);
                        
                        color("purple")
                        translate([-(4+11+7)/2-10-3.5-.45,0,0])
                        rotate([0,90,0])
                        cylinder(d=3, h=4, center=true, $fn=100);
                    }
                    
                    // screw hole
                    rotate([0,90,0])
                    color("red")
                    cylinder(d=2, h=100, center=true, $fn=50);
                }
                
                // bottom mount mass
                if(1)
                color("green")
                translate([-5/2,0,0])
                cube([5,8,8], center=true);
                
            }
        
            // axle mounting hole
            translate([5,0,0])
            rotate([0,90,0])
            make_countersink(d1=2.5, d2=5, outer=10, inner=15, $fn=25);
        }
    }
    
}


if(1)
rotate([0,0,45])
rotate([0,90,0])
leg_hip_holder();

if(0)
rotate([0,0,45])
rotate([0,90,0])
leg_hip_axle();
