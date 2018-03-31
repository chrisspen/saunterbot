use <openscad-extra/src/countersink.scad>;

module make_motors_alturn_abrs_5314htg_hv_horn(buffer=0){

    // main mass
    cylinder(d=20+buffer, h=2+buffer, center=true, $fn=100);
    
    // bottom side mass
    translate([0,0,-1/2-1])
    cylinder(d=9+buffer, h=1+buffer, center=true, $fn=100);
    
    // top side mass
    translate([0,0,+1/2+1])
    cylinder(d=10+buffer, h=1+buffer, center=true, $fn=100);
}

module make_motors_alturn_abrs_5314htg_hv_holes(buffer=0, axle=0, flip_holes=1){

        // bottom mounting holes
        color("red")
        for(i=[-1:2:1])
        translate([(32/2-2.5)*i,43/2-3,0])
        //cylinder(d=2, h=100, center=true, $fn=100);
        rotate([0,180*flip_holes,0])
        make_countersink(d1=2, d2=4, inner=50, outer=50);
        
        // middle mounting holes
        color("red")
        for(i=[-1:2:1])
        translate([(32/2-2.5)*i,43/2-3-27,0])
        //cylinder(d=2, h=100, center=true, $fn=100);
        rotate([0,180*flip_holes,0])
        make_countersink(d1=2, d2=4, inner=50, outer=50);
        
        if(axle){                
            // axle centerpoint
            color("red")
            translate([0,-43/2+12,0])
            cylinder(d=22, h=200, center=true, $fn=100);
        }
        
}

module make_motors_alturn_abrs_5314htg_hv(show_axle_reference=1){
    difference(){
        // main body bounding box
        cube([32, 43, 32.5], center=true);
        
        make_motors_alturn_abrs_5314htg_hv_holes();
    }
    
    // axle centerpoint
    if(show_axle_reference)
    color("red")
    translate([0,-43/2+12,0])
    cylinder(d=1, h=100, center=true, $fn=100);
}

make_motors_alturn_abrs_5314htg_hv(show_axle_reference=0);

//make_motors_alturn_abrs_5314htg_hv_horn();
