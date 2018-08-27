use <openscad-extra/src/countersink.scad>;

module leg_spring_hinge_extender(){
    bar_length = 35;
    bar_height = 5+5;
    
    difference(){
        union(){
            // main mass
            hull()
            for(i=[0:1])
            translate([0,(5/2 + (bar_height)/2)*i,0])
            cylinder(d=10, h=10, center=true, $fn=50);
            
            // mount bar
            translate([bar_length/2, bar_height - (bar_height-5)/2, -5/2])
            cube([bar_length, bar_height, 5+2], center=true);
        }
        
        // axle hole
        translate([0,0,2.5+5/2])
        make_countersink();
    
        // hub cutout
        color("red")
        translate([0,0,-5])
        cylinder(d=10.5, h=10, center=true, $fn=50);

        // mount holes
        color("red")
        for(i=[0:2])
        translate([10*i+10,7.5 + (bar_height)/2,-5/2])
        rotate([-90,0,0])
        make_countersink(inner=30);
    
    }
}

rotate([-90,0,0])
leg_spring_hinge_extender();
