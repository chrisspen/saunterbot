module leg_hip_tendon_retainer(){
    hood_offset_x = 3;
    hood_height = 8;
    hood_gap = 0.5;

    difference(){
        union(){
            translate([0,0,3/2])
            cylinder(d=5, h=3, center=true, $fn=100);

            hull(){
                translate([0,0,2/2])
                cylinder(d=12, h=2, center=true, $fn=100);
                translate([hood_offset_x,0,2/2])
                cylinder(d=12, h=2, center=true, $fn=100);
            }
            
            difference(){
                translate([hood_offset_x,0,hood_height/2])
                cylinder(d=12, h=hood_height, center=true, $fn=100);
                translate([0,0,(hood_height+0.1)/2+2])
                scale([1,3,1])
                cylinder(d=12+hood_gap, h=hood_height+0.1, center=true, $fn=100);
            }
        }
        
        cylinder(d=2, h=10, center=true, $fn=100);
    
        translate([0,0,-20/2+1])
        cube([20,20,20], center=true);
    }
}

leg_hip_tendon_retainer();
