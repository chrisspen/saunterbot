use <openscad-extra/src/rounded.scad>;
use <openscad-extra/src/countersink.scad>;

module torso_side(){
    difference(){
        color("green")
        cube([5,40,80-0.5], center=true);
        
        color("red")
        for(i=[-1:2:1])
        translate([0, -23*i, 0])
        rotate([0,90,0])
        rounded_cube_2d([70,20,20], r=4.99, $fn=100, center=true);
    
        // chassis mount holes
        color("blue")
        for(j=[-1:2:1])
        for(i=[0:3])
        translate([2.5,10*i-15,(-35-2.5)*j])
        rotate([0,90,0])
        make_countersink(d1=2.5, d2=5, inner=20, outer=20);
        
        // cross support mount holes
        color("red")
        for(j=[-1:2:1])
        for(i=[0:4])
        translate([2.5,10*j,10*i-20])
        rotate([0,90,0])
        make_countersink(d1=2.5, d2=5, inner=20, outer=20);
        
        // wire thrus
        color("blue")
        for(i=[0:2])
        translate([0,0,-22.5*i+22.5])
        scale([1,1,2])
        rotate([0,90,0])
        cylinder(d=10, h=100, center=true, $fn=100);
        
    }
    
}

rotate([0,-90,0])
torso_side();
