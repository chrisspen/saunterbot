use <openscad-extra/src/countersink.scad>;

module foot_mock(){
    difference(){
        union(){
            cube([80, 10, 5], center=true);
            
            if(0)
            difference(){
                color("blue")
                scale([1,1,.6])
                translate([0,0,0])
                rotate([90,0,0])
                cylinder(d=80, h=10, center=true, $fn=50);
                
                // cylinder flat cutout
                if(0)
                translate([0,0,-100/2+5/2])
                cube([100, 100, 100], center=true);
            }
                    
            // pivot hub mass
            for(j=[0:1])
            mirror([j,0,0])
            hull()
            for(i=[0:1])
            translate([(-30/2-10/2/2-.5),0,5/2+5*i])
            rotate([0,90,0])
            cylinder(d=10, h=5, center=true, $fn=50);
        }
        
        // slot cutout
        if(0)
        translate([0,0,50/2+5/2-0.25])
        cube([100, 5.5, 50], center=true);

    
        // bottom mount holes
        color("red")
        for(i=[0:7])
        translate([10*i-35,0,-7])
        rotate([180,0,0])
        make_countersink(d1=2.5, d2=5, inner=10);
        
        // side mount holes
        color("red")
        for(i=[0:6])
        translate([10*i-30,-10,0])
        rotate([90,0,0])
        make_countersink(d1=2.5, d2=5, inner=20);
        
        // pivot holes
        color("red")
        for(i=[0:1])
        mirror([i,0,0])
        translate([-20.5,0,5/2+5])
        rotate([0,-90,0])
        make_countersink(d1=3, d2=5, inner=20);
    }
    
    // mounting holes
    if(0)
    color("red")
    for(i=[0:2])
    translate([11.2*i-11.2,5,17.5+10])
    rotate([-90,0,0])
    make_countersink(d1=2.5, d2=5, inner=20);
    
    
    
    if(0)
    translate([0,10,50/2+5/2])
    cube([30, 20, 50], center=true);
    
}

module foot_mock_ankle(){
    
    difference(){
        
        // main mass
        if(1)
        hull()
        for(i=[0:1])
        translate([0,0,(20+2.5)*i])
        rotate([0,90,0])
        cylinder(d=10-0.5, h=30, center=true, $fn=50);
        
        // mount axle hole
        rotate([0,90,0])
        cylinder(d=2.5, h=100, center=true, $fn=50);
        
        // mounting holes
        translate([0,0,-10+2.5])
        color("red")
        for(i=[0:2])
        translate([11.2*i-11.2, 5-.2, 17.5+10])
        rotate([-90,0,0])
        make_countersink(d1=2.5, d2=5, inner=20);
        
        // slot cutout
        if(1)
        translate([0,0,50/2+5/2+5/2])
        cube([100, 5.5, 50], center=true);
        
    }// end diff
}

module foot_mock_toe(major_length=80, minor_length=10, width=20, height=5){
    difference(){
        //color("red")
        linear_extrude(height=height)
        polygon([[-major_length/2,0], [major_length/2,0], [minor_length/2,width], [-minor_length/2,width]]);
        
        translate([0,5/2,5/2])
        cube([30.5,5.5,5.5], center=true);
        
        // side mount holes
        for(j=[0:1])
        mirror([j,0,0])
        color("red")
        for(i=[0:1])
        translate([10*i-30,4,5/2])
        rotate([-90,0,0])
        make_countersink(d1=3, d2=5, inner=20);
    
        // middle mount hole
        color("red")
        translate([0,20,5/2])
        rotate([-90,0,0])
        make_countersink(d1=3, d2=5, inner=20);

    }
}

module foot_mock_toe_spring(thickness=.4*2, length=20){

    difference(){
        union(){
            translate([0,thickness/2,length/2])
            cube([30, thickness, length], center=true);

            translate([0,5/2,5/2])
            cube([30, 5, 5], center=true);
        }
        
        // side mount holes
        color("red")
        for(i=[0:6])
        translate([10*i-30,5,5/2])
        rotate([-90,0,0])
        make_countersink(d1=3, d2=5, inner=20);
    }
}

if(1)
color("orange")
translate([0,0,10])
rotate([0,-30+4,0])
translate([0,0,50-15])
rotate([0,90,0])
rotate([90,0,0])
import("printable/leg_lower.stl");

translate([0,0,-5/2-15])
foot_mock();

translate([0,0,-10])
foot_mock_ankle();

translate([0,5,-20])
foot_mock_toe();

translate([0,5,-20])
foot_mock_toe_spring();
