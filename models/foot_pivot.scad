module make_foot_pivot(){

    difference(){
        union(){
            // foundation
            translate([0,0,-5/2])
            cube([30,20,5], center=true);
            
            // side supports
            for(i=[-1:2:1]){
                translate([0,(-20/2+5/2)*i,17.5/2]){
                cube([30/2,5,17.5], center=true);

                translate([0,0,17.5/2])
                rotate([90,0,0])
                cylinder(d=15, h=5, center=true, $fn=50);
                }
            }
        }
        
        // pivot axle hole
        color("red")
        translate([0,0,17.5])
        rotate([90,0,0])
        cylinder(d=2, h=50, center=true, $fn=100);

        // foot clearance of 17.5mm
        
        //d=3mm screw hole
        for(i=[-1:2:1])
        for(j=[-1:2:1])
        color("blue")
        translate([11*i,6*j,0])
        cylinder(d=3, h=100, center=true, $fn=100);
    }
}

make_foot_pivot();

// mock foot bottom
if(0)
translate([0,0,17.5])
rotate([90,0,0])
cylinder(r=15, h=5, center=true, $fn=50);