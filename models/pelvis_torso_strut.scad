use <openscad-extra/src/rounded.scad>;
use <openscad-extra/src/countersink.scad>;
use <openscad-extra/src/torus.scad>;

module battery_mount_holes(){
    //36.8+2.6 id
    for(i=[-1:2:1])
    translate([(36.8+2.6)/2*i,0,0])
    cylinder(d=2.5, h=20, center=true, $fn=100);
}

module battery_holder(){
    cube([61,76.5,21], center=true);
}

module pelvis_torso_strut(wire_thrus=1, panel_holes=0){

    difference(){
        color("orange")
        cube([100, 40, 10], center=true);
        
        color("red")
        for(i=[-1:2:1])
        translate([0, -23*i, -1])
        rounded_cube_2d([90,20,20], r=4.99, $fn=100, center=true);
        
        if(wire_thrus)
        color("blue")
        for(i=[0:2])
        translate([i*30-30,0,0])
        scale([2.5,2,1])
        cylinder(d=10, h=20, center=true, $fn=100);
        
        color("red")
        translate([0,10,10])
        rotate([90,0,0])
        rounded_cube_2d([90,20,50], r=4.99, $fn=100, center=true);
    
        // chassis mount holes
        color("blue")
        for(k=[0:1])
        mirror([k,0,0])
        for(j=[-1:2:1])
        for(i=[0:3])
        translate([60,10*i-15,2.5*j])
        rotate([0,90,0])
        make_countersink(d1=2.5, d2=5, inner=20, outer=20);
        
        color("blue")
        if(panel_holes){
            // led holes
            for(i=[-1:2:1])
            translate([0,6*i,0]){
                translate([36,0,10])
                cylinder(d=10, h=5, center=true, $fn=100);
                translate([36,0,0])
                cylinder(d=8, h=50, center=true, $fn=100);
            }
            
            // switch holes
            for(i=[-1:2:1])
            translate([0,6*i,0]){
                translate([-36,0,10])
                cylinder(d=8, h=5, center=true, $fn=100);
                translate([-36,0,0])
                cylinder(d=5, h=50, center=true, $fn=100);
            }
            
            battery_mount_holes();
         
            // cutouts to give room for led/button shafts
            for(i=[-1:2:1])
            color("green")
            translate([36*i,10,10-2.5])
            rotate([90,0,0])
            rounded_cube_2d([15,20,50], r=2.5, $fn=100, center=true);   
                
            // cable tie attachment holes
            for(j=[0:1])
            for(i=[0:2])
            mirror([0,j,0])
            translate([-20+20*i,13,1.5])
            rotate([0,90,0])
            torus(r1=3/2, r2=4);
        }
               
    }

}

module battery_torso_strut(){
    pelvis_torso_strut(wire_thrus=0, panel_holes=1);
}

pelvis_torso_strut();

//translate([0,0,21/2])battery_holder();

//battery_torso_strut();

