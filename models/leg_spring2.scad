use <openscad-extra/src/torus.scad>;
use <openscad-extra/src/rounded.scad>;

module make_leg_spring_inner(buffer=0, length=80){
    rotate([45,0,0])
    cube([length, 4+buffer, 4+buffer], center=true);
    
    if(0)
    translate([0,-2.5,0])
    cube([length, 5+buffer, 2+buffer], center=true);
}

module make_leg_spring_upper(holes=0, track=0){
    //difference(){
    
    difference(){
        union(){
            
            // axle attachment nub
            cylinder(d=10, h=2, center=true, $fn=100);
            
            // axle bar
            if(1)
            translate([13/2,0,0])
            cube([13,10,2], center=true);
            
            // main center bar
            if(!track){
                color("blue")
                translate([50/2+15-3/2,0,0])
                make_leg_spring_inner(buffer=-0.25, length=50);
                
//                translate([54,0,0])
//                rotate([45,0,0])
//                rounded_cube([,4,4], r=.25, $fn=50, center=true);
            }
            
            // main center bar shroud
            if(track){
                difference(){
                    union(){
                        color("green")
                        translate([35-3+10/2,0,0])
                        rotate([0,90,0])
                        cylinder(d=8, h=40+10, center=true, $fn=50);
                        
                        translate([52+10,0,0])
                        rotate([0,90,0])
                        torus(r1=1, r2=4-1);
                    }

                    color("blue")
                    translate([40+15-3/2,0,0])
                    make_leg_spring_inner(buffer=0.5);
                }
                
            }
            
            difference(){
                //spring end stop
                translate([5/2+10,0,0])
                rotate([0,90,0])
                cylinder(d1=7.5, d2=15, h=5, center=true, $fn=100);
                
                //spring end stop inner cutout
                color("red")
                translate([5/2+13+1,0,0])
                rotate([0,90,0])
                cylinder(d1=11, d2=19, h=5, center=true, $fn=100);
            }
        }
    
        //axle hole
        cylinder(d=3, h=20, center=true, $fn=100);
        
        if(holes)
            if(0)
        color("red")
        for(i=[0:4])
        translate([i*10+25,0,0])
        cylinder(d=2, h=10, center=true, $fn=100);
        
        if(track)
            if(0)
        color("red"){
            hull(){
                translate([-.5*10+25,0,0])
                cylinder(d=2.5, h=10, center=true, $fn=100);
                translate([4*10+25,0,0])
                cylinder(d=2.5, h=10, center=true, $fn=100);
            }
        }
        
        // bottom evening cutout
        if(1)
        color("gray")
        translate([0,0,-(200/2+7.5-3)])
        cube([200,200,200], center=true);
    
        // top evening cutout
        if(1)
        color("gray")
        translate([0,0,200/2+7.5-3])
        cube([200,200,200], center=true);
        
        // alignment holes
        for(i=[0:2])
        translate([24+10*i+5,0,0])
        cylinder(d=2, h=20, center=true, $fn=50);

        // spring end alignent hole
        color("red")
        rotate([25,0,0])
        translate([0,-5,0])
        rotate([0,90,0])
        cylinder(d=1.5, h=100, center=true, $fn=50);
        
    }// end diff

    if(0)
    translate([0,-100/2,0])
    cube([100,100,100],center=true);
        
    //}//end diff
    
    // axle printing support
//    if(!track)
//    difference(){
//        union(){
//            for(i=[0:16])
//            translate([15.75+i*2.5,0,-2.5])
//            rotate([0,0,45])
//            cube([7, .5, 4], center=true);
//
//            for(i=[0:16])
//            translate([15+i*2.5,0,-2.5])
//            rotate([0,0,-45])
//            cube([7, .5, 4], center=true);
//        }
//        color("blue")
//        translate([40/2+15-3/2,0,0])
//        make_leg_spring_inner(buffer=0.1, length=40);
//        translate([58.5,0,0])
//        cube([10,10,10], center=true);
//    }

}

module make_full_spring(extension=0){
    
    make_leg_spring_upper(holes=1);

    translate([85+35*extension,0,2])
    rotate([0,0,180])
    make_leg_spring_upper(track=1);

}

if(1){
translate([0,16,0])make_leg_spring_upper(holes=1);

make_leg_spring_upper(track=1);
}

if(0)
make_full_spring(extension=1.0);
