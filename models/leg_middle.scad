use <openscad-extra/src/countersink.scad>;
use <openscad-extra/src/nut.scad>;
use <sensor_rotary.scad>;

module leg_middle(sensor_axle=0){

    difference(){
        union(){
            difference(){
                union(){
                    // main mass
                    cube([100-10,25,5], center=true);
                    
                    // end nubs to contain axle screws
                    for(j=[-1:2:1])
                    for(i=[-1:2:1])
                    translate([90/2*i,-7.5*j,0])
                    rotate([90,0,0])
                    cylinder(d=10, h=10, center=true, $fn=50);
                                
                }
                
                // narrow center notch cutout
                color("red")
                translate([-50/2,0,0])
                cube([50,6,50], center=true);

                // wide center notch cutout
                color("red")
                translate([-15-5/2,0,0])
                cube([45,15,50], center=true);
                
                // lower narrow center notch cutout
                color("red")
                translate([75-25,0,0])
                cube([50,6,50], center=true);
                
                // part joining holes
                color("blue")
                for(i=[0:1])
                translate([10+10*i,-12.5,0])
                rotate([90,0,0])
                make_countersink(d1=2.5, d2=5, inner=30);
                
                // extra mounting holes
                color("blue")
                for(i=[0:7])
                for(j=[-1:2:1])
                translate([10*i-35,10*j,0])
                cylinder(d=2.5, h=10, center=true, $fn=100);
                
                // cutout for nuts
                color("blue")
                translate([-45,11,0])
                rotate([90,0,0])
                make_hex_nut(w=5, h=4, d=0);
                
                if(!sensor_axle){
                color("blue")
                translate([45,11,0])
                rotate([90,0,0])
                make_hex_nut(w=5, h=4, d=0);
                }
                
                // axle holes
                color("blue")
                translate([-45,-12.5,0])
                rotate([90,0,0])
                //cylinder(d=2.5, h=50, center=true, $fn=50);
                make_countersink(d1=2.5, d2=5, inner=30);
                
                color("blue")
                if(!sensor_axle){
                    translate([45,-12.5,0])
                    rotate([90,0,0])
                    //cylinder(d=2.5, h=50, center=true, $fn=50);
                    make_countersink(d1=2.5, d2=5, inner=30);
                }else{
                    
                    
                }
                
            }// end diff

            if(sensor_axle)
            translate([100/2-5,0,0])
            rotate([90,0,0])
            color("red")
            make_rotary_sensor_3382G_axle(length=20, buffer=-0.1);
            
        }

        if(sensor_axle){
            // sensor true axle hole
            translate([45,-12.5,0])
            rotate([90,0,0])
            cylinder(d=2, h=200, center=true, $fn=50);
            
            // sensor screw nub countersink
            translate([45,-14+2,0])
            rotate([90,0,0])
            cylinder(d=4, h=3, center=true, $fn=50);
        }
    }//end diff
    
}

module leg_middle_top_a(){
    difference(){
        leg_middle();
        translate([0,100/2,0])
        cube([200,100,100], center=true);
    }
}

module leg_middle_top_b(){
    difference(){
        leg_middle();
        translate([0,-100/2,0])
        cube([200,100,100], center=true);
    }
}

module leg_middle_bottom_a(){
    difference(){
        leg_middle(sensor_axle=1);
        translate([0,100/2-3,0])
        cube([200,100,100], center=true);
    }
}

module leg_middle_bottom_b(){
    difference(){
        leg_middle(sensor_axle=1);
        translate([0,-100/2-2.99,0])
        cube([200,100,100], center=true);
    }
}

leg_middle(sensor_axle=0);

//leg_middle_bottom_a();

//leg_middle_bottom_b();

//leg_middle_b();
