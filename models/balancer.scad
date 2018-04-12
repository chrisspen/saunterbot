use <openscad-extra/src/strut.scad>;
use <openscad-extra/src/countersink.scad>;
use <gears/parametric_involute_gear_v5.0.scad>;
use <gears/rack_and_pinion.scad>;

gear_pitch = 200;
mm_per_tooth = 3;


module balancer_drive_gear(){
    h = 2.5+2.5+1;
    
    difference(){
        //translate([0,0,-10])//TODO:remove
        rotate([0,0,4])
        color("orange")
        pinion_gear(
            mm_per_tooth=mm_per_tooth,
            number_of_teeth=39,
            hole_diameter=8,
            thickness=h
        );
        
        color("red")
        for(i=[-1:2:1])
        translate([(15.8-1.9)/2*i,0,-3])
        rotate([180,0,0])
        make_countersink(d1=2, d2=4.5, inner=20, outer=20);
    }
    
}

module balancer_opp_gear(d=(2*60)/(3.14159)){
    difference(){
        
        //translate([0,0,-10])//TODO:remove
        rotate([0,0,4])
        color("blue")
        pinion_gear(
            mm_per_tooth=mm_per_tooth,
            number_of_teeth=39,
            hole_diameter=10,
            thickness=2.5+2.5+1
        );
        
        translate([0,0,-10/2+1/2])
        cylinder(d=(d+10)/2, h=10, center=true);
    }
}

module balancer_gear_axle(){
    // assumes a 10mm OD 6mm ID goes between it and the axle
    //todo
}

module balancer_gear_rack(servo_rack_length=40, servo_rack_width=5){
    intersection(){
        color("yellow")
        cube([servo_rack_length,servo_rack_width*2,5*2], center=true);

        color("red")
        //scale([1/2,1/2,1/2])
        //translate([0,0,-10])//TODO:remove
        translate([-3*8,0,0])
        translate([-3,0,0])
        //translate([-2.8,0,0])
        //rack(mm_per_tooth=8,number_of_teeth=8,thickness=6,height=12);
        for(i=[0:1])
        mirror([0,i,0])
        InvoluteGear_rack(mm_per_tooth=mm_per_tooth, number_of_teeth=20, thickness=5, height=3.5);
    }

    color("red")
    translate([0,0,-3])
    cube([servo_rack_length, 8-.5, 4], center=true);
}

module servo_hitec_hs300_holes(d=3, h=100){
    for(i=[-1:2:1])
    for(j=[-1:2:1])
    color("red")
    translate([(7+3)/2*i,(41+3.5*2)/2*j,0])
    cylinder(d=d, h=h, center=true, $fn=50);
}

module servo_hitec_hs300_bb(horn_top_extra=0, horn_bottom_extra=0, buffer=0, horn_buffer=0, show_axle=0){
    axle_offset_y = -10.15;

    // main body bb
    cube([20+buffer,41+buffer,36.2+buffer],center=true);
    
    // servo horn bb
    translate([0,axle_offset_y,0])
    translate([0,0,6.7/2+36.2/2+horn_top_extra/2-horn_bottom_extra/2])
    cylinder(d=24+horn_buffer, h=6.7+horn_top_extra+horn_bottom_extra, center=true, $fn=50);

    difference(){
            
        // flange bb, 25mm from bottom
        translate([0,0,+6/2-36.2/2+25])
        cube([20+buffer,41+7*2+buffer,6+buffer],center=true);
    
        // mount tab holes
        servo_hitec_hs300_holes();
    }
    
    // mock servo axle
    if(show_axle)
    color("red")
    translate([0,axle_offset_y,0])
    cylinder(d=2, h=100, center=true, $fn=50);
    
}

module balancer_translation_gear(d=5, extra_rotation_z=0){
    translate([0,0,-(-2.5-1)/2])
    rotate([0,0,25.5+extra_rotation_z])
    color("green")
    //translate([0,0,-100])//TODO:remove
    rotate([0,0,4])
    pinion_gear(
        mm_per_tooth=mm_per_tooth,
        number_of_teeth=21,
        hole_diameter=10,
        thickness=2.5
    );
   
}

module balancer_middle_plate(){
    h = 1+2.5+1+2.5+1+2.5;
    
    difference(){
        union(){
        if(1)
        translate([0,0,-h/2-5/2])
        difference(){
            // walls
            color("orange")
            cube([110,110,h], center=true);
            
            // center cutout
            cube([100,90,50], center=true);
            
            // mount holes
            /*
            color("red")
            for(j=[0:8])
            for(i=[-1:2:1])
            translate([10*j-40,(50-5/2)*i,0])
            cylinder(d=2.5, h=50, center=true, $fn=50);
            */
            
        }
        
        if(1)
        difference(){
            union(){
                // floor
                translate([0,0,-5/2-h-2.5/2+2.5])
                cube([100,90,2.5], center=true);
                
                balancer_gear_layout(show_gears=0, show_top_axles=0, show_bottom_axles=0, show_bottom_axle_mass=1);
            }
            
            balancer_gear_layout(show_gears=0, show_top_axles=0, show_bottom_axles=1);
            
            // rack slot
            translate([0,0,-10])
            cube([100, 8, 20], center=true);

            // rail slots
            translate([0,0,-14])
            color("blue")
            balancer_weight_container_rails(length=100, thickness=3);
        }

        //balancer_gear_layout(show_gears=1, show_top_axles=0, show_bottom_axles=0);
        }
        
        balancer_bottom_expansion_holes();
        
        balancer_bottom_plate_tapper();
        
        
        // These need to be 2.5 so the bottom plate can grip.
        balancer_bottom_plate_mount_holes(d=2.5);
            
        // vertical mount holes to attach the middle plate to the upper plate
        color("blue")
        for(i=[0:3])
        for(j=[-1:2:1])
        translate([20*i-30, (50-5/2)*j, -23+10.3])
        rotate([180,0,0])
        make_countersink(d1=2.5, d2=5.5, inner=15, outer=10);

    }// end diff
        
        
}

module balancer_gear_axle_top(cutoff=0){
    difference(){
        union(){
            translate([0,0,1/2])
            cylinder(d=7.25, h=1, center=true, $fn=50);
            translate([0,0,2.5/2+1])
            cylinder(d=5.5, h=2.5, center=true, $fn=50);
        }
        cylinder(d=2, h=10, center=true, $fn=50);
        
        if(cutoff)
        translate([0,10/2+2,0])
        cube([10,10,10], center=true);
    }
}

module dime_roll_cutout(extra=0){
    translate([0,-extra/2,0])
    rotate([90,0,0]){
        cylinder(d=17.9+0.75, h=1.35*63+extra, center=true, $fn=50);
        translate([0,0,-100/2])
        cylinder(d=3, h=100, center=true, $fn=50);
    }
}

module balancer_weight_container_rails(thickness=2.5, length=40, cutoff=0){
    difference(){
        // top rails
        for(i=[-1:2:1])
        color("green")
        translate([0,35*i,0])
        rotate([45,0,0])
        cube([length, thickness, thickness], center=true);

        if(cutoff)
        translate([0,0,-100/2])
        cube([100,100,100], center=true);
    }
}

module balancer_weight_container_door_holes(hole_buffer=0){
    
    color("red")
    for(i=[-1:2:1])
    for(j=[0:1])
    translate([-17.5*i,-50,-16.4+-j*17.15])
    rotate([90,0,0])
    make_countersink(d1=2+hole_buffer, d2=4.5, inner=30, outer=20);
}

module balancer_weight_container_complete(hole_buffer=0){
    
    translate([0,0,-14]){
        difference(){
            union(){
                // main mass
                translate([0,0,-22/2])
                cube([40, 89, 22], center=true);
                
                balancer_weight_container_rails();
                
                //color("blue")
                translate([0,0,-9.5+14])
                balancer_gear_rack();
            }
            
                
//            color("gray")
//            translate([0,-41.5,-34.3+14])
//            scale([1.15,1.15,1.15])
//            rotate([0,0,180])
//            balancer_weight_container_door(width=16.5, height=30);//17);
            
            // mass removal to stop falty print
            //translate([0,0,-11])
            //cube([10, 1.35*67, 10], center=true);
            
            color("blue")
            for(i=[-1:2:1])
            translate([0,0,-17/2-2.5])
            translate([9.5*i,0,-0]) dime_roll_cutout(extra=0);
            
            // door mount holes
            translate([0,5.5,14])
            balancer_weight_container_door_holes(hole_buffer=hole_buffer);
            
            // bottom rail cutout
            translate([0,0,-23.5])
            balancer_weight_container_rails(length=50, thickness=3);
            
        }
        
        
    }

}

module balancer_weight_container(){
    intersection(){
        balancer_weight_container_complete();
        color("blue")
        translate([0,100/2-40,0])
        cube([100,100,100],center=true);
    }
}

module balancer_weight_container_door(){
    intersection(){
        balancer_weight_container_complete(hole_buffer=0.5);
        color("red")
        translate([0,-100/2-40-0.5,0])
        cube([100,100,100],center=true);
    }
}

//module balancer_weight_container_door(width=10, thickness=2, height=17){
//    linear_extrude(height)
//    polygon([
//        [-width+thickness, -thickness/2],
//        [-width, thickness/2],
//        [width, thickness/2],
//        [width-thickness, -thickness/2],
//    ]);
//}

module balancer_bottom_expansion_holes(){
    
        // horizontal expansion holes
        for(k=[-1:2:1])
        for(i=[-1:2:1])
        for(j=[0:3])
        translate([55*i,(45+2.5)*k,10*j-36])
        rotate([0,90,0])
        cylinder(d=2.5, h=10, center=true, $fn=50);
}

module balancer_bottom_plate_mount_holes(d=2.5){
    // vertical screw holes
    for(i=[0:8])
    for(j=[-1:2:1])
    translate([40*i-40, (50-5/2+2.5+0.5)*j, -17])
    rotate([8*j,0,0])
    rotate([180,0,0])
    make_countersink(d1=2.5, d2=5.5, inner=15-2, outer=30);
}

module balancer_bottom_plate(){
    h = 23+2.5+1;

    difference(){
        union(){
            difference(){
                // main mass and outer wall
                color("green")
                translate([0,0,-h/2-14+1])
                cube([110,110, h], center=true);

                // inner cutout
                color("purple")
                translate([0,0,0])
                translate([0,0,-h/2-14+2.5])
                cube([100,91, h], center=true);
                
                // test inner cutout
//                color("purple")
//                translate([30,30,0])
//                translate([0,0,-h/2-14+2.5])
//                cube([100,91, h], center=true);
            }

            // rail slots
            translate([0,0,-14-23.5-0.5])
            color("blue")
            balancer_weight_container_rails(length=100, cutoff=1);
        }
        
        balancer_bottom_plate_mount_holes(d=3);
        
        balancer_bottom_plate_tapper();
        
        balancer_bottom_expansion_holes();
    }// end diff
        
}

module balancer_bottom_plate_tapper(){
    for(i=[0:1]){
        rotate([0,0,180*i])
        translate([0,-100/2-0.5,-39.5])
        rotate([7.75,0,0])
        translate([0,-200/2,0])
        cube([200,200,200],center=true);
    }
    
    //translate([200,-100/2,-39.5])color("red")sphere(1, $fn=100);
}

module balancer_gear_layout(servo_drive_gear_diameter=(2*60)/(3.14159), servo_rack_width=5, show_gears=1, show_top_axles=1, show_bottom_axles=1, show_bottom_axle_mass=0){
    opp_gear_offset_x = 23;//20;
    translation_gear_diameter = servo_drive_gear_diameter*.5+2;//30;
    translation_gear_angle = 12.5;
    bottom_axle_z_offset = -6.4;
    
    // servo drive gear
    if(show_gears)
    translate([0,-(servo_drive_gear_diameter/2+servo_rack_width/2),-(2.5+2.5+1)/2-5/2-1])
    {
        //
        //cylinder(d=servo_drive_gear_diameter, h=2.5+2.5+1, center=true);
        //balancer_drive_gear();
        color("orange")import("printable/balancer_drive_gear.stl");
        
        //color("blue")
        cylinder(d=1, h=200, center=true, $fn=50);
    }
    
    // opposite drive gear
    //color("blue")
    for(i=[-1:2:1])
    translate([opp_gear_offset_x*i,servo_drive_gear_diameter/2+servo_rack_width/2,-(2.5+2.5+1)/2-5/2-1]){
        if(show_gears){
            rotate([0,0,(i==1?0:3)])
            //balancer_opp_gear(d=servo_drive_gear_diameter);
            color("blue")rotate([180,0,0])import("printable/balancer_opposite_gear.stl");
        }
        if(show_top_axles) translate([0,0,10-2.5]) make_countersink(d1=2, d2=4.5, inner=20, outer=20);
        if(show_bottom_axles) translate([0,0,bottom_axle_z_offset]) rotate([180,0,0]) make_countersink(d1=2, d2=4.5, inner=20, outer=20);
        if(show_bottom_axle_mass) translate([0,0,-3.5/2]) cylinder(d=7.25, h=1+2.5+1, center=true, $fn=50);
    }
    
    // translation gears drive-side
    //color("blue")
    for(i=[-1:2:1])
    translate([(0)*i,0,-(2.5+2.5+1)/2-5/2-1])
    translate([0,-(servo_drive_gear_diameter/2+servo_rack_width/2),0])
    rotate([0,0,(90-translation_gear_angle+0.5)*i])
    translate([0,-servo_drive_gear_diameter/2,0])
    translate([0,-translation_gear_diameter/2+0.5,0]){
        if(show_gears){
            balancer_translation_gear(d=translation_gear_diameter, extra_rotation_z=(i==-1?-10:5));
        }
        if(show_top_axles) translate([0,0,10-2.5]) make_countersink(d1=2, d2=4.5, inner=20, outer=20);
        if(show_bottom_axles) translate([0,0,bottom_axle_z_offset]) rotate([180,0,0]) make_countersink(d1=2, d2=4.5, inner=20, outer=20);
        if(show_bottom_axle_mass) translate([0,0,-3.5/2]) cylinder(d=7.25, h=1+2.5+1, center=true, $fn=50);
    }

    // translation gears opp-side
    //color("blue")
    for(i=[-1:2:1])
    translate([opp_gear_offset_x*i,servo_drive_gear_diameter/2+servo_rack_width/2,-(2.5+2.5+1)/2-5/2-1])
    rotate([0,0,(translation_gear_angle-1.5)*i])
    translate([0,-servo_drive_gear_diameter/2,0])
    translate([0,-translation_gear_diameter/2+0.5,0]){
        if(show_gears) balancer_translation_gear(d=translation_gear_diameter, extra_rotation_z=(i==1?1:5.5));
        if(show_top_axles) translate([0,0,10-2.5]) make_countersink(d1=2, d2=4.5, inner=20, outer=20);
        if(show_bottom_axles) translate([0,0,bottom_axle_z_offset]) rotate([180,0,0]) make_countersink(d1=2, d2=4.5, inner=20, outer=20);
        if(show_bottom_axle_mass) translate([0,0,-3.5/2]) cylinder(d=7.25, h=1+2.5+1, center=true, $fn=50);
    }

    // mock rack
    if(show_gears)
    translate([25*-1.5+1.7,0,0])
    translate([0+3*2.5,0,-5-1-2.5-1])
    balancer_gear_rack(servo_rack_length=40, servo_rack_width=servo_rack_width);
    
}

module balancer_top_plate(show_servo=0, show_layers=0, show_gears=0){
    
    //servo_axle_y = -29.15;
    servo_rack_length = 40; // should be long enough to span two rows of dimes 18*2+1+1+1 = 39 => 40
    servo_rack_width = 5;
    
    // Needs to be long enough to move middle point of the rack from one side to the other
    // 100 - 40/2 - 40/2 = 60 = rack traversal length = 2*pi*r => r = length/(2*pi)
//    servo_drive_gear_diameter = 60/(3.14159); // C = pi*d => d = C/pi
    servo_drive_gear_diameter = (2*60)/(3.14159); // C = pi*d => d = C/pi
    servo_horn_diameter = 24;

    union(){
        difference(){
            
            // main plate mass
            //rotate([0,90,0])
            //make_cross_plate(w=100,h=100,hub_d=0,t=5,cross_type=1);
            cube([100,100,5], center=true);
            
            // servo hole rim mass
            //translate([0,(41+21)/2-50,0])
            //cube([20+10,41+21, 5], center=true);
            
            color("blue")
            translate([0,-(servo_drive_gear_diameter/2+servo_rack_width/2),-50/2+5])
            cylinder(d=servo_horn_diameter+2, h=50, center=true, $fn=50);

            // front/back horizontal mount holes
            color("red")
            for(j=[-1:2:1])
            for(i=[0:8])
            translate([49.5*j,10*i-40,0])
            rotate([0, 90, 0])
            cylinder(d=2.5, h=10, center=true, $fn=25);

            // left/right horizontal mount holes
            color("red")
            for(j=[-1:2:1])
            for(i=[0:9])
            translate([10*i-45,49.5*j,0])
            rotate([90, 0, 0])
            cylinder(d=2.5, h=10, center=true, $fn=25);
        
            // front/back vertical mount holes
            color("red")
            for(j=[-1:2:1])
            for(i=[0:9])
            translate([(49.5-5/2)*j,10*i-45,0])
            //rotate([0, 90, 0])
            cylinder(d=2.5, h=10, center=true, $fn=25);
            
            // left/right vertical mount holes
            color("red")
            for(j=[-1:2:1])
            for(i=[0:8])
            translate([10*i-40,(49.5-5/2)*j,0])
            //rotate([90, 0, 0])
            cylinder(d=2.5, h=10, center=true, $fn=25);
            
            // gear axle screw holes
            balancer_gear_layout(servo_drive_gear_diameter=servo_drive_gear_diameter, servo_rack_width=servo_rack_width, show_gears=0, show_top_axles=1);    
            
        }// end diff

        // servo mount nubs
        translate([0,-11.5,0]){
            difference(){
                for(i=[-1:2:1])
                translate([0,(42/2+7/2)*i,5])
                cube([20, 6.5, 6], center=true);
                
                color("red")
                translate([0,0,50/2+12-10])
                servo_hitec_hs300_holes(d=2.5, h=50);
            }//end diff
        }
    }
    
    // rim to block top holes
//    color("red")
    for(j=[0:1])
    for(i=[-1:2:1])
    rotate([0,0,90*j])
    translate([0,(100/2-5/2)*i,3])
    cube([100,5,1], center=true);

    // mock servo placement
    //translate([0,-19-3.25,0])
    if(show_servo)
    rotate([0,0,0])
    translate([0,0,25-3.75])
    translate([0,-11.5,0]){

        translate([0,-10.15,17]){
            
            color("green")
            rotate([-90,0,0])
            scale([10,10,10])
            import("electronics/hs-311-hitec-standard-servo/HS-311 Hitec Standard Servo.stl");

            // mock servo axle
            if(0)
            color("red")
            translate([0,0,-25])
            cylinder(d=2, h=100, center=true, $fn=50);
        }

        // measured servo bb
        color("blue")
        rotate([0,180,0])
        servo_hitec_hs300_bb(buffer=0.5, horn_bottom_extra=10, horn_buffer=1, show_axle=1);
    }
    
    if(show_gears){
        balancer_gear_layout(servo_drive_gear_diameter=servo_drive_gear_diameter, servo_rack_width=servo_rack_width, show_top_axles=0);
    }

    if(show_layers)
    translate([100,0,0]){

        // mock gear engagment layer
        color("purple")
        translate([0,0,-2.5/2-3.5-2.5-1])
        cube([100,100,2.5], center=true);

        // mock gear translation layer
        color("gray")
        translate([0,0,-2.5/2-3.5])
        cube([100,100,2.5], center=true);
        
        // mock weight layer
        color("turquoise")
        translate([0,0,-2.5/2-3.5-2.5-1-1-2.5])
        cube([100,100,2.5], center=true);

    }

}

//translate([0,50,-43+3.5])cube([10,10,43]);
//translate([0,60,-25+3.5])cube([10,10,25]);

difference(){
    union(){
        //balancer_top_plate(show_servo=0, show_layers=0, show_gears=0);
        //import("printable/balancer_top_plate.stl");

        //translate([0,0,-10])
        //balancer_middle_plate();
        //import("printable/balancer_middle_plate.stl");

        balancer_bottom_plate();
        //import("printable/balancer_bottom_plate.stl");
    }

    translate([200/2,0,0])
    cube([200,200,200],center=true);
}

//balancer_weight_container();
import("printable/balancer_weight_container.stl");
