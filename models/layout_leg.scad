use <leg_upper.scad>;
use <leg_middle.scad>;
use <leg_lower.scad>;
use <leg_spring.scad>;
use <motors_alturn.scad>;
use <pelvis.scad>;
use <torso_side.scad>;
use <pelvis_torso_strut.scad>;
use <arduino_mega_mount.scad>;
use <leg_hip_holder.scad>;

x_offset = 60;

translate([1.25,-1,0.2])
for(i=[0:1])
mirror([i,0,0])
translate([0,0,-40])//lower leg
translate([0,-25,0])//center leg
translate([x_offset+1.5,7.5,-5])
rotate([-45,0,0])
translate([0,0,5])
if(1){
    
    translate([7.5,0,0])
    if(1){
        translate([0,0,-50+10])
        rotate([0,-90,0])
        leg_upper(show_axle=1);
        //import("printable/leg_lower.stl");

        translate([0,90,-100])
        rotate([0,0,180])
        rotate([0,90,0])
        //leg_lower();
        import("printable/leg_lower.stl");

        translate([0,50-5,-60])
        rotate([0,0,90])
        //leg_middle();
        import("printable/leg_middle.stl");

        translate([0,50-5,-80])
        rotate([0,0,-90])
        //leg_middle();
        import("printable/leg_middle.stl");

        translate([0,0,-60])
        rotate([-12.5,0,0])
        rotate([0,90,0])
        rotate([0,0,90])
        color("blue")
        make_full_spring(extension=0.21);
    }
    
    translate([2.5,0,0])
    translate([0,0,-5])
    rotate([0,90,0])
    color("red")
    rotate([0,0,90+6.1])//spin
    import("printable/leg_hip_gear_main.stl");  
    
}


// center body about origin
if(1)
translate([x_offset,0,0]){

    if(1)
    for(i=[0:1]){
        translate([i*(-12.6-100-4.9),0,0])
        mirror([i,0,0]){

            color("orange")
            translate([-3.5,-2.5+10,-5])
            rotate([0,90,0])
            make_motors_alturn_abrs_5314htg_hv_horn();
            
            translate([-3.5+3.5,-2.5+10,-5])
            rotate([0,90,0])
            rotate([0,180,0])
            color("green")
            import("printable/leg_hip_spacer.stl");

            color("purple")
            translate([-22.5,-2.5+10,4.5])
            rotate([90,0,0])
            rotate([0,90,0])
            //make_motors_alturn_abrs_5314htg_hv();
            import("printable/motors_alturn_abrs_5314htg_hv.stl");

            color("purple")
            translate([-22.5,-32-5+2.5-10,4.5])
            rotate([-90,0,0])
            rotate([0,90,0])
            //make_motors_alturn_abrs_5314htg_hv();
            import("printable/motors_alturn_abrs_5314htg_hv.stl");

            translate([-3.75-2.5/2,-(32/2+5/2),-4+8.5])
            //pelvis_motor_face();
            rotate([0,-90,0])import("printable/pelvis_motor_face.stl");
            
        }
    }

    if(1){
        translate([-58.75, 29+5, 4.5])
        //pelvis_cross_support();
        rotate([-90,0,0])
        import("printable/pelvis_cross_support.stl");

        translate([-58.75, -66-5, 4.5])
        //pelvis_cross_support();
        rotate([90,0,0])
        import("printable/pelvis_cross_support.stl");
    }

    color("gray")
    translate([-109.5,0,100+0.5])
    rotate([-90,0,0])
    import("electronics/arduino-mega-2560/Arduino_MEGA2560.stl");

    translate([-58.75, -18.5, 4.5+30])
    //pelvis_torso_strut();
    import("printable/pelvis_torso_strut.stl");

    color("blue")
    translate([-58.75, -66-5+55+5+2.5, 4.5+70])
    //pelvis_cross_support();
    rotate([90,0,0])
    import("printable/pelvis_cross_support.stl");

    for(i=[0:1])
    translate([-6.25-i*105,-18.5,74.5])
    //torso_side();
    rotate([0,90,0])
    import("printable/torso_side.stl");

    //translate([-58.75, -66-5+55+5+2.5, 4.5+70])
    //arduino_mega_mount();
    import("printable/arduino_mega_mount.stl");



}

translate([1.25,0,0])
for(i=[0:1])
    mirror([i,0,0])
    //color("blue")
    translate([50,-20+1.5,-28+5])
    rotate([0,0,180])//spin
    //leg_hip_holder();
    rotate([0,-90,0])rotate([0,0,-45])import("printable/leg_hip_holder.stl");
