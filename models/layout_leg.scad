use <leg_upper.scad>;
use <leg_middle.scad>;
use <leg_lower.scad>;
use <leg_spring.scad>;
use <motors_alturn.scad>;
use <pelvis.scad>;
use <torso_side.scad>;
use <pelvis_torso_strut.scad>;
use <arduino_mega_mount.scad>;

x_offset = 60;

for(i=[0:1])
mirror([i,0,0])
translate([x_offset,7.5,-5])
rotate([-45,0,0])
translate([0,0,5])
if(1){
    translate([0,0,-50+10])
    rotate([0,-90,0])
    leg_upper();
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

// center body about origin
translate([x_offset,0,0]){

    for(i=[0:1]){
        translate([i*(-12.6-100-4.9),0,0])
        mirror([i,0,0]){

            color("orange")
            translate([-3.5,-2.5+10,-5])
            rotate([0,90,0])
            make_motors_alturn_abrs_5314htg_hv_horn();

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