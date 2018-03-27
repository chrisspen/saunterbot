use <motors_alturn_abrs_5314htg_hv.scad>;

module standard_stairs(){
    // 7" rise, 11" run = 177.8mm x 279.4mm
    translate([0,0,177.8/2])
    cube([500, 279.5, 177.8], center=true);
}

module make_femur(length=100){
    color("blue")
    translate([0,0,-length/2])
    cube([10, 1, length], center=true);
}

module make_connector(length=50, show_tibia=0, tibia_length=100, knee_angle=0){
    color("orange")
    translate([0,0,-length/2])
    cube([10, 1, length], center=true);
    
    if(show_tibia){
        translate([0,0,-length])
        rotate([0,knee_angle,0])
        make_tibia(length=tibia_length);
    }
}

module make_tibia(length=100){
    color("red")
    translate([0,0,-length/2]){
        cube([10, 1, length], center=true);
    
        translate([0,0,-length/2-25/2])
        make_foot();
    }
}

module make_foot(h=25){
    sphere(d=h);
}

module make_leg(hip_angle=0, femur_length=100, tibia_length=100, connector_length=50, connector_separation=0.25, knee_angle=90){
    rotate([0,hip_angle,0]){
        make_femur(length=femur_length);
        
        translate([0,0,-femur_length])
        rotate([0,-knee_angle,0]){
            make_connector(length=connector_length);
        }
        translate([0,0,-femur_length+femur_length*connector_separation]){
            rotate([0,-knee_angle,0]){
                make_connector(length=connector_length, show_tibia=1, tibia_length=tibia_length, knee_angle=knee_angle);
            }
        }
    }
}

standard_stairs();

translate([0,-250,410]){
    
    make_motors_alturn_abrs_5314htg_hv();
    
    rotate([0,0,90])
    make_leg(
        femur_length=125,
        connector_length=150,
        connector_separation=0.5,
        tibia_length=175,
        hip_angle=45-40-20,
        knee_angle=90+70-60 // 90=center, +/- 60
    );

    translate([-25,0,0])
    rotate([0,0,90])
    make_leg(
        femur_length=125,
        connector_length=150,
        connector_separation=0.5,
        tibia_length=175,
        hip_angle=45,
        knee_angle=90+20 // 90=center, +/- 60
    );

    translate([-50,0,0])
    rotate([0,0,90])
    make_leg(
        femur_length=125,
        connector_length=150,
        connector_separation=0.5,
        tibia_length=175,
        hip_angle=45-40,
        knee_angle=90-70-0 // 90=center, +/- 60
    );
}