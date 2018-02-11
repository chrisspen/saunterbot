
module standard_stairs(){
    // 7" rise, 11" run = 177.8mm x 279.4mm
    translate([0,0,177.8/2])
    cube([500, 279.5, 177.8], center=true);
}

standard_stairs();