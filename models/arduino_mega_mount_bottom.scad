use <arduino_mega_mount.scad>;

rotate([0,0,0])
difference(){
arduino_mega_mount();

translate([0,0,100/2])
cube([150,100,100], center=true);
}
