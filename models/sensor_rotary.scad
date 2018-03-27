
module make_rotary_sensor_3382G_axle(buffer=0, length=50){
    
    difference(){
        color("blue")
        translate([0,0,0])
        cylinder(d=3.95-buffer/2, h=length, center=true, $fn=100);
        
        translate([0,10/2+1,0])
        color("green")
        cube([10,10,100],center=true);
    }
    
}

module make_rotary_sensor_3382G_bb(buffer=0, hole=0, extra_length=0){
    
    x = 7.4 + buffer/3;
    
    difference(){
        union(){
            color("red")
            translate([0,-0.45-extra_length/2,0])
            cube([11+buffer,12.15+buffer+extra_length,1.9+buffer],center=true);
            
            // external hole rim
            translate([0,0,(1.9+buffer+0.75)/2])
            cylinder(d=5, h=0.75, center=true, $fn=100);
        }
        
        for(i=[0:1])
        mirror([i,0,0])
        color("orange")
        translate([x,x,0])
        rotate([0,0,40])
        cube([10,10,10], center=true);
        
        if(hole)
        make_rotary_sensor_3382G_axle();
        
    }
    
}

if(0)
translate([0,-.63,0])
rotate([90,0,0])
import("sensor.stl");

make_rotary_sensor_3382G_bb(buffer=0.4, hole=1);
