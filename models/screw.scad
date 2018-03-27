
module screw_2sd_5l_4hd(head_buffer=0, shaft_buffer=0, extra_length=0){
    //2mm shaft diameter
    //5mm length
    //4mm head diameter
    cylinder(d=3.8+head_buffer, h=1.8+extra_length, $fn=100);
    translate([0,0,-5-extra_length])
    cylinder(d=2+shaft_buffer, h=5+extra_length, $fn=100);
}

screw_2sd_5l_4hd(extra_length=10);
