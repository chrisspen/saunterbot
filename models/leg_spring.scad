module make_leg_spring_upper(holes=0, track=0){
    //union(){
    difference(){
        union(){
            
            // axle attachment nub
            cylinder(d=10, h=2, center=true, $fn=100);
            
            // main bar
            translate([70/2,0,0])
            cube([70,7.5,2], center=true);
            
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
        color("red")
        for(i=[0:4])
        translate([i*10+25,0,0])
        cylinder(d=2, h=10, center=true, $fn=100);
        
        if(track)
        color("red"){
            hull(){
                translate([-.5*10+25,0,0])
                cylinder(d=2.5, h=10, center=true, $fn=100);
                translate([4*10+25,0,0])
                cylinder(d=2.5, h=10, center=true, $fn=100);
            }
        }
        
        // bottom evening cutout
        color("gray")
        translate([0,0,-200/2-1])
        cube([200,200,200], center=true);
    
        // top evening cutout
        color("gray")
        translate([0,0,200/2+7.5-3])
        cube([200,200,200], center=true);
        
    }// end diff

}

module make_full_spring(extension=0){
    
    make_leg_spring_upper(holes=1);

    translate([85+35*extension,0,2])
    rotate([0,0,180])
    make_leg_spring_upper(track=1);

}

if(1){
translate([0,16,0])
make_leg_spring_upper(holes=1);

make_leg_spring_upper(track=1);
}

if(0)
make_full_spring(extension=1.0);
