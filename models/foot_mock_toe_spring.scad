use <foot_mock.scad>;

if(0)
translate([0,0,-5/2-15])
foot_mock();

if(0)
translate([0,0,-10])
foot_mock_ankle();

if(0)
translate([0,5,-20])
foot_mock_toe();

if(1)
rotate([90,0,0])
translate([0,5,-20])
foot_mock_toe_spring();
