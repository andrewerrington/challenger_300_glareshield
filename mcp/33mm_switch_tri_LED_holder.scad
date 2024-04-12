use <33mm_switch_LED_baffle_short.scad>

difference(){
  union(){
    // Main body
    translate([-2.3,-4,0])
    cube([4.6,8,16.8+7]);
    cylinder(d=10,h=5.8+7,$fn=40);
    // New LED block
    translate([0,0,-8])
    cylinder(d=15.8,h=8,$fn=40);    
  }
  
  // Slot for baffle
  translate([-10,-1,-8])
  cube([20,2,4]);
  
  // Orientation marker (top is red)
  translate([0,4.5,5.5])
  rotate([-90,0,0])
  cylinder(d=3,h=2,$fn=20);
  
  // Slots for internal blocks
  translate([-2.4,-1.2,12.8])
  cube([0.9,2.4,30]);
  translate([2.4-0.9,-1.2,12.8])
  cube([0.9,2.4,30]);
  
  // LEDs top
  // LED body
  translate([0,4,-8.1])
  cylinder(d=3.5,h=6,$fn=20);
  translate([-4.5,4,-8.4])
  rotate([0,10,0])
  cylinder(d=3.6,h=6,$fn=20);
  translate([4.5,4,-8.4])
  rotate([0,-10,0])
  cylinder(d=3.6,h=6,$fn=20);

  // LED flange
  translate([0,4,-5.1])
  cylinder(d=4.2,h=6.5,$fn=20);
  translate([-4.0,4,-5.1])
  rotate([0,10,0])
  cylinder(d=4.4,h=5,$fn=20);
  translate([4.0,4,-5.1])
  rotate([0,-10,0])
  cylinder(d=4.4,h=5,$fn=20);

  translate([0,2.3,-4])
  rotate([-90,0,0])
  difference(){
    cylinder(d=12,h=3,$fn=40);
    translate([-6,0,0])
    cube([12,6,3]);
  }


  // Remove space for leads
  translate([-6.1,4,-5.1])
  cube([12.2,6,5.2]);

  // LED wire holes
  translate([-1.6,2.9-0.6,0]) 
  cube([1.2,1.2,12.8]);
  translate([0.4,2.9-0.6,0]) 
  cube([1.2,1.2,12.8]);
  
  // Slot
  translate([-1.6,4-1.6,12.8])
  cube([3.2,1.6,4]);

  // Ramp
  translate([-1.6,4-1.6,16.8])
  rotate([-30,0,0])
  cube([3.2,1.6,4]);
  
  // LEDs bottom
  // LED body
  translate([0,-4,-8.1])
  cylinder(d=3.5,h=6,$fn=20);
  translate([-4.5,-4,-8.4])
  rotate([0,10,0])
  cylinder(d=3.6,h=6,$fn=20);
  translate([4.5,-4,-8.4])
  rotate([0,-10,0])
  cylinder(d=3.6,h=6,$fn=20);

  // LED flange
  translate([0,-4,-5.1])
  cylinder(d=4.2,h=6.5,$fn=20);
  translate([-4.0,-4,-5.1])
  rotate([0,10,0])
  cylinder(d=4.4,h=5,$fn=20);
  translate([4.0,-4,-5.1])
  rotate([0,-10,0])
  cylinder(d=4.4,h=5,$fn=20);
  
  translate([0,-5.3,-4])
  rotate([-90,0,0])
  difference(){
    cylinder(d=12,h=3,$fn=40);
    translate([-6,0,0])
    cube([12,6,3]);
  }

  // Remove space for leads
  translate([-6.1,-10,-5.1])
  cube([12.2,6,5.2]);

  // LED wire holes
  translate([-1.6,-2.9-0.6,0]) 
  cube([1.2,1.2,12.8]);
  translate([0.4,-2.9-0.6,0]) 
  cube([1.2,1.2,12.8]);
  
  // Slot
  translate([-1.6,-4,12.8])
  cube([3.2,1.6,4]);
  
  // Ramp
  translate([-1.6,-2.4,16.8])
  rotate([30,0,0])
  translate([0,-1.6,0])
  cube([3.2,1.6,4]);
  
}

if (0){

translate([0,0,-14.7])
baffle();

translate([-19/2,-8.9-0.6,-14.7])
color("white",0.6)
cube([19,8.9,3]);
translate([-19/2,0.6,-14.7])
color("white",0.6)
cube([19,8.9,3]);
} 