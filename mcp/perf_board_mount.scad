// MCP connector bracket
//
// Attach a short piece of perfboard to the back of the
// MCP.

difference(){
  union(){
    translate([0,18/2,0])
    cylinder(d=6,h=2,$fn=20);
    translate([0,-18/2,0])
    cylinder(d=6,h=2,$fn=20);
    translate([0,(-18/2)-3,0])
    cube([7,18+6,2]);
    translate([0,(-18/2)-3,2])
    cube([7,18+6,4]);
    
  }
    translate([0,18/2,-1])
    cylinder(d=3,h=10,$fn=20);
    translate([0,-18/2,-1])
    cylinder(d=3,h=10,$fn=20);

    translate([0,18/2,2])
    cylinder(d=6,h=10,$fn=20);
    translate([0,-18/2,2])
    cylinder(d=6,h=10,$fn=20);
  
    // Slot for edge of perfboard
    translate([3,(-20/2)-0.2,2]){
      difference(){
        cube([2,20+0.4,10]);
        // Ribs to grip perfboard
        translate([0,6,0])
        cylinder(d=1,h=10,$fn=20);
        translate([0,14,0])
        cylinder(d=1,h=10,$fn=20);
      }
    }
  
    // Capture pins
    translate([0,16/2,4])
    rotate([0,90,0])
    cylinder(d=2,h=10,$fn=20);
    translate([0,-16/2,4])
    rotate([0,90,0])
    cylinder(d=2,h=10,$fn=20);
  
}
//cube