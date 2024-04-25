// Master caution light baffle for 33mm arcade switch

module baffle(){

difference(){
  union(){
  // Front face
  translate([-21.4/2,-21.4/2,0])
  cube([21.4,21.4,0.8]);
  }
  // Top aperture
  translate([(-21.4/2)+1.2,0.6,0])
  cube([21.4-2.4,(21.4/2)-1.2-0.6,6]);
  // Bottom aperture
  translate([(-21.4/2)+1.2,(-21.4/2)+1.2,0])
  cube([21.4-2.4,(21.4/2)-1.2-0.6,6]); 
}


// Baffle (2D shape on XY plane)
translate([0,0.6,0.8])
rotate([90,0,0])
linear_extrude(1.2){
polygon(points=[
[19/2,0],
[19/2,2.4],
[16/2,2.4+1.5],
[16/2,2.4+1.5+3],
//[10.8/2,2.4+1.5+10],
//[10.8/2,2.4+1.5+10+2.2],

//[-10.8/2,2.4+1.5+10+2.2],
//[-10.8/2,2.4+1.5+10],
[-16/2,2.4+1.5+3],
[-16/2,2.4+1.5],
[-19/2,2.4],
[-19/2,0],

]);
}

// Rib for inset
translate([-18/2,-1,3.2])
cube([18,2,0.4]);

}

baffle();
