// 3D print version of MCP.

// Accepts a modified 33mm square arcade button for
// CAUTION/WARNING indicator.

// Original file is a DXF for laser cutting of three
// layers:
// Panel: 1.5 or 1.6mm (1/16") acrylic
// Fascia: two layers of 3mm (1/8") MDF
// These are stacked and screwed together, then primed
// and painted.

// The 3D model is based on the final stacked assembly.
// Takes several minutes to render due to the Minkowski
// operation to make the rounded top edges.

function to_mm(in) = in * 25.4;
mcp_file = "mcp.dxf";

module panel() {
  linear_extrude(height = to_mm(1/16), convexity = 10, $fn = 30)
  import(mcp_file, layer = "panel");
 }
 
module fascia() {
  linear_extrude(height = to_mm((1/4)-(1/32)), convexity = 10, $fn = 30)
  import(mcp_file, layer = "fascia_outline");
  
  // The top slice is made with a minkowski sum to
  // round off the top edges. The 'lower' rounded
  // part is lost inside the bulk extrusion from above.
  translate([0, 0, to_mm((1/4) - (1/32)) - 0.01])
  minkowski() {
    sphere(r = to_mm(1/32), $fn = 30);
    linear_extrude(height = 0.01, convexity = 10, $fn = 30)
    offset(r = -to_mm(1/32))
    import(mcp_file, layer = "fascia_outline");
  }
}

difference() {
  // Build the panel and the fascia as one block.
  union() {
    panel();
    translate([0, 0, to_mm(1/16)])
    fascia();
  }

  // Remove the hole in fascia, step up by the thickness
  // of the panel, plus 1mm so that the switch does
  // not sit too far below the face of the fascia.
  // Expand the hole by 0.2mm.
  translate([0, 0, to_mm(1/16) + 1])
  linear_extrude(height = to_mm(1/4), convexity = 10, $fa=0.5, $fs=0.2, $fn = 60)
  offset(r = 0.2)
  import(mcp_file, layer = "fascia_cut");
  
  // Remove the hole in the panel.
  // Expand the hole by 0.2mm.
  linear_extrude(height = to_mm(1/4), convexity = 10, $fn = 60)
  offset(r = 0.2)
  import(mcp_file, layer = "panel_cut");

  // The holes that are used to fasten the fascia and
  // panel together will be present, but only two are
  // used, for the connector PCB bracket.
  linear_extrude(height = to_mm(1/4), convexity = 10, $fn = 60)
  import(mcp_file, layer = "fasteners_fascia");

}
