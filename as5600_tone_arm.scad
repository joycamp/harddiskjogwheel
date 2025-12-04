//
// HDD jog wheel "tone arm" for Adafruit AS5600 (Adafruit 6357)
// Arm mounts at the left screw hole, treated as (0, 0)
// Spindle centre is at (spindle_dx, spindle_dy)
//
// All units in millimetres
//

// --------------------------
// Geometry inputs
// --------------------------

// Height from HDD lid to magnet top
magnet_above_lid = 3;          // 3 mm magnet on the hub

// Sensor gap you want between chip and magnet
sensor_gap = 2;                // 1 to 3 mm is ideal

// Board dimensions for Adafruit 6357 (approx, tweak if needed)
board_x = 20;   // along arm
board_y = 18;   // across arm
pcb_thickness = 1.6;
chip_above_pcb = 1.0;          // distance chip body top above PCB

// Tone arm geometry
arm_outer_diameter  = 8;
arm_inner_diameter  = 4;
headshell_length    = 26;      // region under sensor
headshell_width     = 22;
headshell_thickness = 3;

// Mount block geometry
base_block_length   = 26;
base_block_width    = 24;
base_block_height   = 10;
mount_hole_diameter = 3.5;     // M3 screw

// --------------------------
// Position of spindle relative to screw hole
// X to the right, Y up when looking at the lid
// Your measurements:
//   X = 65 mm
//   Y = 20 mm
// --------------------------

spindle_dx = 65;
spindle_dy = 20;

// --------------------------
// Derived heights
// --------------------------

// Magnet top relative to lid plane
magnet_top_z = magnet_above_lid;

// Chip height
chip_z       = magnet_top_z + sensor_gap;

// PCB bottom height so chip lands at chip_z
pcb_bottom_z = chip_z - chip_above_pcb - pcb_thickness / 2;

// Place underside of headshell just above PCB
headshell_z  = pcb_bottom_z + headshell_thickness / 2;

// Arm tube centre height
arm_centre_z = headshell_z + 2;   // small offset above headshell

// --------------------------
// Helper modules
// --------------------------

module arm_tube(length, od, id) {
    difference() {
        rotate([0,90,0])
            cylinder(h = length, r = od/2, center = true, $fn = 48);
        rotate([0,90,0])
            cylinder(h = length + 0.5, r = id/2, center = true, $fn = 48);
    }
}

module base_block() {
    // Block extends out in +X from the screw hole at (0,0)
    translate([base_block_length/2, 0, base_block_height/2])
        cube([base_block_length, base_block_width, base_block_height], center = true);

    // Mount hole at origin down through the block
    translate([0, 0, base_block_height/2])
        cylinder(h = base_block_height + 2,
                 d = mount_hole_diameter,
                 center = true, $fn = 32);
}

module headshell() {
    translate([spindle_dx, spindle_dy, headshell_z])
        cube([headshell_length, headshell_width, headshell_thickness],
             center = true);
}

module as5600_pocket() {
    // Slightly oversized pocket for the PCB inside the headshell
    pocket_x = board_x + 1;
    pocket_y = board_y + 1;
    pocket_z = headshell_thickness - 0.8;   // thin roof

    translate([spindle_dx, spindle_dy,
               pcb_bottom_z + pocket_z/2])
        cube([pocket_x, pocket_y, pocket_z], center = true);
}

// --------------------------
// Main arm
// --------------------------

module tone_arm() {
    difference() {
        union() {
            // Base mount at screw hole
            base_block();

            // Arm tube from block to headshell
            arm_start_x = base_block_length;             // from just ahead of screw
            arm_end_x   = spindle_dx - headshell_length/2;
            arm_len     = arm_end_x - arm_start_x;

            translate([(arm_start_x + arm_end_x) / 2,
                       spindle_dy / 2,
                       arm_centre_z])
                arm_tube(arm_len, arm_outer_diameter, arm_inner_diameter);

            // Small support between block and arm
            translate([base_block_length / 2 + 3,
                       spindle_dy / 2,
                       arm_centre_z / 2])
                cube([6, 10, arm_centre_z], center = true);

            // Headshell
            headshell();
        }

        // Cut PCB pocket
        as5600_pocket();
    }
}

// --------------------------
// Preview helpers
// --------------------------

$fn = 64;

// Show HDD lid as a plane at z = 0
color([0.8,0.8,0.8,0.3])
translate([0,0,-0.5])
cube([140, 140, 1], center = true);

// Screw hole marker at origin
color("blue")
translate([0,0,0])
cylinder(h = 2, d = 5, center = true);

// Spindle centre marker and magnet top
color("red")
translate([spindle_dx, spindle_dy, magnet_top_z])
cylinder(h = 1, d = 8, center = true);

// Tone arm
color("silver")
tone_arm();



