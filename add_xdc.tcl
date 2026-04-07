# =============================================================================
# add_xdc.tcl — Run this in the Vivado Tcl Console to add the XDC constraint
# file and regenerate the bitstream.
#
# Usage: In Vivado Tcl Console, type:
#   source C:/Users/USER/OneDrive/Desktop/smart_power_grid/add_xdc.tcl
# =============================================================================

# Step 1: Add the XDC to the constraints fileset
add_files -fileset constrs_1 -norecurse {C:/Users/USER/OneDrive/Desktop/smart_power_grid/src/basys3_constraints.xdc}

# Step 2: Set it as the target constraint file
set_property target_constrs_file {C:/Users/USER/OneDrive/Desktop/smart_power_grid/src/basys3_constraints.xdc} [current_fileset -constrset]

# Step 3: Verify it was added (prints list of constraint files)
puts "===== Constraint files in project: ====="
foreach f [get_files -of_objects [get_filesets constrs_1]] {
    puts "  $f"
}
puts "========================================"

# Step 4: Reset and re-launch everything from scratch
reset_runs synth_1
launch_runs synth_1 -jobs 4
wait_on_run synth_1

launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1

puts "===== DONE! Check impl_1 for bitstream. ====="
