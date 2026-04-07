open_project smart_power_grid.xpr
reset_run synth_1
launch_runs synth_1 -jobs 6
wait_on_run synth_1
puts "Synthesis Status: [get_property STATUS [get_runs synth_1]]"
exit 0
