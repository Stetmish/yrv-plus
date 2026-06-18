# 50 MHz main clock
create_clock -period "50.0 MHz" [get_ports max10_clk1_50]

derive_clock_uncertainty

set_false_path -from [get_ports {key[*]}]  -to [all_clocks]
set_false_path -from [get_ports {sw[*]}]   -to [all_clocks]
set_false_path -from [get_ports {tmd_gpio[*]}] -to [all_clocks]

set_false_path -from * -to [get_ports {led[*]}]
set_false_path -from * -to [get_ports {rgb_led_data}]
set_false_path -from * -to [get_ports {tmd_gpio[*]}]