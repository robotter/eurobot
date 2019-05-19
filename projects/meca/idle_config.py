# Set minimum period (in microseconds)
set_min_period(1000)
# Add tasks
add_task('rome_strat_update', 10000, 1)
add_task('rome_jevois_update', None)
add_task('rome_telemetry', 100000, 2)
add_task('arm_l_update', None)
add_task('arm_r_update', None)

