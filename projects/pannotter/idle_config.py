# Set minimum period (in microseconds)
set_min_period(1000)
# Add tasks
add_task('rome_update', None)
add_task('display_update', 50000, 10)

