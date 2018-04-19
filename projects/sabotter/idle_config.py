# Set minimum period (in microseconds)
set_min_period(100)  # 0.1ms
# Add tasks
add_task('rome_update', None)
add_task('rome_telemetry', 200000, 1)

