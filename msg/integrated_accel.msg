uint64 timestamp	# time since system start (microseconds)

float32 integrated_x		# rough integration of the acceleration in the NED X board axis in m/s
float32 integrated_y		# rough integration of the acceleration in the NED Y board axis in m/s
float32 integrated_z		# rough integration of the acceleration in the NED Z board axis in m/s

uint32 integral_dt	# integration time  (microseconds)

bool armed  # armed status flag

# TOPICS integrated_accel
