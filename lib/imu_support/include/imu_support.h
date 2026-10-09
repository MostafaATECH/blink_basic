#ifndef IMU_SUPPORT_H
#define IMU_SUPPORT_H

#include <stdbool.h>
#include <stdint.h>

/* A named X/Y/Z vector makes units and axis use clear in the application. */
struct imu_xyz {
	float x;
	float y;
	float z;
};

/* One complete row of data produced by the firmware. */
struct imu_reading {
	int64_t time_ms;
	struct imu_xyz accel_m_s2;
	struct imu_xyz gyro_raw_deg_s;
	struct imu_xyz gyro_cal_deg_s;
	float roll_deg;
	float pitch_deg;
	float yaw_relative_deg;
};

bool imu_sensor_ready(void);

/* Returns false and asks for a retry if motion is detected. */
bool imu_calibrate_gyro(struct imu_xyz *bias, struct imu_reading *last_reading);

/* Sampling uses measured elapsed time and a 10 ms (about 100 Hz) schedule. */
void imu_start_sampling(void);
void imu_wait_for_sample(void);
int imu_read(struct imu_reading *reading, float *elapsed_seconds);

/* Output is queued so UART printing never delays sensor sampling. */
void imu_print_stream_header(void);
void imu_publish(const struct imu_reading *reading);
void imu_publish_error(int error);

#endif /* IMU_SUPPORT_H */
