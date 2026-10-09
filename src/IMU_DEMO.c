/*
 * MPU-6050 teaching example: calibrate the gyro, then estimate orientation.
 * Detailed sensor access and serial formatting are in the imu_support library.
 */
#include <imu_support.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <math.h>

#define RAD_TO_DEG             57.2957795f
#define FILTER_TIME_CONSTANT_S 0.5f
#define OUTPUT_EVERY_N_SAMPLES 2 /* 100 Hz / 2 = about 50 Hz */

/* Keep roll continuous when it crosses from +180 to -180 degrees. */
static float wrap_angle(float angle)
{
	if (angle > 180.0f) {
		angle -= 360.0f;
	} else if (angle < -180.0f) {
		angle += 360.0f;
	}
	return angle;
}

int main(void)
{
	struct imu_reading data = {0};
	struct imu_xyz gyro_bias;
	unsigned int sample_number = 0;

	if (!imu_sensor_ready()) {
		printk("# ERROR: MPU-6050 not ready; check power, wiring and address 0x68.\n");
		return 0;
	}

	/* A stationary average measures the gyro's zero-rate error (bias). */
	printk("# Hold the IMU still. Calibration starts in one second.\n");
	k_sleep(K_SECONDS(1));
	while (!imu_calibrate_gyro(&gyro_bias, &data)) {
		k_sleep(K_SECONDS(1)); /* Calibration repeats if the sensor moved. */
	}

	/* At rest, the accelerometer sees gravity, giving initial roll and pitch. */
	data.roll_deg = atan2f(data.accel_m_s2.y, data.accel_m_s2.z) * RAD_TO_DEG;
	data.pitch_deg = atan2f(-data.accel_m_s2.x,
		sqrtf(data.accel_m_s2.y * data.accel_m_s2.y +
		       data.accel_m_s2.z * data.accel_m_s2.z)) * RAD_TO_DEG;
	data.yaw_relative_deg = 0.0f;

	imu_print_stream_header();
	imu_start_sampling();

	while (1) {
		float dt;

		imu_wait_for_sample(); /* One new reading every 10 ms: about 100 Hz. */
		int rc = imu_read(&data, &dt);
		if (rc != 0) {
			imu_publish_error(rc);
			return 0;
		}

		/* Calibration means subtracting the measured bias from each gyro axis. */
		data.gyro_cal_deg_s.x = data.gyro_raw_deg_s.x - gyro_bias.x;
		data.gyro_cal_deg_s.y = data.gyro_raw_deg_s.y - gyro_bias.y;
		data.gyro_cal_deg_s.z = data.gyro_raw_deg_s.z - gyro_bias.z;

		float accel_roll = atan2f(data.accel_m_s2.y, data.accel_m_s2.z) * RAD_TO_DEG;
		float accel_pitch = atan2f(-data.accel_m_s2.x,
			sqrtf(data.accel_m_s2.y * data.accel_m_s2.y +
			       data.accel_m_s2.z * data.accel_m_s2.z)) * RAD_TO_DEG;
		float alpha = FILTER_TIME_CONSTANT_S / (FILTER_TIME_CONSTANT_S + dt);

		/* Gyro follows fast rotation; accelerometer slowly corrects roll/pitch drift. */
		float predicted_roll = data.roll_deg + data.gyro_cal_deg_s.x * dt;
		data.roll_deg = wrap_angle(predicted_roll +
			(1.0f - alpha) * wrap_angle(accel_roll - predicted_roll));
		data.pitch_deg = alpha * (data.pitch_deg + data.gyro_cal_deg_s.y * dt) +
			(1.0f - alpha) * accel_pitch;

		/* MPU-6050 has no compass, so yaw is relative and slowly drifts. */
		data.yaw_relative_deg += data.gyro_cal_deg_s.z * dt;

		if (++sample_number % OUTPUT_EVERY_N_SAMPLES == 0) {
			imu_publish(&data);
		}
	}
}
