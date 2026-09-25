/* MPU-6050: startup gyro calibration and a simple complementary filter. */
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <math.h>
#include <stdint.h>

#define SAMPLE_MS           10
#define CALIBRATION_SAMPLES 250 /* 2.5 seconds at 100 Hz */
#define RAD_TO_DEG          57.2957795f
#define FILTER_TAU_S        0.5f
#define GRAVITY             9.80665f

static const struct device *const imu = DEVICE_DT_GET(DT_NODELABEL(mpu6050));

struct sample {
	float accel[3]; /* m/s^2 */
	float gyro[3];  /* degrees/s, before bias subtraction */
};

struct output {
	int64_t time_ms;
	struct sample raw;
	float gyro[3]; /* calibrated degrees/s */
	float angle[3]; /* roll, pitch, relative yaw in degrees */
	int error; /* A nonzero value requests a final status message. */
	unsigned int dropped;
};

/* UART output can take longer than a sample period. Keep it off the sampler. */
K_MSGQ_DEFINE(output_queue, sizeof(struct output), 4, 4);

static int read_sample(struct sample *s)
{
	struct sensor_value accel[3], gyro[3];
	int rc = sensor_sample_fetch(imu);

	if (rc == 0) {
		rc = sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, accel);
	}
	if (rc == 0) {
		rc = sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ, gyro);
	}
	if (rc != 0) {
		return rc;
	}
	for (int i = 0; i < 3; i++) {
		s->accel[i] = sensor_value_to_float(&accel[i]);
		/* Zephyr reports angular velocity in rad/s. */
		s->gyro[i] = sensor_value_to_float(&gyro[i]) * RAD_TO_DEG;
	}
	return 0;
}

/* Print two decimal places using integer formatting (no float printf needed). */
static void print_decimal(float value)
{
	int64_t hundredths = (int64_t)(value * 100.0f + (value < 0 ? -0.5f : 0.5f));

	if (hundredths < 0) {
		printk("-");
		hundredths = -hundredths;
	}
	printk("%lld.%02lld", hundredths / 100, hundredths % 100);
}

static void print_vector(const float value[3])
{
	for (int i = 0; i < 3; i++) {
		printk(",");
		print_decimal(value[i]);
	}
}

static void serial_output(void *a, void *b, void *c)
{
	struct output row;

	(void)a;
	(void)b;
	(void)c;
	while (1) {
		k_msgq_get(&output_queue, &row, K_FOREVER);
		if (row.error != 0) {
			printk("# ERROR: sample read failed (%d); reset to recalibrate and restart.\n",
			       row.error);
			printk("# Dropped serial rows: %u\n", row.dropped);
			continue;
		}
		if (row.dropped != 0) {
			printk("# Serial queue full; dropped %u rows since last report.\n",
			       row.dropped);
		}
		printk("DATA,%lld", row.time_ms);
		print_vector(row.raw.accel);
		print_vector(row.raw.gyro);
		print_vector(row.gyro);
		print_vector(row.angle);
		printk("\n");
	}
}

/* Main has priority 0; this lower-priority worker waits for completed rows. */
K_THREAD_DEFINE(serial_thread, 1536, serial_output, NULL, NULL, NULL, 5, 0, 0);

/* Absolute deadlines avoid adding the processing time to each 10 ms period. */
static void wait_for_sample(int64_t *deadline)
{
	*deadline += SAMPLE_MS;
	if (*deadline <= k_uptime_get()) {
		/* If late, start a new period instead of taking catch-up samples. */
		*deadline = k_uptime_get() + SAMPLE_MS;
	}
	k_sleep(K_TIMEOUT_ABS_MS(*deadline));
}

static bool calibrate(float bias[3], struct sample *last)
{
	float gyro_sum[3] = {0}, gyro_sq[3] = {0};
	float accel_min[3] = {0}, accel_max[3] = {0};
	bool moved = false;

	printk("# Calibration: hold the IMU still for 2.5 seconds.\n");
	int64_t deadline = k_uptime_get();

	for (int n = 0; n < CALIBRATION_SAMPLES; n++) {
		wait_for_sample(&deadline);
		int rc = read_sample(last);

		if (rc != 0) {
			printk("# Calibration read failed (%d); retrying.\n", rc);
			return false;
		}
		float norm_sq = 0;

		for (int i = 0; i < 3; i++) {
			float g = last->gyro[i];
			float a = last->accel[i];

			gyro_sum[i] += g;
			gyro_sq[i] += g * g;
			norm_sq += a * a;
			if (n == 0 || a < accel_min[i]) {
				accel_min[i] = a;
			}
			if (n == 0 || a > accel_max[i]) {
				accel_max[i] = a;
			}
			if (fabsf(g) > 10.0f) {
				moved = true;
			}
		}
		if (fabsf(sqrtf(norm_sq) - GRAVITY) > 1.5f) {
			moved = true;
		}
	}

	for (int i = 0; i < 3; i++) {
		bias[i] = gyro_sum[i] / CALIBRATION_SAMPLES;
		float variance = gyro_sq[i] / CALIBRATION_SAMPLES - bias[i] * bias[i];

		/* Motion heuristic: gyro SD > 0.8 deg/s or accel span > 0.8 m/s^2.
		 * Very slow, steady rotation can look like bias: stillness matters! */
		if (variance > 0.8f * 0.8f || accel_max[i] - accel_min[i] > 0.8f) {
			moved = true;
		}
	}
	if (moved) {
		printk("# Motion detected; calibration rejected. Hold still for the retry.\n");
		return false;
	}
	printk("# Calibration accepted: no motion detected. Gyro bias X,Y,Z [deg/s]");
	print_vector(bias);
	printk("\n");
	return true;
}

static void accel_angles(const struct sample *s, float *roll, float *pitch)
{
	*roll = atan2f(s->accel[1], s->accel[2]) * RAD_TO_DEG;
	*pitch = atan2f(-s->accel[0],
		       sqrtf(s->accel[1] * s->accel[1] + s->accel[2] * s->accel[2])) * RAD_TO_DEG;
}

/* Take the short way around the +/-180-degree roll boundary. */
static float wrap_angle(float angle)
{
	while (angle > 180.0f) {
		angle -= 360.0f;
	}
	while (angle < -180.0f) {
		angle += 360.0f;
	}
	return angle;
}

int main(void)
{
	struct output row = {0};
	float bias[3];
	unsigned int samples = 0;
	unsigned int dropped = 0;

	if (!device_is_ready(imu)) {
		printk("# ERROR: MPU-6050 not ready; check power, wiring and address 0x68.\n");
		return 0;
	}
	printk("# Hold the IMU still. Calibration starts in one second.\n");
	k_sleep(K_SECONDS(1));
	while (!calibrate(bias, &row.raw)) {
		k_sleep(K_SECONDS(1));
	}

	accel_angles(&row.raw, &row.angle[0], &row.angle[1]);
	row.angle[2] = 0; /* Relative yaw starts at zero after each calibration. */
	printk("# Simple body-axis filter: use moderate tilts; yaw is most useful near level.\n");
	printk("# Yaw is integrated gyro Z: drifting relative rotation, NOT absolute heading.\n");
	printk("# Raw gyro means scaled deg/s before bias subtraction, not register counts.\n");
	printk("# Columns: type,time_ms,ax_m_s2,ay_m_s2,az_m_s2,"
	       "gx_raw_deg_s,gy_raw_deg_s,gz_raw_deg_s,"
	       "gx_cal_deg_s,gy_cal_deg_s,gz_cal_deg_s,"
	       "roll_deg,pitch_deg,yaw_relative_drifting_deg\n");

	int64_t deadline = k_uptime_get();
	int64_t previous_us = k_ticks_to_us_floor64(k_uptime_ticks());

	while (1) {
		wait_for_sample(&deadline);
		int rc = read_sample(&row.raw);
		int64_t now_us = k_ticks_to_us_floor64(k_uptime_ticks());
		float dt = (now_us - previous_us) / 1000000.0f;

		previous_us = now_us;
		if (rc != 0) {
			/* Send the error through the writer to avoid interrupting a DATA line.
			 * Stop rather than integrating stale measurements across a gap. */
			row.error = rc;
			row.dropped = dropped;
			k_msgq_put(&output_queue, &row, K_FOREVER);
			return 0;
		}
		for (int i = 0; i < 3; i++) {
			row.gyro[i] = row.raw.gyro[i] - bias[i];
		}

		float accel_roll, accel_pitch;
		float alpha = FILTER_TAU_S / (FILTER_TAU_S + dt);

		accel_angles(&row.raw, &accel_roll, &accel_pitch);
		/* Gyro predicts the short-term change; gravity corrects slow tilt drift.
		 * This simple gx/gy approximation is not a full 3-D attitude estimator.
		 * Linear acceleration also disturbs the gravity-based tilt estimate. */
		float predicted_roll = row.angle[0] + row.gyro[0] * dt;

		row.angle[0] = wrap_angle(predicted_roll +
			(1 - alpha) * wrap_angle(accel_roll - predicted_roll));
		row.angle[1] = alpha * (row.angle[1] + row.gyro[1] * dt) +
			(1 - alpha) * accel_pitch;
		row.angle[2] += row.gyro[2] * dt;
		row.time_ms = now_us / 1000; /* Time since boot, not nominal sample count. */

		/* Send every second sample: about 50 rows/s from 100 samples/s. */
		if (++samples % 2 == 0) {
			row.dropped = dropped;
			if (k_msgq_put(&output_queue, &row, K_NO_WAIT) != 0) {
				dropped++; /* Never stall sampling if serial output falls behind. */
			} else {
				dropped = 0;
			}
		}
	}
}
