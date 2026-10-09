#include <imu_support.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <math.h>

#define SAMPLE_PERIOD_MS    10
#define CALIBRATION_SAMPLES 250 /* 2.5 seconds at 100 Hz */
#define RAD_TO_DEG          57.2957795f
#define GRAVITY_M_S2        9.80665f

static const struct device *const imu = DEVICE_DT_GET(DT_NODELABEL(mpu6050));
static int64_t sample_deadline_ms;
static int64_t previous_sample_us;
static unsigned int dropped_rows;

struct serial_message {
	struct imu_reading reading;
	int error;
	unsigned int dropped;
};

K_MSGQ_DEFINE(output_queue, sizeof(struct serial_message), 4, 4);

static void print_vector(const struct imu_xyz *value);

static int read_sensor(struct imu_reading *reading)
{
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
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

	reading->accel_m_s2.x = sensor_value_to_float(&accel[0]);
	reading->accel_m_s2.y = sensor_value_to_float(&accel[1]);
	reading->accel_m_s2.z = sensor_value_to_float(&accel[2]);
	/* Zephyr supplies angular velocity in rad/s; the demo uses degrees/s. */
	reading->gyro_raw_deg_s.x = sensor_value_to_float(&gyro[0]) * RAD_TO_DEG;
	reading->gyro_raw_deg_s.y = sensor_value_to_float(&gyro[1]) * RAD_TO_DEG;
	reading->gyro_raw_deg_s.z = sensor_value_to_float(&gyro[2]) * RAD_TO_DEG;
	return 0;
}

static void wait_until(int64_t *deadline_ms)
{
	*deadline_ms += SAMPLE_PERIOD_MS;
	if (*deadline_ms <= k_uptime_get()) {
		*deadline_ms = k_uptime_get() + SAMPLE_PERIOD_MS;
	}
	k_sleep(K_TIMEOUT_ABS_MS(*deadline_ms));
}

bool imu_sensor_ready(void)
{
	return device_is_ready(imu);
}

bool imu_calibrate_gyro(struct imu_xyz *bias, struct imu_reading *last)
{
	float gyro_sum[3] = {0};
	float gyro_square_sum[3] = {0};
	float accel_min[3] = {0};
	float accel_max[3] = {0};
	bool moved = false;
	int64_t deadline_ms = k_uptime_get();

	printk("# Calibration: hold the IMU still for 2.5 seconds.\n");
	for (int sample = 0; sample < CALIBRATION_SAMPLES; sample++) {
		wait_until(&deadline_ms);
		int rc = read_sensor(last);

		if (rc != 0) {
			printk("# Calibration read failed (%d); retrying.\n", rc);
			return false;
		}

		float gyro[3] = {last->gyro_raw_deg_s.x, last->gyro_raw_deg_s.y,
				 last->gyro_raw_deg_s.z};
		float accel[3] = {last->accel_m_s2.x, last->accel_m_s2.y,
				  last->accel_m_s2.z};
		float accel_norm_squared = 0.0f;

		for (int axis = 0; axis < 3; axis++) {
			gyro_sum[axis] += gyro[axis];
			gyro_square_sum[axis] += gyro[axis] * gyro[axis];
			accel_norm_squared += accel[axis] * accel[axis];
			if (sample == 0 || accel[axis] < accel_min[axis]) {
				accel_min[axis] = accel[axis];
			}
			if (sample == 0 || accel[axis] > accel_max[axis]) {
				accel_max[axis] = accel[axis];
			}
			if (fabsf(gyro[axis]) > 10.0f) {
				moved = true;
			}
		}
		if (fabsf(sqrtf(accel_norm_squared) - GRAVITY_M_S2) > 1.5f) {
			moved = true;
		}
	}

	float bias_value[3];
	for (int axis = 0; axis < 3; axis++) {
		bias_value[axis] = gyro_sum[axis] / CALIBRATION_SAMPLES;
		float variance = gyro_square_sum[axis] / CALIBRATION_SAMPLES -
			bias_value[axis] * bias_value[axis];

		if (variance > 0.8f * 0.8f || accel_max[axis] - accel_min[axis] > 0.8f) {
			moved = true;
		}
	}
	if (moved) {
		printk("# Motion detected; calibration rejected. Hold still for the retry.\n");
		return false;
	}

	bias->x = bias_value[0];
	bias->y = bias_value[1];
	bias->z = bias_value[2];
	printk("# Calibration accepted: no motion detected. Gyro bias X,Y,Z [deg/s]");
	print_vector(bias);
	printk("\n");
	return true;
}

void imu_start_sampling(void)
{
	sample_deadline_ms = k_uptime_get();
	previous_sample_us = k_ticks_to_us_floor64(k_uptime_ticks());
}

void imu_wait_for_sample(void)
{
	wait_until(&sample_deadline_ms);
}

int imu_read(struct imu_reading *reading, float *elapsed_seconds)
{
	int rc = read_sensor(reading);
	int64_t now_us = k_ticks_to_us_floor64(k_uptime_ticks());

	*elapsed_seconds = (now_us - previous_sample_us) / 1000000.0f;
	previous_sample_us = now_us;
	reading->time_ms = now_us / 1000;
	return rc;
}

static void print_decimal(float value)
{
	int64_t hundredths = (int64_t)(value * 100.0f + (value < 0 ? -0.5f : 0.5f));

	if (hundredths < 0) {
		printk("-");
		hundredths = -hundredths;
	}
	printk("%lld.%02lld", hundredths / 100, hundredths % 100);
}

static void print_vector(const struct imu_xyz *value)
{
	printk(",");
	print_decimal(value->x);
	printk(",");
	print_decimal(value->y);
	printk(",");
	print_decimal(value->z);
}

static void serial_output(void *unused1, void *unused2, void *unused3)
{
	struct serial_message message;

	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);
	while (1) {
		k_msgq_get(&output_queue, &message, K_FOREVER);
		if (message.error != 0) {
			printk("# ERROR: sample read failed (%d); reset to recalibrate and restart.\n",
			       message.error);
			continue;
		}
		if (message.dropped != 0) {
			printk("# Serial queue full; dropped %u rows since last report.\n",
			       message.dropped);
		}

		const struct imu_reading *r = &message.reading;
		struct imu_xyz angles = {r->roll_deg, r->pitch_deg, r->yaw_relative_deg};

		printk("DATA,%lld", r->time_ms);
		print_vector(&r->accel_m_s2);
		print_vector(&r->gyro_raw_deg_s);
		print_vector(&r->gyro_cal_deg_s);
		print_vector(&angles);
		printk("\n");
	}
}

K_THREAD_DEFINE(serial_thread, 1536, serial_output, NULL, NULL, NULL, 5, 0, 0);

void imu_print_stream_header(void)
{
	printk("# Complementary filter: gyro follows fast changes; gravity corrects roll/pitch.\n");
	printk("# Yaw is integrated gyro Z: drifting relative rotation, NOT absolute heading.\n");
	printk("# Raw gyro means scaled deg/s before bias subtraction, not register counts.\n");
	printk("# Columns: type,time_ms,ax_m_s2,ay_m_s2,az_m_s2,"
	       "gx_raw_deg_s,gy_raw_deg_s,gz_raw_deg_s,"
	       "gx_cal_deg_s,gy_cal_deg_s,gz_cal_deg_s,"
	       "roll_deg,pitch_deg,yaw_relative_drifting_deg\n");
}

void imu_publish(const struct imu_reading *reading)
{
	struct serial_message message = {
		.reading = *reading,
		.dropped = dropped_rows,
	};

	if (k_msgq_put(&output_queue, &message, K_NO_WAIT) != 0) {
		dropped_rows++;
	} else {
		dropped_rows = 0;
	}
}

void imu_publish_error(int error)
{
	struct serial_message message = {
		.error = error,
		.dropped = dropped_rows,
	};

	k_msgq_put(&output_queue, &message, K_FOREVER);
}
