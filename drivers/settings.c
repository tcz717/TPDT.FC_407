#include "settings.h"
#include <dfs_posix.h>
#include "ahrs.h"
#include <finsh.h>

extern PID p_rate_pid, r_rate_pid, y_rate_pid;
extern PID p_angle_pid, r_angle_pid, y_angle_pid;
extern PID x_v_pid, y_v_pid;
extern PID x_d_pid, y_d_pid;
extern PID h_v_pid;

struct setting_t settings;

rt_err_t load_settings(struct setting_t * s, const char * path,
	PID * p_angle, PID * p_rate,
	PID * r_angle, PID * r_rate,
	PID * y_angle, PID * y_rate,
	PID * x_d, PID * x_v,
	PID * y_d, PID * y_v,
	PID * h)
{
	int fd;
	rt_kprintf("start load settings\n");

	fd = open(path, O_RDWR | O_CREAT, 0);

	if (fd >= 0)
	{
		if (read(fd, &settings, sizeof(settings)) != sizeof(settings) ||
			settings.magic != SETTING_MAGIC)
		{
			rt_kprintf("read error,use default settings and save it.\n");
			s->magic = SETTING_MAGIC;

			s->pitch_angle_p = p_angle->p;
			s->pitch_rate_p = p_rate->p;
			s->pitch_rate_i = p_rate->i;
			s->pitch_rate_d = p_rate->d;

			s->roll_angle_p = r_angle->p;
			s->roll_rate_p = r_rate->p;
			s->roll_rate_i = r_rate->i;
			s->roll_rate_d = r_rate->d;

			s->yaw_angle_p = y_angle->p;
			s->yaw_rate_p = y_rate->p;
			s->yaw_rate_i = y_rate->i;
			s->yaw_rate_d = y_rate->d;

			s->h_p = h->p;
			s->h_i = h->i;
			s->h_d = h->d;

			write(fd, s, sizeof(struct setting_t));
			close(fd);
			return -RT_EIO;
		}
		else
		{
			p_angle->p = s->pitch_angle_p;
			p_rate->p = s->pitch_rate_p;
			p_rate->i = s->pitch_rate_i;
			p_rate->d = s->pitch_rate_d;

			r_angle->p = s->roll_angle_p;
			r_rate->p = s->roll_rate_p;
			r_rate->i = s->roll_rate_i;
			r_rate->d = s->roll_rate_d;

			y_angle->p = s->yaw_angle_p;
			y_rate->p = s->yaw_rate_p;
			y_rate->i = s->yaw_rate_i;
			y_rate->d = s->yaw_rate_d;

			x_d->p = s->x_d_p;
			x_v->p = s->x_v_p;
			x_v->i = s->x_v_i;
			x_v->d = s->x_v_d;

			y_d->p = s->y_d_p;
			y_v->p = s->y_v_p;
			y_v->i = s->y_v_i;
			y_v->d = s->y_v_d;

			h->p = s->h_p;
			h->i = s->h_i;
			h->d = s->h_d;

			rt_kprintf("settings load succeed.\n");
		}
		close(fd);
	}
	else
	{
		rt_kprintf("load error,use default settings.\n");
		return -RT_EIO;
	}
	return RT_EOK;
}

rt_err_t save_settings(struct setting_t * s, const char * path)
{
	int fd;

	fd = open(path, O_WRONLY | O_TRUNC, 0);

	if (fd >= 0)
	{
		write(fd, s, sizeof(struct setting_t));
		close(fd);
		return RT_EOK;
	}
	else
	{
		return -RT_EIO;
	}
}

void save()
{
	save_settings(&settings, "/setting");

	get_pid();

	rt_kprintf("pitch:		%d	%d	%d.\n", settings.pitch_min,
		settings.pitch_mid,
		settings.pitch_max);
	rt_kprintf("roll:		%d	%d	%d.\n", settings.roll_min,
		settings.roll_mid,
		settings.roll_max);
	rt_kprintf("yaw:		%d	%d	%d.\n", settings.yaw_min,
		settings.yaw_mid,
		settings.yaw_max);
	rt_kprintf("throttle:	%d	%d.\n", settings.th_min,
		settings.th_max);
}
FINSH_FUNCTION_EXPORT(save, save all config)

void get_pid()
{
	rt_kprintf("pitch_angle:\t\t%d.%d.\n", (s32)p_angle_pid.p, (s32)(p_angle_pid.p*10.0f) % 10);
	rt_kprintf("pitch_rate:		%d.%d	%d.%02d	%d.%03d.\n", 
		(s32)p_rate_pid.p, (s32)(p_rate_pid.p*10.0f) % 10,
		(s32)p_rate_pid.i, (s32)(p_rate_pid.i*100.0f) % 100,
		(s32)p_rate_pid.d, (s32)(p_rate_pid.d*1000.0f) % 1000);

	rt_kprintf("roll_angle:		%d.%d.\n", (s32)r_angle_pid.p, (s32)(r_angle_pid.p*10.0f) % 10);
	rt_kprintf("roll_rate:		%d.%d	%d.%02d	%d.%03d.\n", 
		(s32)r_rate_pid.p, (s32)(r_rate_pid.p*10.0f) % 10,
		(s32)r_rate_pid.i, (s32)(r_rate_pid.i*100.0f) % 100,
		(s32)r_rate_pid.d, (s32)(r_rate_pid.d*1000.0f) % 1000);

	rt_kprintf("yaw_angle:		%d.%d.\n", (s32)y_angle_pid.p, (s32)(y_angle_pid.p*10.0f) % 10);
	rt_kprintf("yaw_rate:		%d.%d	%d.%02d	%d.%03d.\n", 
		(s32)y_rate_pid.p, (s32)(y_rate_pid.p*10.0f) % 10,
		(s32)y_rate_pid.i, (s32)(y_rate_pid.i*100.0f) % 100,
		(s32)y_rate_pid.d, (s32)(y_rate_pid.d*1000.0f) % 1000);

	rt_kprintf("x_axis_d:		%d.%02d.\n", (s32)x_d_pid.p, (s32)(x_d_pid.p*100.0f) % 100);
	rt_kprintf("x_axis_v:		%d.%d	%d.%02d	%d.%03d.\n",
		(s32)x_v_pid.p, (s32)(x_v_pid.p*10.0f) % 10,
		(s32)x_v_pid.i, (s32)(x_v_pid.i*100.0f) % 100,
		(s32)x_v_pid.d, (s32)(x_v_pid.d*1000.0f) % 1000);

	rt_kprintf("y_axis_d:		%d.%02d.\n", (s32)y_d_pid.p, (s32)(y_d_pid.p*100.0f) % 100);
	rt_kprintf("y_axis_v:		%d.%d	%d.%02d	%d.%03d.\n",
		(s32)y_v_pid.p, (s32)(y_v_pid.p*10.0f) % 10,
		(s32)y_v_pid.i, (s32)(y_v_pid.i*100.0f) % 100,
		(s32)y_v_pid.d, (s32)(y_v_pid.d*1000.0f) % 1000);

	rt_kprintf("height	:		%d.%d	%d.%02d	%d.%03d.\n", (s32)h_v_pid.p, (s32)(h_v_pid.p*10.0f) % 10,
		(s32)h_v_pid.i, (s32)(h_v_pid.i*100.0f) % 100,
		(s32)h_v_pid.d, (s32)(h_v_pid.d*1000.0f) % 1000);
}
FINSH_FUNCTION_EXPORT(get_pid, show the value of pid)

void set_pitch(s16 ap, s16 p, s16 i, s16 d)
{
	PID_Init(&p_angle_pid, ap / 10.0f, 0.0f, 0.0f);
	PID_Init(&p_rate_pid, p / 10.0f, i / 100.0f, d / 1000.0f);

	settings.pitch_angle_p = p_angle_pid.p;
	settings.pitch_rate_p = p_rate_pid.p;
	settings.pitch_rate_i = p_rate_pid.i;
	settings.pitch_rate_d = p_rate_pid.d;

	save_settings(&settings, "/setting");

	get_pid();
}
FINSH_FUNCTION_EXPORT(set_pitch, set the value of pid in pitch)

void set_roll(s16 ap, s16 p, s16 i, s16 d)
{
	PID_Init(&r_angle_pid, ap / 10.0f, 0.0f, 0.0f);
	PID_Init(&r_rate_pid, p / 10.0f, i / 100.0f, d / 1000.0f);

	settings.roll_angle_p = r_angle_pid.p;
	settings.roll_rate_p = r_rate_pid.p;
	settings.roll_rate_i = r_rate_pid.i;
	settings.roll_rate_d = r_rate_pid.d;

	save_settings(&settings, "/setting");

	get_pid();
}
FINSH_FUNCTION_EXPORT(set_roll, set the value of pid in roll)

void set_yaw(s16 ap, s16 p, s16 i, s16 d)
{
	PID_Init(&y_angle_pid, ap / 10.0f, 0.0f, 0.0f);
	PID_Init(&y_rate_pid, p / 10.0f, i / 100.0f, d / 1000.0f);

	settings.yaw_angle_p = y_angle_pid.p;
	settings.yaw_rate_p = y_rate_pid.p;
	settings.yaw_rate_i = y_rate_pid.i;
	settings.yaw_rate_d = y_rate_pid.d;

	save_settings(&settings, "/setting");

	get_pid();
}
FINSH_FUNCTION_EXPORT(set_yaw, set the value of pid in yaw)

void set_x(s16 ap, s16 p, s16 i, s16 d)
{
	PID_Init(&x_d_pid, ap / 100.0f, 0.0f, 0.0f);
	PID_Init(&x_v_pid, p / 10.0f, i / 100.0f, d / 1000.0f);

	settings.x_d_p = x_d_pid.p;
	settings.x_v_p = x_v_pid.p;
	settings.x_v_i = x_v_pid.i;
	settings.x_v_d = x_v_pid.d;

	save_settings(&settings, "/setting");

	get_pid();
}
FINSH_FUNCTION_EXPORT(set_x, set the value of pid in x axis)

void set_y(s16 ap, s16 p, s16 i, s16 d)
{
	PID_Init(&y_d_pid, ap / 100.0f, 0.0f, 0.0f);
	PID_Init(&y_v_pid, p / 10.0f, i / 100.0f, d / 1000.0f);

	settings.y_d_p = y_d_pid.p;
	settings.y_v_p = y_v_pid.p;
	settings.y_v_i = y_v_pid.i;
	settings.y_v_d = y_v_pid.d;

	save_settings(&settings, "/setting");

	get_pid();
}
FINSH_FUNCTION_EXPORT(set_y, set the value of pid in y axis)

void set_height(s16 p, s16 i, s16 d)
{
	PID_Init(&h_v_pid, p / 10.0f, i / 100.0f, d / 1000.0f);

	settings.h_p = h_v_pid.p;
	settings.h_i = h_v_pid.i;
	settings.h_d = h_v_pid.d;

	save_settings(&settings, "/setting");

	get_pid();
}
FINSH_FUNCTION_EXPORT(set_height, set the value of pid in yaw)

void set_pwm()
{
	settings.pwm_init_mode = settings.pwm_init_mode ? 0 : 0xAF;

	save_settings(&settings, "/setting");

	if (settings.pwm_init_mode)
		rt_kprintf("warning:after reset, we will set moter out full!\n");
}
FINSH_FUNCTION_EXPORT(set_pwm, set pwm to the same)

extern u8 en_out_ahrs;
void out_ahrs()
{
	en_out_ahrs = !en_out_ahrs;
}
FINSH_FUNCTION_EXPORT(out_ahrs, output the ahrs)

extern u8 en_out_pwm;
void out_pwm()
{
	en_out_pwm = !en_out_pwm;
}
FINSH_FUNCTION_EXPORT(out_pwm, output the pwm)

static void calculate_speed_print(rt_uint32_t speed)
{
	rt_uint32_t k, m;

	k = speed / 1024UL;
	if (k)
	{
		m = k / 1024UL;
		if (m)
		{
			rt_kprintf("%d.%dMbyte/s", m, k % 1024UL * 100 / 1024UL);
		}
		else
		{
			rt_kprintf("%d.%dKbyte/s", k, speed % 1024UL * 100 / 1024UL);
		}
	}
	else
	{
		rt_kprintf("%dbyte/s", speed);
	}
}

static rt_err_t _block_device_test(rt_device_t device)
{
	rt_err_t result;
	struct rt_device_blk_geometry geometry;
	rt_uint8_t * read_buffer = RT_NULL;
	rt_uint8_t * write_buffer = RT_NULL;

	rt_kprintf("\r\n");

	if ((device->flag & RT_DEVICE_FLAG_RDWR) == RT_DEVICE_FLAG_RDWR)
	{
		// device can read and write.
		// step 1: open device
		result = rt_device_open(device, RT_DEVICE_FLAG_RDWR);
		if (result != RT_EOK)
		{
			return result;
		}

		// step 2: get device info
		rt_memset(&geometry, 0, sizeof(geometry));
		result = rt_device_control(device,
			RT_DEVICE_CTRL_BLK_GETGEOME,
			&geometry);
		if (result != RT_EOK)
		{
			rt_kprintf("device : %s cmd RT_DEVICE_CTRL_BLK_GETGEOME failed.\r\n");
			return result;
		}
		rt_kprintf("device info:\r\n");
		rt_kprintf("sector  size : %d byte\r\n", geometry.bytes_per_sector);
		rt_kprintf("sector count : %d \r\n", geometry.sector_count);
		rt_kprintf("block   size : %d byte\r\n", geometry.block_size);

		rt_kprintf("\r\n");
		read_buffer = rt_malloc(geometry.bytes_per_sector);
		if (read_buffer == RT_NULL)
		{
			rt_kprintf("no memory for read_buffer!\r\n");
			goto __return;
		}
		write_buffer = rt_malloc(geometry.bytes_per_sector);
		if (write_buffer == RT_NULL)
		{
			rt_kprintf("no memory for write_buffer!\r\n");
			goto __return;
		}

		/* step 3:  R/W test */
		{
			rt_uint32_t i, err_count, sector_no;
			rt_uint8_t * data_point;

			i = rt_device_read(device, 0, read_buffer, 1);
			if (i != 1)
			{
				rt_kprintf("read device :%s ", device->parent.name);
				rt_kprintf("the first sector failed.\r\n");
				goto __return;
			}

			data_point = write_buffer;
			for (i = 0; i < geometry.bytes_per_sector; i++)
			{
				*data_point++ = (rt_uint8_t)i;
			}

			/* write first sector */
			sector_no = 0;
			data_point = write_buffer;
			*data_point++ = (rt_uint8_t)sector_no;
			i = rt_device_write(device, sector_no, write_buffer, 1);
			if (i != 1)
			{
				rt_kprintf("read the first sector success!\r\n");
				rt_kprintf("but write device :%s ", device->parent.name);
				rt_kprintf("the first sector failed.\r\n");
				rt_kprintf("maybe readonly!\r\n");
				goto __return;
			}

			/* write the second sector */
			sector_no = 1;
			data_point = write_buffer;
			*data_point++ = (rt_uint8_t)sector_no;
			i = rt_device_write(device, sector_no, write_buffer, 1);
			if (i != 1)
			{
				rt_kprintf("write device :%s ", device->parent.name);
				rt_kprintf("the second sector failed.\r\n");
				goto __return;
			}

			/* write the end sector */
			sector_no = geometry.sector_count - 1;
			data_point = write_buffer;
			*data_point++ = (rt_uint8_t)sector_no;
			i = rt_device_write(device, sector_no, write_buffer, 1);
			if (i != 1)
			{
				rt_kprintf("write device :%s ", device->parent.name);
				rt_kprintf("the end sector failed.\r\n");
				goto __return;
			}

			/* verify first sector */
			sector_no = 0;
			i = rt_device_read(device, sector_no, read_buffer, 1);
			if (i != 1)
			{
				rt_kprintf("read device :%s ", device->parent.name);
				rt_kprintf("the first sector failed.\r\n");
				goto __return;
			}
			err_count = 0;
			data_point = read_buffer;
			if ((*data_point++) != (rt_uint8_t)sector_no)
			{
				err_count++;
			}
			for (i = 1; i < geometry.bytes_per_sector; i++)
			{
				if ((*data_point++) != (rt_uint8_t)i)
				{
					err_count++;
				}
			}
			if (err_count > 0)
			{
				rt_kprintf("verify device :%s ", device->parent.name);
				rt_kprintf("the first sector failed.\r\n");
				goto __return;
			}

			/* verify sector sector */
			sector_no = 1;
			i = rt_device_read(device, sector_no, read_buffer, 1);
			if (i != 1)
			{
				rt_kprintf("read device :%s ", device->parent.name);
				rt_kprintf("the second sector failed.\r\n");
				goto __return;
			}
			err_count = 0;
			data_point = read_buffer;
			if ((*data_point++) != (rt_uint8_t)sector_no)
			{
				err_count++;
			}
			for (i = 1; i < geometry.bytes_per_sector; i++)
			{
				if ((*data_point++) != (rt_uint8_t)i)
				{
					err_count++;
				}
			}
			if (err_count > 0)
			{
				rt_kprintf("verify device :%s ", device->parent.name);
				rt_kprintf("the second sector failed.\r\n");
				goto __return;
			}

			/* verify the end sector */
			sector_no = geometry.sector_count - 1;
			i = rt_device_read(device, sector_no, read_buffer, 1);
			if (i != 1)
			{
				rt_kprintf("read device :%s ", device->parent.name);
				rt_kprintf("the end sector failed.\r\n");
				goto __return;
			}
			err_count = 0;
			data_point = read_buffer;
			if ((*data_point++) != (rt_uint8_t)sector_no)
			{
				err_count++;
			}
			for (i = 1; i < geometry.bytes_per_sector; i++)
			{
				if ((*data_point++) != (rt_uint8_t)i)
				{
					err_count++;
				}
			}
			if (err_count > 0)
			{
				rt_kprintf("verify device :%s ", device->parent.name);
				rt_kprintf("the end sector failed.\r\n");
				goto __return;
			}
			rt_kprintf("device R/W test pass!\r\n");
		} /* step 3: I/O R/W test */

		rt_kprintf("\r\nRT_TICK_PER_SECOND:%d\r\n", RT_TICK_PER_SECOND);

		// step 4: continuous single sector speed test
		{
			rt_uint32_t tick_start, tick_end;
			rt_uint32_t i;

			rt_kprintf("\r\ncontinuous single sector speed test:\r\n");

			if (geometry.sector_count < 10)
			{
				rt_kprintf("device sector_count < 10, speed test abort!\r\n");
			}
			else
			{
				unsigned int sector;

				// sign sector write
				rt_kprintf("write: ");
				sector = 0;
				tick_start = rt_tick_get();
				for (i = 0; i < 200; i++)
				{
					sector += rt_device_write(device, i, read_buffer, 1);
					if ((i != 0) && ((i % 4) == 0))
					{
						if (sector < 4)
						{
							rt_kprintf("#");
						}
						else
						{
							rt_kprintf("<");
						}
						sector = 0;
					}
				}
				tick_end = rt_tick_get();
				rt_kprintf("\r\nwrite 200 sector from %d to %d, ", tick_start, tick_end);
				calculate_speed_print((geometry.bytes_per_sector * 200UL*RT_TICK_PER_SECOND) / (tick_end - tick_start));
				rt_kprintf("\r\n");

				// sign sector read
				rt_kprintf("read : ");
				sector = 0;
				tick_start = rt_tick_get();
				for (i = 0; i < 200; i++)
				{
					sector += rt_device_read(device, i, read_buffer, 1);
					if ((i != 0) && ((i % 4) == 0))
					{
						if (sector < 4)
						{
							rt_kprintf("#");
						}
						else
						{
							rt_kprintf(">");
						}
						sector = 0;
					}
				}
				tick_end = rt_tick_get();
				rt_kprintf("\r\nread 200 sector from %d to %d, ", tick_start, tick_end);
				calculate_speed_print((geometry.bytes_per_sector * 200UL*RT_TICK_PER_SECOND) / (tick_end - tick_start));
				rt_kprintf("\r\n");
			}
		}// step 4: speed test

		// step 5: random single sector speed test
		{
			rt_uint32_t tick_start, tick_end;
			rt_uint32_t i;

			rt_kprintf("\r\nrandom single sector speed test:\r\n");

			if (geometry.sector_count < 10)
			{
				rt_kprintf("device sector_count < 10, speed test abort!\r\n");
			}
			else
			{
				unsigned int sector;

				// sign sector write
				rt_kprintf("write: ");
				sector = 0;
				tick_start = rt_tick_get();
				for (i = 0; i < 200; i++)
				{
					sector += rt_device_write(device, (geometry.sector_count / 10) * (i % 10) + (i % 10), read_buffer, 1);
					if ((i != 0) && ((i % 4) == 0))
					{
						if (sector < 4)
						{
							rt_kprintf("#");
						}
						else
						{
							rt_kprintf("<");
						}
						sector = 0;
					}
				}
				tick_end = rt_tick_get();
				rt_kprintf("\r\nwrite 200 sector from %d to %d, ", tick_start, tick_end);
				calculate_speed_print((geometry.bytes_per_sector * 200UL*RT_TICK_PER_SECOND) / (tick_end - tick_start));
				rt_kprintf("\r\n");

				// sign sector read
				rt_kprintf("read : ");
				sector = 0;
				tick_start = rt_tick_get();
				for (i = 0; i < 200; i++)
				{
					sector += rt_device_read(device, (geometry.sector_count / 10) * (i % 10) + (i % 10), read_buffer, 1);
					if ((i != 0) && ((i % 4) == 0))
					{
						if (sector < 4)
						{
							rt_kprintf("#");
						}
						else
						{
							rt_kprintf(">");
						}
						sector = 0;
					}
				}
				tick_end = rt_tick_get();
				rt_kprintf("\r\nread 200 sector from %d to %d, ", tick_start, tick_end);
				calculate_speed_print((geometry.bytes_per_sector * 200UL*RT_TICK_PER_SECOND) / (tick_end - tick_start));
				rt_kprintf("\r\n");
			}
		}// step 4: speed test

		/* step 6: multiple sector speed test */
		{
			rt_uint8_t * multiple_buffer;
			rt_uint8_t * ptr;
			rt_uint32_t tick_start, tick_end;
			rt_uint32_t sector, i;

			rt_kprintf("\r\nmultiple sector speed test\r\n");

			for (sector = 2; sector < 256; sector = sector * 2)
			{
				multiple_buffer = rt_malloc(geometry.bytes_per_sector * sector);

				if (multiple_buffer == RT_NULL)
				{
					rt_kprintf("no memory for %d sector! multiple sector speed test abort!\r\n", sector);
					break;
				}

				rt_memset(multiple_buffer, sector, geometry.bytes_per_sector * sector);
				rt_kprintf("write: ");
				tick_start = rt_tick_get();
				for (i = 0; i < 10; i++)
				{
					rt_size_t n;
					n = rt_device_write(device, 50, multiple_buffer, sector);
					if (n == sector)
					{
						rt_kprintf("<");
					}
					else
					{
						rt_kprintf("#");
					}
				}
				tick_end = rt_tick_get();
				rt_kprintf("\r\n");
				rt_kprintf("multiple write %d sector speed : ", sector);
				calculate_speed_print((geometry.bytes_per_sector * sector * 10 * RT_TICK_PER_SECOND) / (tick_end - tick_start));
				rt_kprintf("\r\n");

				rt_memset(multiple_buffer, ~sector, geometry.bytes_per_sector * sector);
				rt_kprintf("read : ");
				tick_start = rt_tick_get();
				for (i = 0; i < 10; i++)
				{
					rt_size_t n;
					n = rt_device_read(device, 50, multiple_buffer, sector);
					if (n == sector)
					{
						rt_kprintf(">");
					}
					else
					{
						rt_kprintf("#");
					}
				}
				tick_end = rt_tick_get();
				rt_kprintf("\r\n");
				rt_kprintf("multiple read %d sector speed : ", sector);
				calculate_speed_print((geometry.bytes_per_sector * sector * 10 * RT_TICK_PER_SECOND) / (tick_end - tick_start));

				ptr = multiple_buffer;
				for (i = 0; i < geometry.bytes_per_sector * sector; i++)
				{
					if (*ptr != sector)
					{
						rt_kprintf(" but data verify fail!");
						break;
					}
					ptr++;
				}
				rt_kprintf("\r\n\r\n");

				rt_free(multiple_buffer);
			}
		} /* step 5: multiple sector speed test */

		return RT_EOK;
	}// device can read and write.
	else
	{
		// device read only
		return RT_EOK;
	}// device read only

__return:
	if (read_buffer != RT_NULL)
	{
		rt_free(read_buffer);
	}
	if (write_buffer != RT_NULL)
	{
		rt_free(write_buffer);
	}
	return RT_ERROR;
}

int device_test(const char * device_name)
{
	rt_device_t device = RT_NULL;

	// step 1:find device
	device = rt_device_find(device_name);
	if (device == RT_NULL)
	{
		rt_kprintf("device %s: not found!\r\n");
		return RT_ERROR;
	}

	// step 2:init device
	if (!(device->flag & RT_DEVICE_FLAG_ACTIVATED))
	{
		rt_err_t result;
		result = rt_device_init(device);
		if (result != RT_EOK)
		{
			rt_kprintf("To initialize device:%s failed. The error code is %d\r\n",
				device->parent.name, result);
			return result;
		}
		else
		{
			device->flag |= RT_DEVICE_FLAG_ACTIVATED;
		}
	}

	// step 3: device test
	switch (device->type)
	{
	case RT_Device_Class_Block:
		rt_kprintf("block device!\r\n");
		return _block_device_test(device);
	default:
		rt_kprintf("unkown device type : %02X", device->type);
		return RT_ERROR;
	}
}

#ifdef RT_USING_FINSH
FINSH_FUNCTION_EXPORT(device_test, e.g: device_test("sd0"));
#endif
