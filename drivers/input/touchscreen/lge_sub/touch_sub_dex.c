/* Driver for Touch DEX */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <touch_sub_core.h>

static int touch_set_input_prop(struct touch_sub_core_data *ts, struct input_dev *input)
{
	int ret;
	TOUCH_TRACE();

	input->phys = "devices/virtual/input";
	TOUCH_I("%s %d-%d-%d\n", __func__,
			ts->caps.max_x,
			ts->caps.max_y,
			MAX_FINGER_DEX);

	/* Common Set */
	set_bit(EV_SYN, input->evbit);
	set_bit(EV_ABS, input->evbit);
	set_bit(EV_KEY, input->evbit);
	set_bit(EV_REL, input->evbit);
	set_bit(REL_WHEEL, input->relbit);
	set_bit(REL_HWHEEL, input->relbit);
	set_bit(BTN_RIGHT, input->keybit);
	set_bit(BTN_LEFT, input->keybit);
	set_bit(BTN_TOUCH, input->keybit);
	set_bit(BTN_TOOL_FINGER, input->keybit);
	set_bit(BTN_TOOL_MOUSE, input->keybit);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0,
			ts->caps.max_x, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0,
			ts->caps.max_y, 0, 0);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0,
			ts->caps.max_pressure, 0, 0);
	set_bit(INPUT_PROP_POINTER, input->propbit);
	ret = input_mt_init_slots(input, MAX_FINGER_DEX, INPUT_PROP_POINTER);
	if (ret < 0) {
		TOUCH_E("failed to init slots (ret:%d)\n", ret);
		return -EAGAIN;
	}
	input_set_drvdata(input, ts);

	ts->touch_dex.scroll.scroll_width = ts->caps.max_y / 30;
	return 0;
}

int dex_sub_input_init(struct device *dev)
{
	struct touch_sub_core_data *ts = to_touch_sub_core(dev);
	int ret;

	TOUCH_TRACE();

	ts->input_dex = input_allocate_device();
	if (!ts->input_dex) {
		TOUCH_E("failed to allocate memory for input dex\n");
		return -ENOMEM;
	}

	ts->input_dex->name = "touch_sub_dex_dev";
	ret = touch_set_input_prop(ts, ts->input_dex);
	if (ret < 0) {
		goto error_register;
	}
	ret = input_register_device(ts->input_dex);
	if (ret < 0) {
		TOUCH_E("failed to register input dex(ret:%d)\n", ret);
		goto error_register;
	}
	return 0;

error_register:
	input_mt_destroy_slots(ts->input_dex);
	input_free_device(ts->input_dex);

	if (ts->input_dex) {
		input_mt_destroy_slots(ts->input_dex);
		input_free_device(ts->input_dex);
	}
	return ret;
}

static void slot_control_func(struct device *dev, struct input_dev *input, int i)
{
	struct input_mt *mt = input->mt;
	struct input_mt_slot *i_slot;
	struct touch_sub_core_data *ts = to_touch_sub_core(dev);
	i_slot = &mt->slots[i];
	/*
	TOUCH_I("is_active:%d is_used:%d\n", input_mt_is_active(i_slot),
			input_mt_is_used(mt, i_slot));
	*/
	if (input_mt_is_active(i_slot)) {
		input_mt_slot(input, i);
		input_mt_report_slot_state(input, MT_TOOL_FINGER, false);
		if (ts->tdata[i].dex_data.status != DEX_RELEASED) {
			input_report_key(input, BTN_TOOL_FINGER, 0);
			input_sync(input);
		}
	}
}

static void specific_release_event(struct device *dev, struct input_dev *input, int i)
{
	struct touch_sub_core_data *ts = to_touch_sub_core(dev);
	struct input_mt *mt = input->mt;
	struct input_mt_slot *i_slot;
	i_slot = &mt->slots[i];
	TOUCH_TRACE();
	if (input_mt_is_active(i_slot)) {
		input_mt_slot(input, i);
		input_mt_report_slot_state(input, MT_TOOL_FINGER,
				true);
		input_report_key(input, BTN_TOUCH, 1);
		input_report_key(input, BTN_TOOL_FINGER, 1);
		input_report_abs(input, ABS_MT_PRESSURE,
				255);
		TOUCH_I("finger canceled dex:<%d>(%4d,%4d,%4d)\n",
				i,
				ts->tdata[i].x,
				ts->tdata[i].y,
				ts->tdata[i].pressure);
		input_sync(input);
	}
}

int chk_time_interval(u64 t_aft, u64 t_bef, int t_val)
{
	u64 interval = t_val * 1000000;
	if ((chk_interval(t_aft, t_bef)) <= interval)
		return 0;
	return 1;
}

int move_check(struct device *dev, int prev, int curr)
{
	struct touch_sub_core_data *ts = to_touch_sub_core(dev);
	if ((chk_interval(prev, curr)) < ts->touch_dex.scroll.scroll_width)
		return 0;
	return 1;
}

static int dex_wheel_acc(struct device *dev, struct input_dev *input, int area, int rel,  int x, int y)
{
	struct touch_sub_core_data *ts = to_touch_sub_core(dev);
	int ret = 0;
	int direction = -1;
	u64 curr_time = 0;
	/*
	TOUCH_I("prevy:%d y:%d prevx:%d x:%d point:%d\n", ts->touch_dex.scroll.prev_y,
			y,
			ts->touch_dex.scroll.prev_x,
			x,
			ts->touch_dex.scroll.scroll_point);
			*/
	if (area == VERTICAL) {
		if (ts->touch_dex.scroll.scrolling == 0) {
			TOUCH_I("enter V\n");
			ts->touch_dex.scroll.prev_y = y;
			ts->touch_dex.scroll.scroll_point = y;
			ts->touch_dex.scroll.prev_direction = direction;
			ts->touch_dex.scroll.scrolling = 1;
			ts->touch_dex.scroll.accu_count = 0;
		}
		if (ts->touch_dex.scroll.prev_y > y) {
			direction = POSITIVE;
		} else if (ts->touch_dex.scroll.prev_y < y) {
			direction = NEGATIVE;
		} else {
		}
		ts->touch_dex.scroll.prev_y = y;
	} else if (area == HORIZONTAL) {
		if (ts->touch_dex.scroll.scrolling == 0) {
			TOUCH_I("enter H\n");
			ts->touch_dex.scroll.prev_x = x;
			ts->touch_dex.scroll.scroll_point = x;
			ts->touch_dex.scroll.prev_direction = direction;
			ts->touch_dex.scroll.scrolling = 1;
			ts->touch_dex.scroll.accu_count = 0;
		}
		if (ts->touch_dex.scroll.prev_x > x) {
			direction = NEGATIVE;
		} else if (ts->touch_dex.scroll.prev_x < x) {
			direction = POSITIVE;
		} else {
		}
		ts->touch_dex.scroll.prev_x = x;
	} else {
	}

	if (ts->touch_dex.scroll.prev_direction != direction) {
		ts->touch_dex.scroll.prev_direction = direction;
		ts->touch_dex.scroll.start_time = ktime_get_ns();
	}
	curr_time = ktime_get_ns();
	if (chk_time_interval(ts->touch_dex.scroll.start_time, curr_time, 10)) {
		if (ts->touch_dex.scroll.accu_count < ACCU_MAX_COUNTS) {
			ts->touch_dex.scroll.accu_count++;
		} else {
			if (area == VERTICAL) {
				if (move_check(dev, ts->touch_dex.scroll.scroll_point, y)) {
					if (direction == NEGATIVE) {
						input_report_rel(input, rel, -1);
					} else if (direction == POSITIVE) {
						input_report_rel(input, rel, 1);
					}
					ts->touch_dex.scroll.scroll_point = y;
				}
			} else if (area == HORIZONTAL) {
				if (move_check(dev, ts->touch_dex.scroll.scroll_point, x)) {
					if (direction == NEGATIVE) {
						input_report_rel(input, rel, -1);
					} else if (direction == POSITIVE) {
						input_report_rel(input, rel, 1);
					}
					ts->touch_dex.scroll.scroll_point = x;
				}
			}
			input_sync(input);
			ts->touch_dex.scroll.accu_count = 0;
		}
	}

	return ret;
}

/* Dex Wheel Scrolling filter */
static void dex_wheel_filter(struct device *dev, struct input_dev *input, int id)
{
	struct touch_sub_core_data *ts = to_touch_sub_core(dev);
	int x = 0;
	int y = 0;
	TOUCH_TRACE();

	if (!ts->touch_dex.enable_pad)
	    return;

	if (ts->touch_dex.l_btn_status == BTN_DOWN ||
			ts->touch_dex.r_btn_status == BTN_DOWN)
		return;

	x = ts->tdata[id].x;
	y = ts->tdata[id].y;
	/* wheel area Vertical */
	if (id == 0 && ts->dex_tcount == 1) {
		if (((x >= ts->touch_dex.wheel_area.x1) && (x <= ts->touch_dex.wheel_area.x2)
					&& (y >= ts->touch_dex.wheel_area.y1) && (y <= ts->touch_dex.wheel_area.y2)) ||
				((x >= ts->touch_dex.wheel_area.x3) && (x <= ts->touch_dex.wheel_area.x4)
				 && (y >= ts->touch_dex.wheel_area.y3) && (y <= ts->touch_dex.wheel_area.y4))) {
			TOUCH_D(DEX, "vertical wheel!!\n");
			specific_release_event(dev, input, id);
			slot_control_func(dev, input, id);
			input_report_key(input, BTN_TOOL_MOUSE, 1);
			input_sync(input);
			if (ts->touch_dex.rotate == 0 || ts->touch_dex.rotate == 2) {
				dex_wheel_acc(dev, input, VERTICAL, REL_WHEEL, x, y);
			} else {
				dex_wheel_acc(dev, input, HORIZONTAL, REL_WHEEL, x, y);
			}
		} else
		/* wheel area Horizontal */
		if ((x >= ts->touch_dex.wheel_area1.x1) && (x <= ts->touch_dex.wheel_area1.x2)
				&& (y >= ts->touch_dex.wheel_area1.y1) && (y <= ts->touch_dex.wheel_area1.y2)) {
			TOUCH_D(DEX, "Horizontal wheel!!\n");
			specific_release_event(dev, input, id);
			slot_control_func(dev, input, id);
			input_report_key(input, BTN_TOOL_MOUSE, 1);
			input_sync(input);
			if (ts->touch_dex.rotate == 0 || ts->touch_dex.rotate == 2) {
				dex_wheel_acc(dev, input, HORIZONTAL, REL_HWHEEL, x, y);
			} else {
				dex_wheel_acc(dev, input, VERTICAL, REL_HWHEEL, x, y);
			}
		} else {
			input_report_key(input, BTN_TOOL_MOUSE, 0);
			if (ts->touch_dex.scroll.scrolling)
				input_sync(input);
			input_report_key(input, BTN_TOOL_FINGER, 1);
			ts->touch_dex.scroll.scrolling = 0;
			TOUCH_D(DEX, "scrolling = 0\n");
		}
	} else {
		TOUCH_D(DEX, "not support 2 counts in scroll area %d\n", ts->dex_tcount);
	}
}

/* Dex Mouse Btn filter */
static void dex_mouse_btn_filter(struct device *dev, struct input_dev *input, int i, int btn_status)
{
	struct touch_sub_core_data *ts = to_touch_sub_core(dev);
	int x = 0;
	int y = 0;
	int area = OUT_OF_AREA;

	TOUCH_TRACE();

	if (!ts->touch_dex.enable_pad)
	    return;

	if (ts->touch_dex.scroll.scrolling)
		return;

	x = ts->tdata[i].x;
	y = ts->tdata[i].y;

	/* check area */
	if ((x >= ts->touch_dex.btn_area.x1) && (x <= ts->touch_dex.btn_area.x2 / 2)
			&& (y >= ts->touch_dex.btn_area.y1) && (y <= ts->touch_dex.btn_area.y2)) {
		area =  L_AREA;
		TOUCH_D(DEX, "area = L_AREA\n");
	} else if ((x > ts->touch_dex.btn_area.x2 / 2) && (x <= ts->touch_dex.btn_area.x2)
			&& (y >= ts->touch_dex.btn_area.y1) && (y <= ts->touch_dex.btn_area.y2)) {
		area =  R_AREA;
		TOUCH_D(DEX, "area = R_AREA\n");
	} else {
		TOUCH_D(DEX, "out of btn area\n");
	}

	if (btn_status == BTN_DOWN) {
		if (area == L_AREA) {
			if (ts->touch_dex.r_btn_status == BTN_DOWN) {
				TOUCH_D(DEX, "r_btn_status: BTN_DOWN, area = L_AREA, R_UP\n");
				goto R_UP;
			}
			if (ts->touch_dex.l_btn_status == BTN_UP) {
				TOUCH_D(DEX, "l_btn_status: BTN_UP, area = L_AREA, L_DOWN\n");
				goto L_DOWN;
			}
		}
		if (area == R_AREA) {
			if (ts->touch_dex.l_btn_status == BTN_DOWN) {
				TOUCH_D(DEX, "l_btn_status: BTN_DOWN, area = R_AREA, L_UP\n");
				goto L_UP;
			}
			if (ts->touch_dex.r_btn_status == BTN_UP) {
				TOUCH_D(DEX, "r_btn_status: BTN_UP, area = R_AREA, R_DOWN\n");
				goto R_DOWN;
			}
		}
		if (area == OUT_OF_AREA) {
			if (ts->touch_dex.l_btn_status == BTN_DOWN) {
				TOUCH_D(DEX, "l_btn_status: BTN_DOWN, area = OUT_OF_AREA, L_UP\n");
				goto L_UP;
			}
			if (ts->touch_dex.r_btn_status == BTN_DOWN) {
				TOUCH_D(DEX, "r_btn_status: BTN_DOWN, area = OUT_OF_AREA, R_UP\n");
				goto R_UP;
			}
		}
	} else if (btn_status == BTN_UP) {
		if (area == L_AREA) {
			if (ts->touch_dex.l_btn_status == BTN_DOWN) {
				TOUCH_D(DEX, "l_btn_status: BTN_DOWN, area = L_AREA, L_UP\n");
				goto L_UP;
			}
		}
		if (area == R_AREA) {
			if (ts->touch_dex.r_btn_status == BTN_DOWN) {
				TOUCH_D(DEX, "r_btn_status: BTN_DOWN, area = R_AREA, R_UP\n");
				goto R_UP;
			}
		}
	}

	return;

L_DOWN:
	if (ts->tdata[i].dex_data.btn_status != BTN_DOWN_L) {
		input_report_key(input, BTN_LEFT, 1);
		ts->tdata[i].dex_data.btn_status = BTN_DOWN_L;
		ts->touch_dex.l_btn_status = BTN_DOWN;
		TOUCH_I("L BTN d\n");
		input_sync(input);
	}
	return;

R_DOWN:
	if (ts->tdata[i].dex_data.btn_status != BTN_DOWN_R) {
		input_report_key(input, BTN_RIGHT, 1);
		ts->tdata[i].dex_data.btn_status = BTN_DOWN_R;
		ts->touch_dex.r_btn_status = BTN_DOWN;
		TOUCH_I("R BTN d\n");
		input_sync(input);
	}
	return;

L_UP:
	if (ts->tdata[i].dex_data.btn_status == BTN_DOWN_L) {
		input_report_key(input, BTN_LEFT, 0);
		ts->tdata[i].dex_data.btn_status = BTN_UP;
		ts->touch_dex.l_btn_status = BTN_UP;
		TOUCH_I("L BTN u\n");
		input_sync(input);
	}
	return;

R_UP:
	if (ts->tdata[i].dex_data.btn_status == BTN_DOWN_R) {
		input_report_key(input, BTN_RIGHT, 0);
		ts->tdata[i].dex_data.btn_status = BTN_UP;
		ts->touch_dex.r_btn_status = BTN_UP;
		TOUCH_I("R BTN u\n");
		input_sync(input);
	}
	return;
}
static int dex_area_filter(struct device *dev, int id)
{
	struct touch_sub_core_data *ts = to_touch_sub_core(dev);
	int x = 0;
	int y = 0;

	TOUCH_TRACE();

	x = ts->tdata[id].x;
	y = ts->tdata[id].y;

	if ((x >= ts->touch_dex.area.x1) && (x <= ts->touch_dex.area.x2)
		&& (y >= ts->touch_dex.area.y1) && (y <= ts->touch_dex.area.y2)) {
		TOUCH_D(DEX, "dex_in\n");
		return DEX_IN;
	} else {
		TOUCH_D(DEX, "dex_out\n");
		return DEX_OUT;
	}
}

static int dex_button_area_filter(struct device *dev, int id)
{
	struct touch_sub_core_data *ts = to_touch_sub_core(dev);
	int x = 0;
	int y = 0;

	TOUCH_TRACE();

	x = ts->tdata[id].x;
	y = ts->tdata[id].y;

	if ((x >= ts->touch_dex.button_area.x1) && (x <= ts->touch_dex.button_area.x2)
		&& (y >= ts->touch_dex.button_area.y1) && (y <= ts->touch_dex.button_area.y2)) {
		TOUCH_D(DEX, "button_in\n");
		return BUTTON_IN;
	} else {
		TOUCH_D(DEX, "button_out\n");
		return BUTTON_OUT;
	}
}

void touch_sub_cancel_event(struct touch_sub_core_data *ts, struct input_dev *input)
{
	u16 old_mask = ts->old_mask;
	int i = 0;

	TOUCH_TRACE();

	for (i = 0; i < ts->dex_tcount; i++) {
		if (old_mask & (1 << i)) {
			input_mt_slot(input, i);
			input_mt_report_slot_state(input, MT_TOOL_FINGER,
						   true);
			input_report_key(input, BTN_TOUCH, 1);
			input_report_key(input, BTN_TOOL_FINGER, 1);
			input_report_key(input, BTN_TOOL_MOUSE, 0);
			input_report_abs(input, ABS_MT_PRESSURE,
					255);
			TOUCH_I("finger canceled dex:<%d>(%4d,%4d,%4d)\n",
					i,
					ts->tdata[i].x,
					ts->tdata[i].y,
					ts->tdata[i].pressure);
		}
	}
	input_sync(input);
}

/* Input Dex in Touch core
 * This function is not support changing ts->mask
 * */
void dex_sub_input_handler(struct device *dev, struct input_dev *input)
{
	struct touch_sub_core_data *ts = to_touch_sub_core(dev);
	u16 old_mask = ts->old_mask;
	u16 new_mask = ts->new_mask;
	u16 press_mask = 0;
	u16 release_mask = 0;
	u16 change_mask = 0;
	int i = 0;
	bool hide_lockscreen_coord =
		((atomic_read(&ts->state.lockscreen) == LOCKSCREEN_LOCK) &&
		 (ts->role.hide_coordinate));

	TOUCH_TRACE();

	change_mask = old_mask ^ new_mask;
	press_mask = new_mask & change_mask;
	release_mask = old_mask & change_mask;

	TOUCH_D(ABS, "mask [new: %04x, old: %04x]\n",
			new_mask, old_mask);
	TOUCH_D(ABS, "mask [change: %04x, press: %04x, release: %04x]\n",
			change_mask, press_mask, release_mask);

	/* Palm state - Report Pressure value 255 */
	if (ts->is_cancel) {
		TOUCH_D(DEX, "dex is canceled\n");
		touch_sub_cancel_event(ts, input);
	}

	for (i = 0; i < MAX_FINGER_DEX; i++) {
		if (new_mask & (1 << i)) {
			TOUCH_D(DEX, "dex new_mask <%d>\n", i);
			ts->tdata[i].dex_data.pos = dex_area_filter(dev, i);
			ts->tdata[i].dex_data.button_pos = dex_button_area_filter(dev, i);
			if (press_mask & (1 << i)) {
				TOUCH_D(DEX, "dex press mask\n");
				if (ts->tdata[i].dex_data.pos == DEX_OUT) {
					ts->tdata[i].dex_data.status = DEX_NOT_SUPPORT;
					TOUCH_D(DEX, "dex_out -> dex_not_support<%d>\n", i);
				} else if (ts->tdata[i].dex_data.pos == DEX_IN && ts->tdata[i].dex_data.button_pos == BUTTON_IN) {
					ts->tdata[i].dex_data.status = DEX_NOT_SUPPORT;
					TOUCH_D(DEX, "button_in -> dex_not_support<%d>\n", i);
				} else {
					ts->tdata[i].dex_data.status = DEX_PRESSED;
					ts->dex_tcount++;
					if (hide_lockscreen_coord) {
						TOUCH_I("%d finger pressed dex:<%d>(xxxx,xxxx,xxxx)\n",
								ts->tcount, i);
					} else {
						TOUCH_I("%d finger pressed dex:<%d>(%4d,%4d)\n",
								ts->tcount,
								i,
								ts->tdata[i].x,
								ts->tdata[i].y);
					}
				}
			}

			if (ts->tdata[i].dex_data.status == DEX_PRESSED ||
					ts->tdata[i].dex_data.status == DEX_MOVED) {
					TOUCH_D(DEX, "dex_pressed or dex_moved\n");
				/* Force Released */
				if (ts->tdata[i].dex_data.pos == DEX_OUT) {
					TOUCH_D(DEX, "dex_data is outside\n");
					input_mt_slot(input, i);
					input_mt_report_slot_state(input, MT_TOOL_FINGER, false);
					ts->tdata[i].dex_data.status = DEX_RELEASED;
					ts->touch_dex.scroll.scrolling = 0;
					if (ts->dex_tcount > 0) {
						ts->dex_tcount--;
						TOUCH_D(DEX, "dex_out -> dex_tcount<%d>\n", ts->dex_tcount);
					}

					if (hide_lockscreen_coord) {
						TOUCH_I(" finger released dex:<%d>(xxxx,xxxx,xxxx)\n",
								i);
					} else {
						TOUCH_I(" finger released dex:<%d>(%4d,%4d)\n",
								i,
								ts->tdata[i].x,
								ts->tdata[i].y);
					}
				} else {
					TOUCH_D(DEX, "dex_data is inside\n");
					/* Wheel filter */
					input_report_key(input, BTN_TOUCH, 1);
					dex_wheel_filter(dev, input, i);
					if (ts->touch_dex.scroll.scrolling == 0) {
						input_mt_slot(input, i);
						input_mt_report_slot_state(input, MT_TOOL_FINGER,
								true);
						input_report_abs(input, ABS_MT_POSITION_X,
								ts->tdata[i].x);
						input_report_abs(input, ABS_MT_POSITION_Y,
								ts->tdata[i].y);
						input_report_abs(input, ABS_MT_PRESSURE,
								ts->tdata[i].pressure);
					} else {
					}
					if (ts->tdata[i].dex_data.status == DEX_PRESSED) {
						if (ts->touch_dex.scroll.scrolling == 0) {
							TOUCH_D(DEX, "dex_data.status == DEX_PRESSED and scrolling == 0\n");
							input_sync(input);
						}
						ts->tdata[i].dex_data.status = DEX_MOVED;
						TOUCH_D(DEX, "dex_pressed -> dex_moved<%d>\n", i);
					}
				}
				/* Mouse Button filter */
				dex_mouse_btn_filter(dev, input, i, BTN_DOWN);
			} else {
				TOUCH_D(ABS,"dex moving Ignore Event:<%d>(%4d,%4d)\n",
						i,
						ts->tdata[i].x,
						ts->tdata[i].y);
			}

		} else if (release_mask & (1 << i)) {
			TOUCH_D(DEX, "dex release_mask <%d>\n", i);
			ts->tdata[i].dex_data.pos = dex_area_filter(dev, i);

			/* Mouse Button filter */
			dex_mouse_btn_filter(dev, input, i, BTN_UP);
			if (ts->tdata[i].dex_data.pos == DEX_IN
					&& ts->tdata[i].dex_data.status == DEX_MOVED) {
				TOUCH_D(DEX, "dex_in and dex_moved\n");
				/* Wheel filter */
				ts->tdata[i].dex_data.status = DEX_RELEASED;
				dex_wheel_filter(dev, input, i);
				slot_control_func(dev, input, i);
				if (ts->dex_tcount > 0) {
					ts->dex_tcount--;
					TOUCH_D(DEX, "dex_in_moved -> dex_tcount<%d>\n", ts->dex_tcount);
				}
				if (hide_lockscreen_coord) {
					TOUCH_I(" finger released dex:<%d>(xxxx,xxxx,xxxx)\n",
							i);
				} else {
					TOUCH_I(" finger released dex:<%d>(%4d,%4d)\n",
							i,
							ts->tdata[i].x,
							ts->tdata[i].y);
				}

			} else {
				TOUCH_D(ABS,"dex Ignore Event:<%d>(%4d,%4d)\n",
						i,
						ts->tdata[i].x,
						ts->tdata[i].y);
			}
		}
	}

	if (!ts->dex_tcount) {
		input_report_key(input, BTN_TOOL_FINGER, 0);
		input_report_key(input, BTN_TOOL_MOUSE, 0);
		input_report_key(input, BTN_TOUCH, 0);
		ts->touch_dex.scroll.scrolling = 0;
	}

	if (ts->tdata[i].dex_data.status != DEX_PRESSED) {
		TOUCH_D(DEX, "dex_data.status != DEX_PRESSED\n");
		if (ts->touch_dex.scroll.scrolling == 0) {
			TOUCH_D(DEX, "dex_data.status != DEX_PRESSED and scrolling == 0\n");
			input_sync(input);
		}
	}
}

MODULE_AUTHOR("rangkast.jeong@lge.com");
MODULE_DESCRIPTION("LGE DEX driver v1");
MODULE_LICENSE("GPL");
