#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/of_gpio.h>
#include <linux/hrtimer.h>
#define MAX_FINGER_DEX		3
#define ACCU_MAX_COUNTS 1
#define chk_interval(a, b)	(a >= b ? a - b : b - a)

struct dex_active_area {
	u16 x1;
	u16 y1;
	u16 x2;
	u16 y2;
	u16 x3;
	u16 y3;
	u16 x4;
	u16 y4;
};

struct scroll_ctrl {
	u32 prev_x;
	u32 prev_y;
	u8 scrolling;
	u8 prev_direction;
	int accu_count;
	u64 start_time;
	int scroll_width;
	int scroll_point;
};

enum {
	DEX_IN = 1,
	DEX_OUT,
	BUTTON_IN,
	BUTTON_OUT,
};

enum {
	BTN_UP,
	BTN_DOWN,
	BTN_DOWN_L,
	BTN_DOWN_R,
};

enum {
	OUT_OF_AREA,
	L_AREA,
	R_AREA,
};
enum {
	DEX_PRESSED = 1,
	DEX_RELEASED,
	DEX_MOVED,
	DEX_NOT_SUPPORT,
};
enum {
	POSITIVE,
	NEGATIVE,
};
enum {
	VERTICAL,
	HORIZONTAL,
};
struct touch_dex_ctrl {
	bool enable;
	bool button_enable;
	bool enable_pad;
	int rotate;
	struct dex_active_area area;
	struct dex_active_area wheel_area;
	struct dex_active_area wheel_area1;
	struct dex_active_area btn_area;
	struct dex_active_area button_area;
	struct scroll_ctrl scroll;
	u8 r_btn_status;
	u8 l_btn_status;
};

struct touch_dex_data {
	u8 pos;
	u8 status;
	u8 button_pos;
	u8 btn_status;
};
extern int dex_sub_input_init(struct device *dev);
extern void dex_sub_input_handler(struct device *dev, struct input_dev *input);
