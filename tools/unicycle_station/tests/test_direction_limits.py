"""Exercise actual image weighting and steering functions with synthetic lines."""
from test_lean_limits import ROOT, _compile_run, _function


def test_direction_large_errors_and_existing_gates(tmp_path):
    image = (ROOT / 'code/image.c').read_text(encoding='utf-8')
    control = (ROOT / 'code/control.c').read_text(encoding='utf-8')
    source = r'''
#define MT9V03X_W 188
#define MT9V03X_H 120
#include "board_config.h"
#include <assert.h>
#include <math.h>
#define TRACK_MODE_MIDDLE 0
#define TRACK_MODE_LEFT 1
#define TRACK_MODE_RIGHT 2
#define TRACK_MODE_HOLD 3
#define STEER_DIR (-1)
static float pixel_kp=2;
#define DIRECTION_PIXEL_KP pixel_kp
#define DIRECTION_HEADING_KP DIRECTION_HEADING_KP_DEFAULT
#define DIRECTION_CURVE_KFF DIRECTION_CURVE_KFF_DEFAULT
#define DIRECTION_BALANCE_KD DIRECTION_BALANCE_KD_DEFAULT
static struct {int track_mode;} g_elem_action;
static struct {
 int Left_Line[IMG_H], Right_Line[IMG_H], Mid_Line[IMG_H];
 uint8 Left_Lost_Flag[IMG_H], Right_Lost_Flag[IMG_H], Mid_Lost_Flag[IMG_H];
 int Track_Valid;
} my_image;
static int Road_Half_Wide[IMG_H];
static float g_direction_camera, g_mid_error;
static uint8 g_direction_valid;
static float s_direction_last_error, g_vision_heading_error, g_vision_curvature;
static float g_yaw_momentum_scale=1, s_direction_offset, s_direction_yaw_rate_target;
static float s_direction_yaw_rate_cmd, s_direction_hold_yaw, g_yaw_target;
static float g_vision_direction_camera;
static uint8 s_run_active=1, s_direction_last_mode, g_vision_track_mode, g_track_valid=1;
static float constrain_float(float v,float lo,float hi){return v<lo?lo:(v>hi?hi:v);}
static float image_fclip(float v,float lo,float hi){return constrain_float(v,lo,hi);}
static int ctrl_is_finite(float v){return isfinite(v);}
static float Y_Motor_GetSpeedMps(void){return .5f;}
static float imu_get_angle_yaw(void){return 17;}
'''
    for text, name in [(image, 'image_update_direction_camera'),
                       (control, 'direction_balance_Control'),
                       (control, 'direction_control_update'),
                       (control, 'direction_yaw_target_update')]:
        source += _function(text, name) + '\n'
    source += r'''
static void near(float a,float b){assert(fabsf(a-b)<.002f);}
static void lines(int offset,int mode){
 memset(&my_image,0,sizeof(my_image));
 g_elem_action.track_mode=mode;
 for(int i=0;i<IMG_H;i++){
   Road_Half_Wide[i]=20;
   my_image.Mid_Line[i]=IMG_MID_COL+offset;
   my_image.Left_Line[i]=IMG_MID_COL-20+offset;
   my_image.Right_Line[i]=IMG_MID_COL+20+offset;
 }
}
static void steer(void){
 g_vision_direction_camera=g_direction_camera;
 g_track_valid=g_direction_valid;
 g_vision_track_mode=g_elem_action.track_mode;
 for(int i=0;i<100;i++) direction_control_update();
}
int main(void){
 /* Small errors retain their slope; both signs of large errors remain distinct. */
 for(int mode=0;mode<3;mode++) for(int e=-60;e<=60;e+=5){
   lines(e,mode); image_update_direction_camera();
   float expected=constrain_float(e*345.f,-15000.f,15000.f);
   assert(g_direction_valid);near(g_direction_camera,expected);
   steer();near(s_direction_offset,-expected/345.f);
   near(s_direction_yaw_rate_target,-2*expected/345.f);
 }
 /* Partial valid rows are renormalized, not weakened by missing row weights. */
 lines(30,TRACK_MODE_MIDDLE);
 for(int i=0;i<IMG_H/2;i++) my_image.Mid_Lost_Flag[i]=1;
 image_update_direction_camera();near(g_direction_camera,10350);
 /* Existing fallback sign, validity and HOLD behavior. */
 memset(my_image.Mid_Lost_Flag,1,sizeof(my_image.Mid_Lost_Flag));
 my_image.Track_Valid=1;g_mid_error=-30;
 image_update_direction_camera();near(g_direction_camera,10350);
 my_image.Track_Valid=0;image_update_direction_camera();
 assert(!g_direction_valid);steer();near(s_direction_yaw_rate_target,0);
 near(g_yaw_target,17);
 lines(30,TRACK_MODE_HOLD);image_update_direction_camera();
 assert(g_direction_valid);near(g_direction_camera,0);
 /* CPU0 still bounds oversized values and final rate/momentum permissions. */
 g_vision_track_mode=TRACK_MODE_MIDDLE;g_track_valid=1;
 g_vision_direction_camera=1e8f;pixel_kp=100;
 for(int i=0;i<100;i++)direction_control_update();
 near(s_direction_offset,-15000.f/345);near(s_direction_yaw_rate_target,-120);
 g_yaw_momentum_scale=.25f;direction_control_update();
 near(s_direction_yaw_rate_target,-30);
 /* Existing 1 ms slew and heading-lead bounds remain active. */
 s_direction_yaw_rate_cmd=0;g_yaw_target=17;
 direction_yaw_target_update();near(s_direction_yaw_rate_cmd,-.6f);
 for(int i=0;i<5000;i++)direction_yaw_target_update();
 near(s_direction_yaw_rate_cmd,-30);near(g_yaw_target,17-35);
 s_run_active=0;direction_control_update();
 near(s_direction_offset,0);near(s_direction_yaw_rate_target,0);
 near(s_direction_yaw_rate_cmd,0);
 return 0;
}
'''
    _compile_run(tmp_path, source)