/* Host contract test only. Never link or flash this file to the TC264. */
#include "../../../../code/vofa.c"
#include <assert.h>

param_t g_param;
volatile uint32 g_param_revision;
start_state_t start_flag;
attitude_t att;
uint8 g_imu_ok=1, g_cam_ok=1;
volatile uint32 g_control_uptime_ms;
uint8 g_track_valid=1,g_vision_active_elem,g_vision_island_state,g_vision_track_mode;
volatile uint16 g_vision_age_ms;
volatile uint32 g_vision_frame_seq=1;
float g_vision_curvature,g_vision_quality=1;
float Y_Motor_GetSpeedMps(void) { return 0.0f; }
static uint8 mock_test, mock_jog, mock_run, mock_remote;
static int save_count, stop_count;
static int wheel_speed;
const param_desc_t g_param_table[] = {
    {"r_rate_kp", &g_param.r_rate_kp, 1, -2000,2000},
    {"r_angle_kp", &g_param.r_angle_kp, 1,-2000,2000},
    {"motor_dir_a", &g_param.motor_dir_a, 0,-1,1},
    {"cam_exposure", &g_param.cam_exposure, 0,4,1600}
};
uint16 param_count(void) { return 4; }
const param_desc_t *param_find(const char *name) {
    uint16 i; for(i=0;i<4;i++) if(strcmp(name,g_param_table[i].name)==0) return &g_param_table[i]; return NULL;
}
uint8 param_get_by_name(const char *name,float *value) {
    const param_desc_t *d=param_find(name); if(!d)return 0;
    *value=d->is_float?*(float*)d->ptr:(float)*(int*)d->ptr; return 1;
}
void param_sync_zero(void) {}
uint8 param_save(void) {save_count++;return 1;}
uint8 param_save_names(const char *const *names,uint16 count) {(void)names;(void)count;save_count++;return 1;}
uint8 control_test_running(void) {return mock_test;}
motor_jog_t control_jog_running(void) {return (motor_jog_t)mock_jog;}
uint8 control_run_running(void) {return mock_run;}
uint8 control_remote_running(void) {return mock_remote;}
void control_stop(void) {start_flag=START_STOP;mock_test=mock_jog=mock_run=mock_remote=0;stop_count++;}
run_stop_t control_run_stop_reason(void) {return RUN_STOP_NONE;}
control_test_status_t control_test_last_status(void) {return CTRL_TEST_STATUS_OK;}
uint8 control_ipm_pending(void) {return 0;}
uint8 control_remote_command(float a,float b) {(void)a;(void)b;return mock_remote;}
void control_run_diag_snapshot(volatile control_run_diag_t *out) {(void)out;}
imu_calib_state_t imu_calib_state(void) {return IMU_CALIB_OK;}
uint8 imu_link_lost(void) {return 0;}
uint8 W_Motor_LinkLost(void) {return 0;}
int16 W_Motor_GetSpeed1(void) {return (int16)wheel_speed;}
int16 W_Motor_GetSpeed2(void) {return (int16)wheel_speed;}
int16 Y_Motor_GetSpeed20ms(void) {return (int16)wheel_speed;}
uint8 attitude_converged(void) {return 1;}
uint8 attitude_diverged(void) {return 0;}

static char response[4096];
static const char *drain(void) {
    unsigned n=0;while(s_tx_tail!=s_tx_head && n<sizeof(response)-1)response[n++]=(char)s_tx_buf[(s_tx_tail++)&VOFA_TX_MASK];
    response[n]=0;return response;
}
static void feed(const char *text) {
    while(*text) {s_rx_buf[s_rx_head&VOFA_RX_MASK]=(uint8)*text++;s_rx_head++;}
    vofa_cmd_poll();
}
int main(void) {
    vofa_init();start_flag=START_STOP;
    feed("cfg:hello,1\r\n");assert(strstr(drain(),"rsp:1,ok,hello,1,"));
    feed("cfg:set,2,r_rate_kp,9999\r\n");assert(g_param.r_rate_kp==2000);assert(strstr(drain(),"2000,CLAMPED"));
    feed("cfg:set,3,r_rate_kp,nan\n");assert(g_param.r_rate_kp==2000);assert(strstr(drain(),"INVALID_VALUE"));
    feed("cfg:set,4,cam_exposure,4.5\n");assert(strstr(drain(),"INVALID_VALUE"));
    feed("cfg:set,5,motor_dir_a,0\n");assert(strstr(drain(),"OUT_OF_RANGE"));
    mock_jog=1;feed("cfg:set,6,r_rate_kp,3\n");assert(strstr(drain(),"RUNNING_LOCKED"));mock_jog=0;
    mock_test=1;g_tune_axis=TUNE_AXIS_ROLL;g_tune_ring=TUNE_RING_RATE;
    feed("cfg:set,7,r_angle_kp,4\n");assert(strstr(drain(),"RUNNING_LOCKED"));
    feed("cfg:set,8,r_rate_kp,12\n");assert(g_param.r_rate_kp==12);assert(strstr(drain(),"APPLIED"));
    start_flag=START_BALANCE;feed("cfg:set,9,motor_dir_a,-1\n");assert(strstr(drain(),"UNSAFE_PARAM"));
    feed("cfg:save,10,all\n");assert(strstr(drain(),"SAVE_BLOCKED"));
    feed("stop\n");assert(stop_count==1 && start_flag==START_STOP);drain();
    feed("cfg:save,11,all\n");assert(s_save_seq==11);feed("stop\n");assert(!s_save_seq && save_count==0);drain();
    feed("cfg:set,12,r_rate_kp,1");assert(g_param.r_rate_kp==12);
    s_rx_idle_ms=1000;vofa_cmd_poll();feed("cfg:set,13,r_rate_kp,2\n");assert(g_param.r_rate_kp==12);drain();
    feed("cfg:set,14,r_rate_kp,3\n");assert(g_param.r_rate_kp==3);drain();
    feed("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxstop\n");assert(stop_count==2);drain();
    s_tx_head=VOFA_TX_SIZE-2;s_tx_tail=0;assert(!tx_push((const uint8*)"abc",3));assert(s_tx_head==VOFA_TX_SIZE-2);s_tx_tail=s_tx_head;
    feed("cfg:schema,15\n");for(unsigned i=0;i<5;i++){g_control_uptime_ms+=31;cfg_poll();}assert(strstr(drain(),"rsp:15,ok,schema,4"));
    g_control_uptime_ms+=600;cfg_poll();drain();
    feed("cfg:save,16,all\n");s_rx_idle_ms=101;cfg_poll();assert(save_count==1);assert(strstr(drain(),"save,all,VERIFIED"));
    /* Diagnostic state capture continues with ordinary VOFA output disabled. */
    task_reset();s_station_att=1;g_vofa_mode=VOFA_OFF;
    g_control_uptime_ms=1000;g_vision_active_elem=ELEM_NONE;g_vision_age_ms=0;
    g_vision_curvature=0;vofa_snapshot();assert(s_task.state==1 && s_task_head==1);
    g_vision_curvature=.04f;g_control_uptime_ms=1010;task_capture();assert(s_task.state==1);
    g_control_uptime_ms=1160;task_capture();assert(s_task.state==2);
    g_vision_curvature=.025f;g_control_uptime_ms=1200;task_capture();assert(s_task.state==2);
    g_vision_active_elem=ELEM_CROSS;g_control_uptime_ms=1201;task_capture();assert(s_task.state==4);
    task_poll();assert(strstr(drain(),"taskevt:1,1000,0,1,"));
    g_vision_active_elem=ELEM_RING_LEFT;g_vision_island_state=1;g_control_uptime_ms=1400;task_capture();
    g_vision_island_state=2;g_control_uptime_ms++;task_capture();assert(s_task.phase==2);
    g_vision_age_ms=VISION_LINK_TIMEOUT_MS+1;g_control_uptime_ms++;task_capture();assert(s_task.state==10);
    for(unsigned i=0;i<40;i++){g_vision_age_ms=(i&1)?0:VISION_LINK_TIMEOUT_MS+1;g_control_uptime_ms++;task_capture();}
    assert(s_task_dropped>0 && s_task_head-s_task_tail==TASK_EVENT_CAPACITY);
    g_control_uptime_ms+=200;task_poll();assert(strstr(drain(),"task:"));
    assert(save_count==1 && stop_count==2);
    printf("MCU host contract checks passed (not an ADS/TASKING build)\n");return 0;
}
