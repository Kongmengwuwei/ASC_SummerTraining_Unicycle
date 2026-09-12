"""Run real Pitch projection and both control paths against kinematic inputs."""
from pathlib import Path
from test_lean_limits import _compile_run, _function

ROOT = Path(__file__).resolve().parents[3]


def test_pitch_euler_projection_and_real_cascade_paths(tmp_path):
    control = (ROOT / 'code/control.c').read_text(encoding='utf-8')
    source = r'''
#include "board_config.h"
#include "pid.h"
#include "pid.c"
#include <assert.h>
#include <math.h>
static struct { float roll, pitch, pitch_rate; } att;
static struct { float gyro_z; } imu;
static float s_pitch_rate_roll, s_pitch_roll_sin, s_pitch_roll_cos;
static uint8 s_pitch_rate_cache_valid;
static pid_t p_vel_pid,p_angle_pid,p_rate_pid;
static float s_speed_ramp,g_pitch_zero;
static int s_test_speed_c,s_test_ring;
#define TUNE_RING_ANGLE 1
#define TUNE_RING_VEL 2
static int Y_Motor_GetSpeed20ms(void) { return 0; }
static void speed_ramp_update(void) {}
static unsigned trig_calls;
static float counted_sin(float v) { ++trig_calls;return sinf(v); }
static float counted_cos(float v) { ++trig_calls;return cosf(v); }
#define sinf counted_sin
#define cosf counted_cos
'''
    for name in ('pitch_euler_rate', 'pitch_cascade_ctrl', 'pitch_test_ctrl'):
        source += _function(control, name) + '\n'
    source += r'''
#undef sinf
#undef cosf
static void close_to(float actual,float expected) {
    if (fabsf(actual-expected)>0.0002f) fprintf(stderr,"actual=%g expected=%g\n",actual,expected);
    assert(fabsf(actual-expected)<0.0002f);
}
/* Forward ZYX kinematics produces body q/r from independent Euler rates.
   A steady banked turn must yield zero Pitch rate even with nonzero body q. */
static void motion(float roll,float pitch,float pitch_dot,float yaw_dot) {
    float phi=roll*0.017453292519943295f;
    float theta=pitch*0.017453292519943295f;
    att.roll=roll;att.pitch=pitch;
    att.pitch_rate=pitch_dot*cosf(phi)+yaw_dot*sinf(phi)*cosf(theta);
    imu.gyro_z=-pitch_dot*sinf(phi)+yaw_dot*cosf(phi)*cosf(theta);
}
static void controller_reset(void) {
    pid_set(&p_vel_pid,0,0,0,200);
    pid_set(&p_angle_pid,8,0,0,50);
    pid_set(&p_rate_pid,25,.75f,0,100);
    s_pitch_rate_cache_valid=0;
    s_speed_ramp=0;g_pitch_zero=0;
}
int main(void) {
    const float banks[]={-20,-8,-.6f,0,.6f,8,20};
    const float pitches[]={-30,0,30};
    const float yaw_rates[]={-120,0,120};
    const float pitch_rates[]={-25,0,25};
    for(unsigned a=0;a<7;a++) for(unsigned b=0;b<3;b++)
    for(unsigned c=0;c<3;c++) for(unsigned d=0;d<3;d++) {
        motion(banks[a],pitches[b],pitch_rates[d],yaw_rates[c]);
        float old_q=att.pitch_rate,old_r=imu.gyro_z;
        close_to(pitch_euler_rate(),pitch_rates[d]);
        assert(att.pitch_rate==old_q && imu.gyro_z==old_r);
    }
    /* A fresh start uses the current bank on its first 1 ms tick. */
    controller_reset();motion(8,0,0,80);trig_calls=0;
    close_to(pitch_euler_rate(),0);assert(trig_calls==2);
    motion(8,0,12,-60);close_to(pitch_euler_rate(),12);
    assert(trig_calls==2); /* latest gyros, cached same attitude */
    motion(-8,0,-12,60);close_to(pitch_euler_rate(),-12);assert(trig_calls==4);
    s_pitch_rate_cache_valid=0;close_to(pitch_euler_rate(),-12);assert(trig_calls==6);
    /* Upright feedback is identical to the previous body-q controller. */
    controller_reset();motion(0,0,3,80);
    close_to(pitch_cascade_ctrl(0,0,0),-77.25f);
    controller_reset();s_test_ring=0;motion(0,0,3,80);
    close_to(pitch_test_ctrl(0,0),-77.25f);
    /* Constant bank + yaw no longer drives the Pitch incremental accumulator. */
    for(int sign=-1;sign<=1;sign+=2) {
        controller_reset();motion(sign*8,0,0,sign*80);
        assert(fabsf(att.pitch_rate)>10);
        for(int n=0;n<1000;n++) close_to(pitch_cascade_ctrl(0,n%5==0,n%20==0),0);
        for(int ring=0;ring<=2;ring++) {
            controller_reset();s_test_ring=ring;motion(sign*8,0,0,sign*80);
            for(int n=0;n<1000;n++) close_to(pitch_test_ctrl(n%5==0,n%20==0),0);
        }
    }
    /* Outer angle control responds to a real pitch change at a bank. */
    controller_reset();motion(8,-1,2,60);
    close_to(pitch_cascade_ctrl(0,1,0),(8-2)*25.75f);
    controller_reset();s_test_ring=TUNE_RING_ANGLE;motion(8,-1,2,60);
    close_to(pitch_test_ctrl(1,0),(8-2)*25.75f);
    /* Existing normal/test output limits remain effective. */
    controller_reset();motion(8,0,-1000,60);
    close_to(pitch_cascade_ctrl(0,0,0),DRIVE_OUT_LIMIT);
    controller_reset();s_test_ring=0;motion(8,0,-1000,60);
    close_to(pitch_test_ctrl(0,0),BAL_TEST_DRIVE_LIMIT);
    return 0;
}
'''
    _compile_run(tmp_path, source)


def test_projection_reset_and_original_sensor_semantics():
    control = (ROOT / 'code/control.c').read_text(encoding='utf-8')
    attitude = (ROOT / 'code/attitude.c').read_text(encoding='utf-8')
    assert 's_pitch_rate_cache_valid = 0;' in _function(control, 'cascade_reset')
    assert 's_pitch_rate_cache_valid = 0;' in _function(control, 'control_init')
    assert 'cascade_reset();' in _function(control, 'control_stop')
    # Existing invalid-input gates precede either controller path.
    assert '!ctrl_is_finite(att.roll)' in _function(control, 'cascade_run')
    assert '!ctrl_is_finite(att.pitch_rate)' in _function(control, 'cascade_run')
    assert '!ctrl_is_finite(imu.gyro_z)' in _function(control, 'cascade_run')
    assert 'att.pitch_rate = imu.gyro_y;' in _function(attitude, 'attitude_update_rate')
    assert 'feedback.pitch_rate = att.pitch_rate;' in control