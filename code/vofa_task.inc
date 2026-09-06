/* UART2 task observation. Included by vofa.c; never a separate translation unit.
 * CPU0 ISR: fixed work, no format/Flash/allocation. CPU0 foreground: TX only.
 * These diagnostic classifications do not feed any motor/control decision. */
#include "element.h"

#define TASK_EVENT_CAPACITY 16u
#define TASK_CURVE_ENTER 0.035f
#define TASK_CURVE_EXIT  0.020f
#define TASK_CURVE_DEBOUNCE_MS 150u

typedef struct {
    uint32 uptime, transition, dropped, vision_seq;
    uint16 age;
    uint8 state, element, phase, track_mode, running;
    float quality, yaw, speed;
} station_task_snapshot_t;
typedef struct {
    uint32 sequence, uptime;
    uint8 previous, current, element, phase, running;
} station_task_event_t;

static volatile station_task_snapshot_t s_task;
static station_task_event_t s_task_events[TASK_EVENT_CAPACITY];
static volatile uint32 s_task_head, s_task_tail;
static uint32 s_task_transition, s_task_dropped, s_task_candidate_since;
static uint8 s_task_state, s_task_candidate, s_task_phase, s_task_running;
static uint32 s_task_last_send, s_task_last_event_send;

static void task_reset(void)
{
    s_task_head = s_task_tail = 0;
    s_task_transition = s_task_dropped = s_task_candidate_since = 0;
    s_task_state = s_task_candidate = s_task_phase = s_task_running = 0;
    s_task_last_send = s_task_last_event_send = 0;
    memset((void *)&s_task, 0, sizeof(s_task));
}

static uint8 task_classify(void)
{
    float curvature;
    if (!g_vision_frame_seq) return 12u; /* waiting for vision */
    if (!g_cam_ok || g_vision_age_ms > VISION_LINK_TIMEOUT_MS) return 10u;
    switch (g_vision_active_elem) {
        case ELEM_CROSS: return 4u;
        case ELEM_RING_LEFT: return 5u;
        case ELEM_RING_RIGHT: return 6u;
        case ELEM_RAMP: return 7u;
        case ELEM_ZEBRA: return 8u;
        default: break;
    }
    if (!g_track_valid) return 9u;
    if (g_vision_track_mode == TRACK_MODE_HOLD) return 11u;
    curvature = g_vision_curvature;
    if (curvature >= TASK_CURVE_ENTER || (s_task_state == 2u && curvature > TASK_CURVE_EXIT)) return 2u;
    if (curvature <= -TASK_CURVE_ENTER || (s_task_state == 3u && curvature < -TASK_CURVE_EXIT)) return 3u;
    return 1u;
}

static void task_capture(void)
{
    uint8 state, running, phase;
    uint32 now, head;
    station_task_event_t *event;
    if (!s_station_att) return;
    now = g_control_uptime_ms;
    state = task_classify();
    if (state != s_task_candidate) { s_task_candidate = state; s_task_candidate_since = now; }
    /* Ordinary straight/curve changes require a persistent observation. */
    if (state >= 1u && state <= 3u && s_task_state >= 1u && s_task_state <= 3u &&
        (uint32)(now - s_task_candidate_since) < TASK_CURVE_DEBOUNCE_MS) state = s_task_state;
    running = control_run_running();
    phase = (state == 5u || state == 6u) ? g_vision_island_state : 0u;
    if (state != s_task_state || phase != s_task_phase || running != s_task_running) {
        s_task_transition++;
        head = s_task_head;
        if ((uint32)(head - s_task_tail) < TASK_EVENT_CAPACITY) {
            event = &s_task_events[head & (TASK_EVENT_CAPACITY - 1u)];
            event->sequence = s_task_transition; event->uptime = now;
            event->previous = s_task_state; event->current = state;
            event->element = g_vision_active_elem; event->phase = phase; event->running = running;
            __dsync(); s_task_head = head + 1u;
        } else s_task_dropped++;
        s_task_state = state; s_task_phase = phase; s_task_running = running;
    }
    s_task.uptime = now; s_task.transition = s_task_transition; s_task.dropped = s_task_dropped;
    s_task.vision_seq = g_vision_frame_seq; s_task.age = g_vision_age_ms;
    s_task.state = state; s_task.element = g_vision_active_elem; s_task.phase = phase;
    s_task.track_mode = g_vision_track_mode; s_task.running = running;
    s_task.quality = g_vision_quality; s_task.yaw = att.yaw; s_task.speed = Y_Motor_GetSpeedMps();
}

static void task_poll(void)
{
    station_task_snapshot_t snapshot;
    station_task_event_t event;
    uint32 irq, tail, now = g_control_uptime_ms;
    char line[192];
    int len;
    if (!s_station_att) return;
    tail = s_task_tail;
    if (tail != s_task_head && (uint32)(now - s_task_last_event_send) >= 20u) {
        __dsync(); event = s_task_events[tail & (TASK_EVENT_CAPACITY - 1u)];
        len = snprintf(line, sizeof(line), "taskevt:%lu,%lu,%u,%u,%u,%u,%u\n",
            (unsigned long)event.sequence, (unsigned long)event.uptime,
            (unsigned)event.previous, (unsigned)event.current, (unsigned)event.element,
            (unsigned)event.phase, (unsigned)event.running);
        if (len > 0 && len < (int)sizeof(line) && tx_push((const uint8 *)line, (uint32)len)) {
            __dsync(); s_task_tail = tail + 1u; s_task_last_event_send = now;
        }
    }
    if ((uint32)(now - s_task_last_send) < 200u) return;
    s_task_last_send = now;
    irq = interrupt_global_disable(); snapshot = s_task; interrupt_global_enable(irq);
    len = snprintf(line, sizeof(line), "task:%lu,%u,%u,%u,%u,%u,%u,%.3f,%lu,%lu,%.3f,%.3f,%lu\n",
        (unsigned long)snapshot.uptime, (unsigned)snapshot.state, (unsigned)snapshot.element,
        (unsigned)snapshot.phase, (unsigned)snapshot.track_mode, (unsigned)snapshot.running,
        (unsigned)snapshot.age, (double)snapshot.quality, (unsigned long)snapshot.transition,
        (unsigned long)snapshot.dropped, (double)snapshot.yaw, (double)snapshot.speed,
        (unsigned long)snapshot.vision_seq);
    if (len > 0 && len < (int)sizeof(line) &&
        (uint32)(s_tx_head - s_tx_tail) + (uint32)len < VOFA_TX_SIZE - 384u)
        (void)tx_push((const uint8 *)line, (uint32)len);
}
