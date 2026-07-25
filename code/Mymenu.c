#include "Mymenu.h"
#include "Attitude.h"
#include "Y_Motor.h"
#include <stdio.h>

#define MENU_FONT_H (16U)
#define MENU_VISIBLE_ITEMS (7U)
#define MENU_VALUE_X (160U)
#define MENU_STEP_COUNT (5U)
#define MENU_STATUS_REFRESH_TICKS (20U)

static const float menu_steps[MENU_STEP_COUNT] =
    {
        0.01f, 0.1f, 1.0f, 10.0f, 100.0f};

/*
 * Placeholder values for the menu framework.
 * Replace these variables and the bindings in Menu_Create() with project data.
 */
static float test_value1;
static float test_value2;
static float test_value3;
static float test_value4;
static float test_value5;
static float test_value6;
static float test_value7;
static bool test_value8;

static Menu_Item root;
static Menu_Item *current;
static uint8_t step_index = 2U;
static uint8_t status_refresh_ticks;
static volatile bool status_refresh_pending;
static bool redraw = true;

void Menu_Create(void)
{
    Menu_Item *test1;
    Menu_Item *test2;
    Menu_Item *test3;
    Menu_Item *test4;

    test1 = Create_Menu_Folder_dynamic(&root, "test1");
    test2 = Create_Menu_Folder_dynamic(&root, "test2");
    test3 = Create_Menu_Folder_dynamic(&root, "test3");
    test4 = Create_Menu_Folder_dynamic(&root, "test4");

    Create_Menu_File_dynamic(test1, "test1", &test_value1, float_Box);
    Create_Menu_File_dynamic(test1, "test2", &test_value2, float_Box);
    Create_Menu_File_dynamic(test2, "test3", &test_value3, float_Box);
    Create_Menu_File_dynamic(test2, "test4", &test_value4, float_Box);
    Create_Menu_File_dynamic(test3, "test5", &test_value5, float_Box);
    Create_Menu_File_dynamic(test3, "test6", &test_value6, float_Box);
    Create_Menu_File_dynamic(test4, "test7", &test_value7, float_Box);
    Create_Menu_File_dynamic(test4, "test8", &test_value8, bool_Box);
}

void Menu_Init(void)
{
    ips200_init(IPS200_TYPE_SPI);
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_clear();
    key_init(5U);

    Menu_Tree_Reset(&root, "MENU");
    Menu_Create();
    All_Folder_Menu_Init(&root);
    current = root.First_Son;
    status_refresh_ticks = 0U;
    status_refresh_pending = true;
    redraw = true;
}

static void Menu_Clear_Row(uint8_t row)
{
    ips200_show_string(0U, (uint16)row * MENU_FONT_H,
                       "                              ");
}

static void Menu_Clear_Page(void)
{
    uint8_t row;

    for (row = 0U; row < 15U; row++)
    {
        Menu_Clear_Row(row);
    }
}

static void Menu_Show_Status_Line(uint16_t y, const char *text)
{
    char line[31];

    (void)snprintf(line, sizeof(line), "%-30.30s", text);
    ips200_show_string(0U, y, line);
}

static void Menu_Show_Attitude(void)
{
    attitude_euler_t attitude;
    attitude_performance_t performance;
    char text[31];

    if (Attitude_GetEuler(&attitude))
    {
        (void)snprintf(text, sizeof(text), "P:%7.2f R:%7.2f",
                       attitude.pitch, attitude.roll);
        Menu_Show_Status_Line(208U, text);

        if (Attitude_GetPerformance(&performance))
        {
            (void)snprintf(text, sizeof(text),
                           "Y:%5.1f T:%4luus L:%4.1f%%",
                           attitude.yaw,
                           (unsigned long)performance.last_time_us,
                           performance.cpu_load_percent);
        }
        else
        {
            (void)snprintf(text, sizeof(text), "Y:%7.2f CPU0:OK",
                           attitude.yaw);
        }
        Menu_Show_Status_Line(224U, text);
    }
    else if (eulerAngle.imu_error)
    {
        Menu_Show_Status_Line(208U, "ATTITUDE: IMU ERROR");
        Menu_Show_Status_Line(224U, "");
    }
    else
    {
        Menu_Show_Status_Line(208U, "ATTITUDE: CALIBRATING");
        Menu_Show_Status_Line(224U, "");
    }
}

static void Menu_Show_Y_Motor_Encoder(void)
{
    y_motor_encoder_data_t encoder;
    char text[31];

    Y_Motor_GetEncoder(&encoder);
    (void)snprintf(text, sizeof(text), "ENC5:%6d TOTAL:%10ld",
                   (int)encoder.count_5ms,
                   (long)encoder.total_count);
    Menu_Show_Status_Line(192U, text);
}

static void Menu_Show_Value(const Menu_Item *item, uint16_t y)
{
    if ((item == NULL) || (item->data == NULL))
    {
        return;
    }

    switch (item->kind)
    {
    case int32_Box:
        ips200_show_int(MENU_VALUE_X, y, *(int32_t *)item->data, 9U);
        break;
    case uint32_Box:
        ips200_show_uint(MENU_VALUE_X, y, *(uint32_t *)item->data, 9U);
        break;
    case int16_Box:
        ips200_show_int(MENU_VALUE_X, y, *(int16_t *)item->data, 9U);
        break;
    case uint16_Box:
        ips200_show_uint(MENU_VALUE_X, y, *(uint16_t *)item->data, 9U);
        break;
    case int8_Box:
        ips200_show_int(MENU_VALUE_X, y, *(int8_t *)item->data, 9U);
        break;
    case uint8_Box:
        ips200_show_uint(MENU_VALUE_X, y, *(uint8_t *)item->data, 9U);
        break;
    case float_Box:
        ips200_show_float(MENU_VALUE_X, y, *(float *)item->data, 5U, 2U);
        break;
    case bool_Box:
        ips200_show_string(MENU_VALUE_X, y,
                           (*(bool *)item->data) ? "ON " : "OFF");
        break;
    case MENU_Folder:
        ips200_show_string(MENU_VALUE_X, y, ">");
        break;
    default:
        break;
    }
}

static void Menu_Change_Value(Menu_Item *item, bool increase)
{
    float step = menu_steps[step_index];

    if ((item == NULL) || (item->data == NULL))
    {
        return;
    }

    switch (item->kind)
    {
    case int32_Box:
        *(int32_t *)item->data += increase ? (int32_t)step : -(int32_t)step;
        break;
    case uint32_Box:
        if (increase)
        {
            *(uint32_t *)item->data += (uint32_t)step;
        }
        else if (*(uint32_t *)item->data >= (uint32_t)step)
        {
            *(uint32_t *)item->data -= (uint32_t)step;
        }
        break;
    case int16_Box:
        *(int16_t *)item->data += increase ? (int16_t)step : -(int16_t)step;
        break;
    case uint16_Box:
        if (increase)
        {
            *(uint16_t *)item->data += (uint16_t)step;
        }
        else if (*(uint16_t *)item->data >= (uint16_t)step)
        {
            *(uint16_t *)item->data -= (uint16_t)step;
        }
        break;
    case int8_Box:
        *(int8_t *)item->data += increase ? (int8_t)step : -(int8_t)step;
        break;
    case uint8_Box:
        if (increase)
        {
            *(uint8_t *)item->data += (uint8_t)step;
        }
        else if (*(uint8_t *)item->data >= (uint8_t)step)
        {
            *(uint8_t *)item->data -= (uint8_t)step;
        }
        break;
    case float_Box:
        *(float *)item->data += increase ? step : -step;
        break;
    case bool_Box:
        *(bool *)item->data = increase;
        break;
    default:
        break;
    }

    redraw = true;
}

void Menu_Show(void)
{
    Menu_Item *folder;
    Menu_Item *item;
    uint8_t first_rank;
    uint8_t row;
    char title[31];
    char step_text[20];

    if (current == NULL)
    {
        return;
    }

    folder = current->Father;
    first_rank = (uint8_t)(((current->rank - 1U) / MENU_VISIBLE_ITEMS) *
                               MENU_VISIBLE_ITEMS +
                           1U);

    Menu_Clear_Page();
    (void)snprintf(title, sizeof(title), "%s/", folder->name);
    ips200_show_string(8U, 0U, title);
    (void)snprintf(step_text, sizeof(step_text), "STEP:%.2f",
                   menu_steps[step_index]);
    ips200_show_string(152U, 0U, step_text);
    ips200_draw_line(0U, 17U, 239U, 17U, RGB565_WHITE);

    item = folder->First_Son;
    while ((item != NULL) && (item->rank < first_rank))
    {
        item = item->Next_Brother;
    }

    for (row = 0U; (row < MENU_VISIBLE_ITEMS) && (item != NULL); row++)
    {
        uint16_t y = (uint16_t)(row + 2U) * MENU_FONT_H;

        ips200_show_string(0U, y, (item == current) ? "->" : "  ");
        if ((item == current) && item->selected)
        {
            ips200_show_string(16U, y, "*");
        }
        ips200_show_string(24U, y, item->name);
        Menu_Show_Value(item, y);

        item = item->Next_Brother;
        if (item == folder->First_Son)
        {
            break;
        }
    }

    ips200_draw_line(0U, 160U, 239U, 160U, RGB565_WHITE);
    ips200_show_string(0U, 176U, "K1:+ K2:- K4:OK K3:BACK");
    Menu_Show_Y_Motor_Encoder();
    Menu_Show_Attitude();
}

void Menu_Switch(void)
{
    key_state_enum key1;
    key_state_enum key2;
    key_state_enum key3;
    key_state_enum key4;
    uint32 interrupt_state;

    if (current == NULL)
    {
        return;
    }

    interrupt_state = interrupt_global_disable();
    key1 = key_get_state(KEY_1);
    key2 = key_get_state(KEY_2);
    key4 = key_get_state(KEY_3);
    key3 = key_get_state(KEY_4);
    key_clear_all_state();
    interrupt_global_enable(interrupt_state);

    if (key1 == KEY_SHORT_PRESS)
    {
        if (current->selected)
        {
            Menu_Change_Value(current, true);
        }
        else
        {
            current = current->Last_Brother;
            redraw = true;
        }
    }
    else if (key2 == KEY_SHORT_PRESS)
    {
        if (current->selected)
        {
            Menu_Change_Value(current, false);
        }
        else
        {
            current = current->Next_Brother;
            redraw = true;
        }
    }
    else if (key3 == KEY_SHORT_PRESS)
    {
        if (current->kind == MENU_Folder)
        {
            if (current->First_Son != NULL)
            {
                current = current->First_Son;
                redraw = true;
            }
        }
        else if (!current->selected)
        {
            current->selected = true;
            redraw = true;
        }
        else
        {
            step_index = (uint8_t)((step_index + 1U) % MENU_STEP_COUNT);
            redraw = true;
        }
    }
    else if (key4 == KEY_SHORT_PRESS)
    {
        if (current->selected)
        {
            current->selected = false;
            redraw = true;
        }
        else if ((current->Father != NULL) &&
                 (current->Father->Father != NULL))
        {
            current = current->Father;
            redraw = true;
        }
    }
}

void Menu_KeyScan_5ms_ISR(void)
{
    key_scanner();

    status_refresh_ticks++;
    if (status_refresh_ticks >= MENU_STATUS_REFRESH_TICKS)
    {
        status_refresh_ticks = 0U;
        status_refresh_pending = true;
    }
}

void Menu_Task(void)
{
    Menu_Switch();

    if (redraw)
    {
        redraw = false;
        status_refresh_pending = false;
        Menu_Show();
    }
    else if (status_refresh_pending)
    {
        status_refresh_pending = false;
        Menu_Show_Y_Motor_Encoder();
        Menu_Show_Attitude();
    }
}
