#include "main.h"
#include <stdio.h>
#include <string.h>

/* ================================================================
 * HANDLE
 * ================================================================ */
TIM_HandleTypeDef  htim1;
UART_HandleTypeDef huart6;

/* ================================================================
 * SENSOR
 * ================================================================ */
#define LINE_DETECTED     0
#define LINE_NOT_DETECTED 1

/* ================================================================
 * PWM
 * ================================================================ */
#define PWM_MAX 999
#define PWM_MIN 0

/* ================================================================
 * Tá»C Äá»˜
 * ================================================================ */
#define SPEED_BASE              300
#define SPEED_FAST              700

/* Ráº½ */
#define SPEED_CORNER_APPROACH   110
#define SPEED_TURN_SHARP        630
#define SPEED_TURN_CATCH        520

#define TURN_KICK_MS            50
#define TURN_RAMP_STEP          25
#define TURN_RAMP_INTERVAL_MS   5

#define SPEED_SEARCH            180

/* ================================================================
 * PID THEO LINE
 * ================================================================ */
#define KP_X10               36
#define SPEED_REDUCE_PER_ERR 36
#define SPEED_FLOOR          130
#define FAST_CORR_BOOST      40

/* ================================================================
 * CĂN CHỈNH KHI S2/S4 LỆCH
 * ================================================================ */
#define SPEED_ALIGN              153
#define ALIGN_KICK_MS            10
#define ALIGN_RAMP_STEP          60
#define ALIGN_RAMP_MS            5

/* ================================================================
 * THÔNG SỐ RẼ
 * ================================================================ */
#define SENSOR_OFFSET_MS        100
#define TURN_BRAKE_MS           150
#define TURN_SHARP_MS           250
#define TURN_CATCH_TIMEOUT_MS   1200
#define TURN_STABLE_COUNT       3
#define TURN_LOOP_MS            2
#define TURN_SETTLE_MS          90
#define TURN_EXIT_MS            550
#define TURN_EXIT_SPEED         160
#define TURN_EXIT_SPEED_MIN     110
#define TURN_EXIT_SPEED_MAX     330
#define TURN_EXIT_STABLE        5
#define TURN_EXIT_MISS_MAX      35
#define TURN_COOLDOWN_MS        400
#define CORNER_CONFIRM_COUNT    2

/* ================================================================
 * SENSOR REPORT & PATH LOG
 * ================================================================ */
#define SENSOR_REPORT_INTERVAL_MS  200
#define PATH_LOG_MAX               50

/* ================================================================
 * TRỌNG SỐ CẢM BIẾN
 * S1=-4, S2=-2, S3=0, S4=2, S5=4
 * ================================================================ */
static const int8_t SENSOR_W[5] = {-4, -2, 0, 2, 4};

/* ================================================================
 * STATE
 * ================================================================ */
typedef enum { MODE_LINE = 0, MODE_BT } ControlMode_t;
typedef enum { DIR_STRAIGHT = 0, DIR_LEFT, DIR_RIGHT } LineDir_t;

static ControlMode_t g_mode                = MODE_LINE;
static LineDir_t     g_last_dir            = DIR_STRAIGHT;
static int8_t        g_last_error          = 0;
static uint32_t      g_turn_cooldown_until = 0;
static uint8_t       g_corner_L_cnt        = 0;
static uint8_t       g_corner_R_cnt        = 0;
static uint8_t       bt_rx;

/* ================================================================
 * PATH LOG
 * ================================================================ */
typedef struct {
    uint32_t tick;
    uint8_t  s[5];
    int8_t   error;
    uint8_t  mode;
    uint8_t  dir;
} PathStep_t;

static PathStep_t g_path_log[PATH_LOG_MAX];
static uint8_t    g_path_count  = 0;
static uint32_t   g_sensor_last = 0;

/* ================================================================
 * PROTOTYPE
 * ================================================================ */
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_TIM1_Init(void);
void MX_USART6_UART_Init(void);
void Error_Handler(void);

static uint8_t Read_S1(void);
static uint8_t Read_S2(void);
static uint8_t Read_S3(void);
static uint8_t Read_S4(void);
static uint8_t Read_S5(void);

static void    Read_All(uint8_t s[5]);
static int8_t  Calc_Error(const uint8_t raw[5]);
static uint8_t Is_Contiguous(const uint8_t raw[5]);

static void Set_LeftSpeed(uint16_t speed);
static void Set_RightSpeed(uint16_t speed);

static void Motor_Stop(void);
static void Motor_Brake(void);
static void Motor_Forward(uint16_t left, uint16_t right);
static void Motor_Backward(uint16_t left, uint16_t right);
static void Motor_SpinLeft(uint16_t speed);
static void Motor_SpinRight(uint16_t speed);
static void Motor_PivotLeft(uint16_t speed);
static void Motor_PivotRight(uint16_t speed);
static void Motor_PivotLeft_Turn(uint16_t speed);
static void Motor_PivotRight_Turn(uint16_t speed);

static void Pivot_Kick_And_Ramp_Left(void);
static void Pivot_Kick_And_Ramp_Right(void);
static void Align_Kick_Left(void);
static void Align_Kick_Right(void);

static void Turn_Exit_PID(LineDir_t dir);
static void Turn_Left_90(void);
static void Turn_Right_90(void);

static void Send_Sensor_Data(void);
static void Log_Path_Step(const uint8_t s[5], int8_t err);
static void Print_Path_Log(void);

static void Print_BT_Menu(void);
static void Bluetooth_Process(uint8_t cmd);
static void LineFollow_Process(void);

/* ================================================================
 * MAIN
 * ================================================================ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_USART6_UART_Init();

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    Motor_Stop();
    HAL_Delay(1000);
    HAL_UART_Transmit(&huart6,
                      (uint8_t *)"HC05 UART START OK\r\n",
                      strlen("HC05 UART START OK\r\n"),
                      100);
    Print_BT_Menu();
    while (1)
    {
        /* Non-blocking Bluetooth receive */
        if (HAL_UART_Receive(&huart6, &bt_rx, 1, 10) == HAL_OK)
            Bluetooth_Process(bt_rx);

        if (g_mode == MODE_LINE)
            LineFollow_Process();
    }
}

/* ================================================================
 * LINE FOLLOW
 * ================================================================ */
static void LineFollow_Process(void)
{
    uint8_t raw[5];
    uint8_t total = 0;
    int8_t  error;
    int16_t correction, spd_base, l, r;

    uint32_t now        = HAL_GetTick();
    uint8_t  allow_turn = (now >= g_turn_cooldown_until);

    Read_All(raw);

    for (uint8_t i = 0; i < 5; i++)
        if (raw[i] == LINE_DETECTED) total++;

    /* Bỏ qua pattern không liên tục (nhiễu hoặc giao lộ phức tạp) */
    if (total > 0 && !Is_Contiguous(raw))
        return;

    /* ── Mất line: tìm lại theo hướng cuối ── */
    if (total == 0)
    {
        g_corner_L_cnt = 0;
        g_corner_R_cnt = 0;

        if (g_last_dir == DIR_LEFT || g_last_error < 0)
            Motor_PivotLeft(SPEED_SEARCH);
        else if (g_last_dir == DIR_RIGHT || g_last_error > 0)
            Motor_PivotRight(SPEED_SEARCH);
        else
            Motor_Stop();

        return;
    }

    /* ── Phát hiện góc ── */
    if (allow_turn)
    {
        /*
         * Góc TRÁI: S1 thấy line, S4 & S5 không thấy
         */
        if (raw[0] == LINE_DETECTED     &&
            raw[3] == LINE_NOT_DETECTED &&
            raw[4] == LINE_NOT_DETECTED)
        {
            g_corner_L_cnt++;
            g_corner_R_cnt = 0;

            if (g_corner_L_cnt >= CORNER_CONFIRM_COUNT)
            {
                g_corner_L_cnt = 0;
                Turn_Left_90();
                Log_Path_Step(raw, Calc_Error(raw));
                return;
            }
        }
        /*
         * Góc PHẢI: S5 thấy line, S1 & S2 không thấy
         */
        else if (raw[4] == LINE_DETECTED     &&
                 raw[0] == LINE_NOT_DETECTED &&
                 raw[1] == LINE_NOT_DETECTED)
        {
            g_corner_R_cnt++;
            g_corner_L_cnt = 0;

            if (g_corner_R_cnt >= CORNER_CONFIRM_COUNT)
            {
                g_corner_R_cnt = 0;
                Turn_Right_90();
                Log_Path_Step(raw, Calc_Error(raw));
                return;
            }
        }
        else
        {
            g_corner_L_cnt = 0;
            g_corner_R_cnt = 0;
        }
    }

    /* ── S2+S3+S4 cùng thấy line → đi thẳng ── */
    if (raw[1] == LINE_DETECTED &&
        raw[2] == LINE_DETECTED &&
        raw[3] == LINE_DETECTED)
    {
        Motor_Forward(SPEED_BASE, SPEED_BASE);
        g_last_error = 0;
        g_last_dir   = DIR_STRAIGHT;
        return;
    }

    /* ── Theo line PID ── */
    error        = Calc_Error(raw);
    g_last_error = error;

    if      (error == 0) g_last_dir = DIR_STRAIGHT;
    else if (error  < 0) g_last_dir = DIR_LEFT;
    else                  g_last_dir = DIR_RIGHT;

    /* ── Vùng chỉ S2/S4 lệch (S1 và S5 chưa thấy) ──
     * Pivot tại chỗ để căn về S3, KHÔNG tiến thêm.
     */
    uint8_t only_outer =
        (
            (
                raw[1] == LINE_DETECTED &&
                raw[2] == LINE_NOT_DETECTED &&
                raw[3] == LINE_NOT_DETECTED
            )
            ||
            (
                raw[3] == LINE_DETECTED &&
                raw[2] == LINE_NOT_DETECTED &&
                raw[1] == LINE_NOT_DETECTED
            )
        )
        &&
        raw[0] == LINE_NOT_DETECTED &&
        raw[4] == LINE_NOT_DETECTED;

    if (only_outer)
    {
        if (error < 0)
            Align_Kick_Left();
        else if (error > 0)
            Align_Kick_Right();
        return;
    }

    /* ── PID đường thẳng bình thường ── */
    correction = ((int16_t)KP_X10 * error) / 10;

    spd_base = (int16_t)SPEED_BASE - ((error < 0 ? -error : error) * SPEED_REDUCE_PER_ERR);
    if (spd_base < SPEED_FLOOR) spd_base = SPEED_FLOOR;

    l = spd_base - correction;
    r = spd_base + correction;

    if (error < 0)
        r += FAST_CORR_BOOST;
    else if (error > 0)
        l += FAST_CORR_BOOST;

    if (l < PWM_MIN) l = PWM_MIN;
    if (l > PWM_MAX) l = PWM_MAX;
    if (r < PWM_MIN) r = PWM_MIN;
    if (r > PWM_MAX) r = PWM_MAX;

    Motor_Forward((uint16_t)l, (uint16_t)r);
    Log_Path_Step(raw, error);
}

/* ================================================================
 * ALIGN KICK – căn chỉnh S2/S4 cho xe nặng
 * Dùng Motor_PivotLeft/Right (1 bánh đứng im)
 * ================================================================ */
static void Align_Kick_Left(void)
{
    uint32_t ramp_steps = (PWM_MAX - SPEED_ALIGN) / ALIGN_RAMP_STEP;
    uint16_t spd;

    Motor_PivotLeft(PWM_MAX);
    HAL_Delay(ALIGN_KICK_MS);

    for (uint32_t i = 0; i < ramp_steps; i++)
    {
        spd = (uint16_t)(PWM_MAX - (i + 1) * ALIGN_RAMP_STEP);
        if (spd < SPEED_ALIGN) spd = SPEED_ALIGN;
        Motor_PivotLeft(spd);
        HAL_Delay(ALIGN_RAMP_MS);
    }

    Motor_PivotLeft(SPEED_ALIGN);
}

static void Align_Kick_Right(void)
{
    uint32_t ramp_steps = (PWM_MAX - SPEED_ALIGN) / ALIGN_RAMP_STEP;
    uint16_t spd;

    Motor_PivotRight(PWM_MAX);
    HAL_Delay(ALIGN_KICK_MS);

    for (uint32_t i = 0; i < ramp_steps; i++)
    {
        spd = (uint16_t)(PWM_MAX - (i + 1) * ALIGN_RAMP_STEP);
        if (spd < SPEED_ALIGN) spd = SPEED_ALIGN;
        Motor_PivotRight(spd);
        HAL_Delay(ALIGN_RAMP_MS);
    }

    Motor_PivotRight(SPEED_ALIGN);
}

/* ================================================================
 * PIVOT KICK + RAMP-DOWN – dùng Motor_PivotLeft/Right_Turn
 *
 * 1. Full PWM (999) trong TURN_KICK_MS → bứt quán tính tĩnh bánh sau.
 * 2. Ramp-down về SPEED_TURN_SHARP để không giật.
 * 3. Giữ SPEED_TURN_SHARP cho phần còn lại của TURN_SHARP_MS.
 * ================================================================ */
static void Pivot_Kick_And_Ramp_Left(void)
{
    uint16_t spd;
    uint32_t ramp_steps;
    uint32_t ramp_time_ms;
    uint32_t remaining_ms;

    Motor_PivotLeft_Turn(PWM_MAX);
    HAL_Delay(TURN_KICK_MS);

    ramp_steps   = (PWM_MAX - SPEED_TURN_SHARP) / TURN_RAMP_STEP;
    ramp_time_ms = ramp_steps * TURN_RAMP_INTERVAL_MS;

    for (uint32_t i = 0; i < ramp_steps; i++)
    {
        spd = (uint16_t)(PWM_MAX - (i + 1) * TURN_RAMP_STEP);
        if (spd < SPEED_TURN_SHARP) spd = SPEED_TURN_SHARP;
        Motor_PivotLeft_Turn(spd);
        HAL_Delay(TURN_RAMP_INTERVAL_MS);
    }

    remaining_ms = TURN_SHARP_MS - TURN_KICK_MS - ramp_time_ms;
    if (remaining_ms > 0 && remaining_ms < TURN_SHARP_MS)
    {
        Motor_PivotLeft_Turn(SPEED_TURN_SHARP);
        HAL_Delay(remaining_ms);
    }
}

static void Pivot_Kick_And_Ramp_Right(void)
{
    uint16_t spd;
    uint32_t ramp_steps;
    uint32_t ramp_time_ms;
    uint32_t remaining_ms;

    Motor_PivotRight_Turn(PWM_MAX);
    HAL_Delay(TURN_KICK_MS);

    ramp_steps   = (PWM_MAX - SPEED_TURN_SHARP) / TURN_RAMP_STEP;
    ramp_time_ms = ramp_steps * TURN_RAMP_INTERVAL_MS;

    for (uint32_t i = 0; i < ramp_steps; i++)
    {
        spd = (uint16_t)(PWM_MAX - (i + 1) * TURN_RAMP_STEP);
        if (spd < SPEED_TURN_SHARP) spd = SPEED_TURN_SHARP;
        Motor_PivotRight_Turn(spd);
        HAL_Delay(TURN_RAMP_INTERVAL_MS);
    }

    remaining_ms = TURN_SHARP_MS - TURN_KICK_MS - ramp_time_ms;
    if (remaining_ms > 0 && remaining_ms < TURN_SHARP_MS)
    {
        Motor_PivotRight_Turn(SPEED_TURN_SHARP);
        HAL_Delay(remaining_ms);
    }
}

/* ================================================================
 * RẼ TRÁI
 * ================================================================ */
static void Turn_Left_90(void)
{
    uint32_t t0;
    uint8_t  stable = 0;

    g_last_dir = DIR_LEFT;

    /* Bước 1: Tiến nhẹ đưa cảm biến vào tâm góc */
    Motor_Forward(SPEED_CORNER_APPROACH, SPEED_CORNER_APPROACH);
    HAL_Delay(SENSOR_OFFSET_MS);

    /* Bước 2: Phanh dừng hẳn */
    Motor_Brake();
    HAL_Delay(TURN_BRAKE_MS);

    /* Bước 3: Pivot nhanh open-loop ~65° với kick + ramp */
    Pivot_Kick_And_Ramp_Left();

    /* Bước 4: Pivot catch – chờ S3 hoặc S4 thấy line mới */
    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < TURN_CATCH_TIMEOUT_MS)
    {
        Motor_PivotLeft_Turn(SPEED_TURN_CATCH);

        if (Read_S3() == LINE_DETECTED || Read_S4() == LINE_DETECTED)
        {
            stable++;
            if (stable >= TURN_STABLE_COUNT) break;
        }
        else
        {
            stable = 0;
        }

        HAL_Delay(TURN_LOOP_MS);
    }

    /* Bước 5: Phanh settle */
    Motor_Brake();
    HAL_Delay(TURN_SETTLE_MS);

    /* Bước 6: Exit PID căn vào giữa line */
    Turn_Exit_PID(DIR_LEFT);

    g_last_dir            = DIR_STRAIGHT;
    g_last_error          = 0;
    g_turn_cooldown_until = HAL_GetTick() + TURN_COOLDOWN_MS;
}

/* ================================================================
 * Rẽ phải
 * ================================================================ */
static void Turn_Right_90(void)
{
    uint32_t t0;
    uint8_t  stable = 0;

    g_last_dir = DIR_RIGHT;

    /* Bước 1 */
    Motor_Forward(SPEED_CORNER_APPROACH, SPEED_CORNER_APPROACH);
    HAL_Delay(SENSOR_OFFSET_MS);

    /* Bước 2 */
    Motor_Brake();
    HAL_Delay(TURN_BRAKE_MS);

    /* Bước 3: Pivot nhanh ~65° với kick + ramp */
    Pivot_Kick_And_Ramp_Right();

    /* Bước 4: Catch – chờ S2 hoặc S3 thấy line mới */
    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < TURN_CATCH_TIMEOUT_MS)
    {
        Motor_PivotRight_Turn(SPEED_TURN_CATCH);

        if (Read_S2() == LINE_DETECTED || Read_S3() == LINE_DETECTED)
        {
            stable++;
            if (stable >= TURN_STABLE_COUNT) break;
        }
        else
        {
            stable = 0;
        }

        HAL_Delay(TURN_LOOP_MS);
    }

    /* Bước 5 */
    Motor_Brake();
    HAL_Delay(TURN_SETTLE_MS);

    /* Bước 6 */
    Turn_Exit_PID(DIR_RIGHT);

    g_last_dir            = DIR_STRAIGHT;
    g_last_error          = 0;
    g_turn_cooldown_until = HAL_GetTick() + TURN_COOLDOWN_MS;
}

/* ================================================================
 * TURN EXIT PID
 *
 * Chạy chậm có PID trong TURN_EXIT_MS để căn xe vào giữa line.
 * Thoát sớm nếu error == 0 liên tiếp TURN_EXIT_STABLE lần.
 * Thoát sớm nếu mất line quá TURN_EXIT_MISS_MAX lần liên tiếp.
 * ================================================================ */
static void Turn_Exit_PID(LineDir_t dir)
{
    uint32_t t0       = HAL_GetTick();
    uint8_t  stable   = 0;
    uint8_t  miss_cnt = 0;
    uint8_t  raw[5];
    int8_t   err;
    int16_t  corr, l, r;

    while ((HAL_GetTick() - t0) < TURN_EXIT_MS)
    {
        uint8_t total = 0;

        Read_All(raw);
        for (uint8_t i = 0; i < 5; i++)
            if (raw[i] == LINE_DETECTED) total++;

        if (total == 0)
        {
            miss_cnt++;
            if (miss_cnt > TURN_EXIT_MISS_MAX) break;

            if (dir == DIR_LEFT)
                Motor_Forward(TURN_EXIT_SPEED_MIN, TURN_EXIT_SPEED);
            else
                Motor_Forward(TURN_EXIT_SPEED, TURN_EXIT_SPEED_MIN);

            HAL_Delay(4);
            continue;
        }

        miss_cnt = 0;
        err      = Calc_Error(raw);

        if (err == 0)
        {
            stable++;
            if (stable >= TURN_EXIT_STABLE) break;
        }
        else
        {
            stable = 0;
        }

        corr = ((int16_t)KP_X10 * err) / 10;

        l = TURN_EXIT_SPEED - corr;
        r = TURN_EXIT_SPEED + corr;

        if (l < TURN_EXIT_SPEED_MIN) l = TURN_EXIT_SPEED_MIN;
        if (l > TURN_EXIT_SPEED_MAX) l = TURN_EXIT_SPEED_MAX;
        if (r < TURN_EXIT_SPEED_MIN) r = TURN_EXIT_SPEED_MIN;
        if (r > TURN_EXIT_SPEED_MAX) r = TURN_EXIT_SPEED_MAX;

        Motor_Forward((uint16_t)l, (uint16_t)r);
        HAL_Delay(4);
    }

    Motor_Stop();
    HAL_Delay(20);
}

/* ================================================================
 * GỬI DỮ LIỆU SENSOR LÊN TERMINAL
 *
 * Format:
 *   [tick] S1:x S2:x S3:x S4:x S5:x ERR:xx MODE:LINE DIR:STR
 * ================================================================ */
static void Send_Sensor_Data(void)
{
    uint8_t raw[5];
    int8_t  err;
    char    buf[96];
    int     len;

    Read_All(raw);
    err = Calc_Error(raw);

    len = snprintf(buf, sizeof(buf),
        "[%05lu] S1:%d S2:%d S3:%d S4:%d S5:%d ERR:%d MODE:%s DIR:%s\r\n",
        (unsigned long)HAL_GetTick(),
        raw[0], raw[1], raw[2], raw[3], raw[4],
        (int)err,
        (g_mode == MODE_LINE) ? "LINE" : "BT  ",
        (g_last_dir == DIR_LEFT)  ? "LEFT " :
        (g_last_dir == DIR_RIGHT) ? "RIGHT" : "STR  ");

    HAL_UART_Transmit(&huart6, (uint8_t *)buf, (uint16_t)len, 15);
}

/* ================================================================
 * LƯU 1 BƯỚC ĐƯỜNG ĐI
 *
 * Chỉ lưu khi có thay đổi thực sự (tránh trùng lặp liên tiếp).
 * Khi đầy 50 bước: dịch mảng, bỏ bước cũ nhất (rolling log).
 * ================================================================ */
static void Log_Path_Step(const uint8_t s[5], int8_t err)
{
    if (g_path_count >= PATH_LOG_MAX)
    {
        memmove(&g_path_log[0], &g_path_log[1],
                sizeof(PathStep_t) * (PATH_LOG_MAX - 1));
        g_path_count = PATH_LOG_MAX - 1;
    }

    /* Bỏ qua nếu giống hệt bước trước */
    if (g_path_count > 0)
    {
        PathStep_t *prev = &g_path_log[g_path_count - 1];
        if (prev->error == err        &&
            prev->dir   == (uint8_t)g_last_dir &&
            prev->mode  == (uint8_t)g_mode)
            return;
    }

    PathStep_t *step = &g_path_log[g_path_count++];
    step->tick  = HAL_GetTick();
    step->error = err;
    step->mode  = (uint8_t)g_mode;
    step->dir   = (uint8_t)g_last_dir;
    for (uint8_t i = 0; i < 5; i++) step->s[i] = s[i];
}

/* ================================================================
 * IN TOÀN BỘ LOG ĐƯỜNG ĐI (khi nhấn '?')
 *
 * Format mỗi dòng:
 *   #01 [01234ms] S:00100 ERR:-2 DIR:LEFT  MODE:LINE
 * ================================================================ */
static void Print_Path_Log(void)
{
    char buf[80];
    int  len;

    len = snprintf(buf, sizeof(buf),
                   "\r\n=== PATH LOG (%d/%d steps) ===\r\n",
                   g_path_count, PATH_LOG_MAX);
    HAL_UART_Transmit(&huart6, (uint8_t *)buf, (uint16_t)len, 15);

    for (uint8_t i = 0; i < g_path_count; i++)
    {
        PathStep_t *p = &g_path_log[i];

        len = snprintf(buf, sizeof(buf),
            "#%02d [%05lums] S:%d%d%d%d%d ERR:%+d DIR:%s MODE:%s\r\n",
            (int)(i + 1),
            (unsigned long)p->tick,
            p->s[0], p->s[1], p->s[2], p->s[3], p->s[4],
            (int)p->error,
            (p->dir == (uint8_t)DIR_LEFT)  ? "LEFT " :
            (p->dir == (uint8_t)DIR_RIGHT) ? "RIGHT" : "STR  ",
            (p->mode == (uint8_t)MODE_LINE) ? "LINE" : "BT  ");

        HAL_UART_Transmit(&huart6, (uint8_t *)buf, (uint16_t)len, 15);
    }

    len = snprintf(buf, sizeof(buf), "=== END LOG ===\r\n\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t *)buf, (uint16_t)len, 15);
}

/* ================================================================
 * BLUETOOTH – WASD + sensor/log commands
 *
 * Q / q  : LINE follow
 * M / m  : BT manual
 * W / w  : Tiến
 * S / s  : Lùi
 * A / a  : Trái
 * D / d  : Phải
 * X / x  : Dừng
 * T / t  : Gửi dữ liệu sensor 1 lần ngay lập tức
 * C / c  : Xóa log đường đi
 * ?      : In toàn bộ log đường đi ra terminal
 * ================================================================ */
static void Print_BT_Menu(void)
{
    const char *menu =
        "\r\n===== BLUETOOTH CONTROL MENU =====\r\n"
        "M - Manual mode\r\n"
        "Q - Auto line mode\r\n"
        "W - Forward\r\n"
        "S - Backward\r\n"
        "A - Turn left\r\n"
        "D - Turn right\r\n"
        "X - Stop\r\n"
        "T - Read sensors once\r\n"
        "C - Clear path log\r\n"
        "? - Print path log\r\n"
        "H - Show this menu\r\n"
        "==================================\r\n";

    HAL_UART_Transmit(&huart6, (uint8_t *)menu, strlen(menu), 100);
}

static void Bluetooth_Process(uint8_t cmd)
{
    switch (cmd)
    {
    	case 'H': case 'h':
			Print_BT_Menu();
			break;

		case 'Q': case 'q':
			g_mode         = MODE_LINE;
			g_last_dir     = DIR_STRAIGHT;
			g_last_error   = 0;
			g_corner_L_cnt = 0;
			g_corner_R_cnt = 0;
			Motor_Stop();

			HAL_UART_Transmit(&huart6,
				(uint8_t *)"[MODE] AUTO LINE\r\n",
				strlen("[MODE] AUTO LINE\r\n"),
				100);
			break;

		case 'M': case 'm':
		    g_mode = MODE_BT;
		    Motor_Stop();
		    HAL_UART_Transmit(&huart6,
		        (uint8_t *)"MODE BT OK\r\n",
		        strlen("MODE BT OK\r\n"),
		        100);
		    break;

		case 'W': case 'w':
		    if (g_mode == MODE_BT)
		    {
		        Motor_Forward(SPEED_FAST, SPEED_FAST);
		        HAL_UART_Transmit(&huart6, (uint8_t *)"W - Forward\r\n", strlen("W - Forward\r\n"), 100);
		        HAL_Delay(50);
		        Motor_Stop();
		    }
		    break;

		case 'S': case 's':
		    if (g_mode == MODE_BT)
		    {
		        Motor_Backward(SPEED_FAST, SPEED_FAST);
		        HAL_UART_Transmit(&huart6, (uint8_t *)"S - Backward\r\n", strlen("S - Backward\r\n"), 100);
		        HAL_Delay(50);
		        Motor_Stop();
		    }
		    break;

		case 'A': case 'a':
		    if (g_mode == MODE_BT)
		    {
		        Motor_PivotLeft(SPEED_FAST);
		        HAL_UART_Transmit(&huart6, (uint8_t *)"A - Turn left\r\n", strlen("A - Turn left\r\n"), 100);
		        HAL_Delay(50);
		        Motor_Stop();
		    }
		    break;

		case 'D': case 'd':
		    if (g_mode == MODE_BT)
		    {
		        Motor_PivotRight(SPEED_FAST);
		        HAL_UART_Transmit(&huart6, (uint8_t *)"D - Turn right\r\n", strlen("D - Turn right\r\n"), 100);
		        HAL_Delay(50);
		        Motor_Stop();
		    }
		    break;

		case 'X': case 'x':
		    if (g_mode == MODE_BT)
		    {
		        Motor_Stop();
		        HAL_UART_Transmit(&huart6, (uint8_t *)"X - Stop\r\n", strlen("X - Stop\r\n"), 100);
		    }
		    break;

        // read sensor
        case 'T': case 't':
            // Send_Sensor_Data();
            break;

        case 'C': case 'c':
        {
            g_path_count = 0;
            const char *msg = "[LOG] Path log cleared.\r\n";
            HAL_UART_Transmit(&huart6, (uint8_t *)msg,
                              (uint16_t)strlen(msg), 15);
            break;
        }

        case '?':
            Print_Path_Log();
            break;

        default:
            break;
    }
}

/* ================================================================
 * SENSOR HELPERS
 * ================================================================ */
static void Read_All(uint8_t s[5])
{
    s[0] = Read_S1();
    s[1] = Read_S2();
    s[2] = Read_S3();
    s[3] = Read_S4();
    s[4] = Read_S5();
}

static int8_t Calc_Error(const uint8_t raw[5])
{
    int16_t wsum = 0;
    uint8_t cnt  = 0;

    for (uint8_t i = 0; i < 5; i++)
    {
        if (raw[i] == LINE_DETECTED)
        {
            wsum += SENSOR_W[i];
            cnt++;
        }
    }

    if (cnt == 0) return g_last_error;

    return (int8_t)(wsum / cnt);
}

static uint8_t Is_Contiguous(const uint8_t raw[5])
{
    uint8_t started = 0;
    uint8_t gap     = 0;

    for (uint8_t i = 0; i < 5; i++)
    {
        if (raw[i] == LINE_DETECTED)
        {
            if (gap) return 0;
            started = 1;
        }
        else
        {
            if (started) gap = 1;
        }
    }

    return 1;
}

/* ================================================================
 * ĐỌC SENSOR PIN
 * ================================================================ */
static uint8_t Read_S1(void) { return HAL_GPIO_ReadPin(SENSOR_S1_GPIO_Port, SENSOR_S1_Pin); }
static uint8_t Read_S2(void) { return HAL_GPIO_ReadPin(SENSOR_S2_GPIO_Port, SENSOR_S2_Pin); }
static uint8_t Read_S3(void) { return HAL_GPIO_ReadPin(SENSOR_S3_GPIO_Port, SENSOR_S3_Pin); }
static uint8_t Read_S4(void) { return HAL_GPIO_ReadPin(SENSOR_S4_GPIO_Port, SENSOR_S4_Pin); }
static uint8_t Read_S5(void) { return HAL_GPIO_ReadPin(SENSOR_S5_GPIO_Port, SENSOR_S5_Pin); }

/* ================================================================
 * PWM
 * ================================================================ */
static void Set_LeftSpeed(uint16_t speed)
{
    if (speed > PWM_MAX) speed = PWM_MAX;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
}

static void Set_RightSpeed(uint16_t speed)
{
    if (speed > PWM_MAX) speed = PWM_MAX;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
}

/* ================================================================
 * MOTOR CƠ BẢN
 * ================================================================ */
static void Motor_Stop(void)
{
    HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_IN1_GPIO_Port, M2_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_IN2_GPIO_Port, M2_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M3_IN1_GPIO_Port, M3_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M3_IN2_GPIO_Port, M3_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M4_IN1_GPIO_Port, M4_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M4_IN2_GPIO_Port, M4_IN2_Pin, GPIO_PIN_RESET);
    Set_LeftSpeed(0);
    Set_RightSpeed(0);
}

static void Motor_Brake(void)
{
    HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M2_IN1_GPIO_Port, M2_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M2_IN2_GPIO_Port, M2_IN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M3_IN1_GPIO_Port, M3_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M3_IN2_GPIO_Port, M3_IN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M4_IN1_GPIO_Port, M4_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M4_IN2_GPIO_Port, M4_IN2_Pin, GPIO_PIN_SET);
    Set_LeftSpeed(PWM_MAX);
    Set_RightSpeed(PWM_MAX);
}

static void Motor_Forward(uint16_t left, uint16_t right)
{
    HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_IN1_GPIO_Port, M2_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M2_IN2_GPIO_Port, M2_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M3_IN1_GPIO_Port, M3_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M3_IN2_GPIO_Port, M3_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M4_IN1_GPIO_Port, M4_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M4_IN2_GPIO_Port, M4_IN2_Pin, GPIO_PIN_RESET);
    Set_LeftSpeed(left);
    Set_RightSpeed(right);
}

static void Motor_Backward(uint16_t left, uint16_t right)
{
    HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M2_IN1_GPIO_Port, M2_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_IN2_GPIO_Port, M2_IN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M3_IN1_GPIO_Port, M3_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M3_IN2_GPIO_Port, M3_IN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M4_IN1_GPIO_Port, M4_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M4_IN2_GPIO_Port, M4_IN2_Pin, GPIO_PIN_SET);
    Set_LeftSpeed(left);
    Set_RightSpeed(right);
}

/* ================================================================
 * SPIN – dùng cho Bluetooth test
 * ================================================================ */
static void Motor_SpinLeft(uint16_t speed)
{
    HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M2_IN1_GPIO_Port, M2_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_IN2_GPIO_Port, M2_IN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M3_IN1_GPIO_Port, M3_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M3_IN2_GPIO_Port, M3_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M4_IN1_GPIO_Port, M4_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M4_IN2_GPIO_Port, M4_IN2_Pin, GPIO_PIN_RESET);
    Set_LeftSpeed(speed);
    Set_RightSpeed(speed);
}

static void Motor_SpinRight(uint16_t speed)
{
    HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_IN1_GPIO_Port, M2_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M2_IN2_GPIO_Port, M2_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M3_IN1_GPIO_Port, M3_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M3_IN2_GPIO_Port, M3_IN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M4_IN1_GPIO_Port, M4_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M4_IN2_GPIO_Port, M4_IN2_Pin, GPIO_PIN_SET);
    Set_LeftSpeed(speed);
    Set_RightSpeed(speed);
}

/* ================================================================
 * PIVOT – 1 bánh đứng im, 1 bánh tiến
 * Dùng cho: Align_Kick, tìm line khi mất, Bluetooth manual
 * ================================================================ */
static void Motor_PivotLeft(uint16_t speed)
{
    /* banh trai dung im */
    HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_IN1_GPIO_Port, M2_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_IN2_GPIO_Port, M2_IN2_Pin, GPIO_PIN_RESET);
    /* banh phai tien */
    HAL_GPIO_WritePin(M3_IN1_GPIO_Port, M3_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M3_IN2_GPIO_Port, M3_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M4_IN1_GPIO_Port, M4_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M4_IN2_GPIO_Port, M4_IN2_Pin, GPIO_PIN_RESET);
    Set_LeftSpeed(0);
    Set_RightSpeed(speed);
}

static void Motor_PivotRight(uint16_t speed)
{
    /* banh trai tien */
    HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_IN1_GPIO_Port, M2_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M2_IN2_GPIO_Port, M2_IN2_Pin, GPIO_PIN_RESET);
    /* banh phai dung im */
    HAL_GPIO_WritePin(M3_IN1_GPIO_Port, M3_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M3_IN2_GPIO_Port, M3_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M4_IN1_GPIO_Port, M4_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M4_IN2_GPIO_Port, M4_IN2_Pin, GPIO_PIN_RESET);
    Set_LeftSpeed(speed);
    Set_RightSpeed(0);
}

/* ================================================================
 * PIVOT DIFFERENTIAL – chỉ dùng khi rẽ 90°

 * Giảm ma sát 4 bánh → xe quay mượt hơn.
 * ================================================================ */
static void Motor_PivotLeft_Turn(uint16_t speed)
{
    /* Bánh trái đứng im */
    HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M2_IN1_GPIO_Port, M2_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_IN2_GPIO_Port, M2_IN2_Pin, GPIO_PIN_SET);
    /* Bánh phải tiến */
    HAL_GPIO_WritePin(M3_IN1_GPIO_Port, M3_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M3_IN2_GPIO_Port, M3_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M4_IN1_GPIO_Port, M4_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M4_IN2_GPIO_Port, M4_IN2_Pin, GPIO_PIN_RESET);
    Set_LeftSpeed(0);
    Set_RightSpeed(speed);
}

static void Motor_PivotRight_Turn(uint16_t speed)
{
    /* Bánh trái tiến */
    HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_IN1_GPIO_Port, M2_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M2_IN2_GPIO_Port, M2_IN2_Pin, GPIO_PIN_RESET);
    /* Bánh phải đứng im */
    HAL_GPIO_WritePin(M3_IN1_GPIO_Port, M3_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M3_IN2_GPIO_Port, M3_IN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M4_IN1_GPIO_Port, M4_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M4_IN2_GPIO_Port, M4_IN2_Pin, GPIO_PIN_SET);
    Set_LeftSpeed(speed);
    Set_RightSpeed(0);
}

/* ================================================================
 * GPIO INIT
 * ================================================================ */
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOA, M1_IN1_Pin | M1_IN2_Pin | M2_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,
                      M2_IN2_Pin | M3_IN1_Pin | M3_IN2_Pin |
                      M4_IN1_Pin | M4_IN2_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = M1_IN1_Pin | M1_IN2_Pin | M2_IN1_Pin;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = M2_IN2_Pin | M3_IN1_Pin | M3_IN2_Pin |
                          M4_IN1_Pin | M4_IN2_Pin;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;

    GPIO_InitStruct.Pin = SENSOR_S1_Pin;
    HAL_GPIO_Init(SENSOR_S1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SENSOR_S2_Pin;
    HAL_GPIO_Init(SENSOR_S2_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SENSOR_S3_Pin;
    HAL_GPIO_Init(SENSOR_S3_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SENSOR_S4_Pin;
    HAL_GPIO_Init(SENSOR_S4_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SENSOR_S5_Pin;
    HAL_GPIO_Init(SENSOR_S5_GPIO_Port, &GPIO_InitStruct);
}

/* ================================================================
 * TIM1 PWM – 1kHz tại 84MHz
 * Prescaler=84 → 1MHz tick, Period=1000 → 1kHz PWM
 * ================================================================ */
void MX_TIM1_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC      = {0};
    GPIO_InitTypeDef   GPIO_InitStruct = {0};

    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 84 - 1;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 1000 - 1;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();

    GPIO_InitStruct.Pin       = EN_LEFT_Pin | EN_RIGHT_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* ================================================================
 * USART6 – 9600 baud, Bluetooth HC-05
 * PC6 = USART6_TX -> HC05 RXD
 * PC7 = USART6_RX <- HC05 TXD
 * ================================================================ */
void MX_USART6_UART_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_USART6_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin       = HC05_TX_Pin | HC05_RX_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    huart6.Instance          = USART6;
    huart6.Init.BaudRate     = 9600;
    huart6.Init.WordLength   = UART_WORDLENGTH_8B;
    huart6.Init.StopBits     = UART_STOPBITS_1;
    huart6.Init.Parity       = UART_PARITY_NONE;
    huart6.Init.Mode         = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart6) != HAL_OK) Error_Handler();
}

/* ================================================================
 * CLOCK – 84 MHz (HSI + PLL, voltage scale 2)
 * ================================================================ */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 16;
    RCC_OscInitStruct.PLL.PLLN            = 336;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ            = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

/* ================================================================
 * ERROR HANDLER
 * ================================================================ */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
