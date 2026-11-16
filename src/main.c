#include "app.h"
#include "board.h"
#include "encoder.h"
#include "config.h"

int main(void) {
    app_init();
    board_init();
    board_watchdog_start();
    console_init();
    board_watchdog_feed();
    encoder_init();
    board_watchdog_feed();
    uint32_t next_encoder=micros(),last_tick=g_app.control_ticks;
    for (;;) {
        console_poll();
        app_poll();
        uint32_t now=micros();
        if (time_reached(now,next_encoder)) {
            next_encoder=now+ENCODER_POLL_PERIOD_US;
            encoder_poll(0); encoder_poll(1);
        }
        /* 只有主循环和ADC控制中断都仍在推进才喂狗。调试器暂停也不冻结看门狗。 */
        uint32_t tick=g_app.control_ticks;
        if (tick!=last_tick) { board_watchdog_feed(); last_tick=tick; }
    }
}
