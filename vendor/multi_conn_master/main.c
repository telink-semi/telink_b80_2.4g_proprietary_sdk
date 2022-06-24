#include "driver.h"
#include "genfsk_ll.h"
#include "common.h"

extern void user_init(void);
extern void app_sync_init(void);
extern void app_sync_task(void);
extern void app_cycle_send_task(void);
extern void proc_user_task();

_attribute_ram_code_sec_noinline_ int main(void)
{
    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    clock_init(SYS_CLK_24M_Crystal);

    user_init();

    app_sync_init();

    while (1)
    {
        proc_user_task();

        app_sync_task();

        app_cycle_send_task();
    }
}
