# V3.2.0

### Version
* SDK version : telink_b80_2.4g_proprietary_sdk V3.2.0

### Features
* Add rf_pa diver.
* Add gen_fsk_pa routine.

### BREAKING CHANGES

* N/A

# V3.2.0

### Version
* SDK 版本: telink_b80_2.4g_proprietary_sdk V3.2.0

### Features
* 增加rf_pa驱动。
* 增加gen_fsk_pa例程。

### BREAKING CHANGES

* N/A



# V3.1.0

### Version
* SDK version : telink_b80_2.4g_proprietary_sdk V3.1.0.
* This version of SDK supports B80(A1).
* The default configuration of LEDs and KEYs match the following hardware revision:
*	B80	 	C1T261A30_V1_1

### Dependency Updates
* telink_b85m_driver_sdk_V1.5.0.

### Features
* Add adc temperature test and random generation functions, and combine them with the original adc gpio sampling and adc Vbat sampling routines to add adc_ Sample routine.
* Add cpu_long_sleep_wakeup interface usage in the original sleep_wakeup routines.
* Add flash operation models GD25LD10C, GD25LD40C, P25Q40SU, P25D09U, delete flash P25D40L and update related routines.
* Add the functions of triggering interrupts on the rising or falling edge of gpio and gpio interrupt group interrupts.
* Add idle_gpio_wakeup\idle_timer_wakeup\key_scan\pwm_count\pwm_ir\pwm_ir_dma_fifo\pwm_ir_fifo\uart_soft_rx\usb_demo routines.
* New otp startup function.
* Add. sdk_ The version section stores version information.
* Add required wd_32k_stop、user_read_flash_value_calib interface after cpu_wakeup_init function.
* Remove cstartup_ Sram. S file, the functions merged into cstartup_ Flash. S and new cstartup_ otp. S files.

### BREAKING CHANGES

* N/A

# V3.1.0

### Version
* SDK 版本: telink_b80_2.4g_proprietary_sdk V3.1.0。
* 此版本SDK支持B80(A1)。
* LED和KEY的默认配置匹配以下硬件版本:
*	B80	 	C1T261A30_V1_1

### Dependency Updates
* telink_b85m_driver_sdk_V1.5.0。

### Features
* 新增adc温度测试与随机生成功能并与原有的adc的gpio采样、adc的Vbat采样例程合并至新增adc_sample例程中。
* 在原有sleep_wakeup例程中新增cpu_long_sleep_wakeup接口用法。
* 新增flash操作型号GD25LD10C、GD25LD40C、P25Q40SU、P25D09U，删除 flash P25D40L型号并更新相关例程。
* 新增gpio上升沿或下降沿触发中断以及gpio中断组中断功能。
* 新增idle_gpio_wakeup\idle_timer_wakeup\key_scan\pwm_count\pwm_ir\pwm_ir_dma_fifo\pwm_ir_fifo\uart_soft_rx\usb_demo例程。
* 新增otp启动功能。
* 新增.sdk_version段存放版本信息。
* 在初始化cpu_wakeup_init函数后增加必需的wd_32k_stop、user_read_flash_value_calib接口。
* 删除cstartup_sram.S文件，功能合并至cstartup_flash.S和新增的cstartup_otp.S文件内。

### BREAKING CHANGES

* N/A



# V3.0.0

### Version
* SDK version : telink_b80_2.4g_proprietary_sdk V3.0.0.
* This version sdk is a initial version for b80.

### Features

* add tl_esb_ll lib
* add frequency hopping low energy demo

### BREAKING CHANGES
* N/A

### CodeSize

* Flash：
  * freq_hopping_master: 9kb
  * freq_hopping_slave: 12kb

* RAM:
  * freq_hopping_master: 7kb
  * freq_hopping_slave: 9k
  
  

<hr style="border-bottom:2.5px solid rgb(146, 240, 161)">

### Version
* SDK 版本: telink_b80_2.4g_proprietary_sdk V3.0.0.
* 此版本为支持 b80 芯片的初始 2.4g sdk 版本

### Features

* 新增 tl_esb_ll 库
* 新增 跳频低功耗 demo

### BREAKING CHANGES
* N/A

### CodeSize

* Flash：
  * freq_hopping_master: 9kb
  * freq_hopping_slave: 12kb

* RAM:
  * freq_hopping_master: 7kb
  * freq_hopping_slave: 9k
