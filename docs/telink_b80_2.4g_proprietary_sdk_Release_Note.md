# Release Note

## V3.2.4

### Version
* SDK version : telink_b80_2.4g_proprietary_sdk V3.2.4

### Refactoring

* N/A

### Features

* Add interfaces to get rx_packet and pid of tpll.

### Bug Fixes

* Fix the problem that the local CRC will not be updated and the next four packets will all be repeat packets when received a wrong crc packet.
* Update the RF related enumeration class to solve the problem of incorrect power enumeration values.
* Update the interrupt status flag bit of i2c_slave_dma to solve the problem of not receiving data correctly.

### BREAKING CHANGES

* N/A

## V3.2.4

### Version
* SDK 版本: telink_b80_2.4g_proprietary_sdk V3.2.4

### Refactoring

* N/A

### Features

* 添加获取tpll rx_packet地址以及pid的接口。

### Bug Fixes

* 在tl_tpll中修复了当收到crc错误包时，local pid与crc不变，接下去四包连续为重复包的问题。
* 更新rf相关枚举类来解决power枚举值错误问题。
* 更新i2c_slave_dma的中断状态标志位来解决无法正确接收数据问题。

### BREAKING CHANGES

* N/A


## V3.2.3

### Version
* SDK version : telink_b80_2.4g_proprietary_sdk V3.2.3

### Refactoring

* N/A

### Features
* 修改docs文件和一些标注。

### Bug Fixes

* N/A

### BREAKING CHANGES

* N/A

## V3.2.3

### Version
* SDK 版本: telink_b80_2.4g_proprietary_sdk V3.2.3

### Refactoring

* N/A

### Features
* change docs files and some info.

### Bug Fixes

* N/A

### BREAKING CHANGES

* N/A

## V3.2.2

### Version
* SDK version : telink_b80_2.4g_proprietary_sdk V3.2.2

### Refactoring
* Put all non-public driver files in the drivers/lib folder.

### Features

* Add the crc function of the firmware update routine and put the relevant script files into the new script folder.
* Add stx/srx/stx2rx/srx2tx routines of tpsll（Telink proprietary stack link layer）.

### Bug Fixes

* Solve the problem that  gotten  payload length is packet length but not real payload length in General fsk fix packet format.
* Sovle the problem of turning into bricks after power failure when firmware updating.

### BREAKING CHANGES

* N/A

## V3.2.2

### Version
* SDK 版本: telink_b80_2.4g_proprietary_sdk V3.2.2

### Refactoring
* 将不公开驱动文件全部放入drivers/lib文件夹下。

### Features
* 添加固件更新例程的crc校验功能并将相关脚本文件放入新建的script文件夹中。
* 添加tpsll（Telink proprietary stack link layer）的stx/srx/stx2rx/srx2tx例程。

### Bug Fixes

* 解决General fsk定长包获取的rx_payload_len为packet length而不是实际payload length的问题。
* 解决固件更新过程中断电变砖问题。

### BREAKING CHANGES

* N/A

## V3.2.1

### Version
* SDK version : telink_b80_2.4g_proprietary_sdk V3.2.1

### Features
*  change channel step setting from one to 0.5.

### Bug Fixes
* Update MI related enumeration variables.

### BREAKING CHANGES

* N/A

## V3.2.1

### Version
* SDK 版本: telink_b80_2.4g_proprietary_sdk V3.2.1

### Features
* 将channel step从一个step修改到0.5 step。

### Bug Fixes
* 更新 MI 相关的枚举变量。

### BREAKING CHANGES

* N/A

# V3.2.0

### Version
* SDK version : telink_b80_2.4g_proprietary_sdk V3.2.0.

### Features
* Add rf_pa diver.
* Add gen_fsk_pa routine.

### BREAKING CHANGES

* N/A

# V3.2.0

### Version
* SDK 版本: telink_b80_2.4g_proprietary_sdk V3.2.0。

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

* add tl_tpll lib
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

* 新增 tl_tpll 库
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
