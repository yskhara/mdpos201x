# 概要
　ブラシDCモータドライバmdpos201xのファームウェアです．速度制御または位置制御を行うことができます．
　STマイクロエレクトロニクス社のLL/HALライブラリなどのIPを含みます．

# コンパイル
　インクルードディレクトリパスにInc/，Drivers/STM32F1xx_HAL_Driver/Inc/，Drivers/STM32F1xx_HAL_Driver/Inc/Legacy/，Drivers/CMSIS/Device/ST/STM32F1xx/Include/，Drivers/CMSIS/Include/を，シンボルにUSE_FULL_LL_DRIVER，__weak="__attribute__((weak))"，__packed="__attribute__((__packed__))"，USE_HAL_DRIVER，STM32F103xBを指定する必要があります．
　main.hppにてCTRL_POSマクロを有効にすると位置制御，CTRL_VELマクロを有効にすると速度制御のファームウェアになります．

# 設定
　制御ゲインなど，いくつかのパラメータは
