# 初见STM32

参考教程:[STM32入门教程-2023版 细致讲解 中文字幕_哔哩哔哩_bilibili](https://www.bilibili.com/video/BV1th411z7sn/?spm_id_from=333.337.search-card.all.click&vd_source=44773dbff08b7facd4f09076f39af4ed)

​				[7天入门STM32_HAL库版，手把手带写程序，项目实战教学视频 【持续更新中】_哔哩哔哩_bilibili](https://www.bilibili.com/video/BV1ZmGQzsE4N/?spm_id_from=333.337.search-card.all.click&vd_source=44773dbff08b7facd4f09076f39af4ed)

​				  [CubeMX使用FreeRTOS编程指南_cubemx freertos-CSDN博客](https://blog.csdn.net/qq_45396672/article/details/120877303)

​				  [STM32 MPU6050 六轴陀螺仪教程（HAL 库零基础入门）_stm32 hal mpu6050-CSDN博客](https://blog.csdn.net/h050210/article/details/145936555)

​				   [PID入门教程-电机控制 倒立摆 全程手把手打代码调试_哔哩哔哩_bilibili](https://www.bilibili.com/video/BV1G9zdYQEr3?spm_id_from=333.788.videopod.episodes&vd_source=44773dbff08b7facd4f09076f39af4ed)

​					[CAN总线入门教程-全面细致 面包板教学 多机通信_哔哩哔哩_bilibili](https://www.bilibili.com/video/BV1vu4m1F7Gt/?spm_id_from=333.1007.top_right_bar_window_default_collection.content.click&vd_source=44773dbff08b7facd4f09076f39af4ed)

​				   DeepSeek

我所学习的STM32的型号及参数:

* STM32F103C8T6
* 系列: 主流系列STM32F1
* 内核:ARM Cortex-M3
* 主频:72MHz
* RAM:20K (SRAM)
* ROM:64K (Flash)
* 供电:2.0~3.6V (标准3.3V)
* 封装:LQFP48

***

## 一. STM32概述

* STM32是ST公司基于ARM Cortex-M内核开发的32位微控制器
* STM32外设:

![Snipaste_2025-11-03_13-49-13](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-03_13-49-13.png)*(我所学习的型号没有最后的四个外设)*

* 芯片命名规则与其参数有关

* 系统结构图*~~虽然我看不太懂但是它很重要~~* :![QQ20251103-122130](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/QQ20251103-122130.png)

* 引脚定义:

  ![图片1](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/%E5%9B%BE%E7%89%871.png)

* 启动配置:

  ![Snipaste_2025-11-03_13-50-19](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-03_13-50-19.png)

  注:我所使用的是通过改变跳线帽的位置来改变BOOT的高低电平,详见下图

  ![IMG_0848](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/IMG_0848.JPG)

* 最小系统电路:

![QQ20251103-131515](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/QQ20251103-131515.png)

* 核心板原理图: 我把它上传到了我的GitHub上,这里仅给出[链接](https://github.com/AAAxiaohuangyu/MCU-Learning/blob/master/STM32F103C8T6.pdf)

  ***

  ## 二.新建工程
  
  ### 1.直接在Keil5中新建工程
  
  ***
  
  ~~我想过会遇到困难但没想到会来的这么快~~
  
  ***
  
  在添加了STM32启动文件,外设寄存器描述文件,时钟配置文件,内核寄存器描述文件,并添加了头文件路径之后我试图通过运行以下代码来检查工程是否可行.
  
  ```c
  #include <stm32f10x.h>
  int main(void)
  {
      while(1){
          
      }
  }
  
  ```
  
  然而在build之后出现了error:
  
  ![Snipaste_2025-11-03_17-18-03](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-03_17-18-03.png)

​		这应该是编译器的路径配置错误,所以我修改了编译器的路径配置:

![Snipaste_2025-11-03_17-20-09](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-03_17-20-09.png)

再次build,error果然减少了*~~此时我还没有意识到事情的严重性~~*

![Snipaste_2025-11-03_17-28-43](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-03_17-28-43.png)

我检查了我的用户名和全局变量TEMP,TMP并进行了修改,以及检查了我的路径中无中文来进一步debug但仍无法解决.(AI甚至告诉我是我的证书过期了)

直到我一次重启之后,再次打开Keil5,**它跳出了一个Warnning!!**

![Snipaste_2025-11-03_17-57-59](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-03_17-57-59.png)

它提示我我的编译器版本不对,但之前Keil5并没有给我这个warning

而不论什么渠道我也得不到相应的问题解决方案,于是我宕机了......

一段时间后我突然想到可能两个原因会导致这些问题:

**1.我的keil5并不是从我看的教程的渠道下载的,而我并没有检查版本信息,因此可能会与教程提供的文件产生版本的冲突**

**2.我的Keil5安装路径中尽管没有中文但是有"_",可能它也不接受特殊字符**

于是我针对以上问题进行了调整,再次build之后:

![QQ20251103-225359](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/QQ20251103-225359.png)

终于问题得到了解决!

***

~~秋豆麻袋,让我们忘记那些不美好的debug并进入正题.~~

### 启动文件选择

可以参考下表:

![Snipaste_2025-11-04_00-56-03](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-04_00-56-03.png)

### 新建工程步骤

1. 建立工程文件夹,Keil5中新建工程,选择型号
2. 工程文件夹中建立Start(启动文件),Library(库函数文件),User(头文件,自己写的程序)文件夹(为了保证工程的独立性),并复制所需文件
3. 工程中建立同名分组并添加文件到分组中
4. 工程选项/'c/c++'中Include Paths内声明所以包含头文件的文件夹(一般为Start,Library,User);Define内定义USE_STDPERIPH_DRIVER
5. 工程选项/Debug中选择对应调试器;Settings/Flash Download里勾选Reset and Run

### 工程架构

各文件作用可以参考下图:

![Snipaste_2025-11-04_00-53-14](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-04_00-53-14.png)

***

## 二.利用STM32CubeMX新建工程

#### 新建工程步骤

1. 新建工程并选择对应芯片

2. 配置SYS,RCC(详情如下):

   Pinout&Configuration
   SYS/Debug:Serial wire
   RCC/ High Speed Clock (HSE):Crystal/Ceramic Resonator
        	 Low Speed Clock (LSE):Crystal/Ceramic Resonator

3. 配置时钟部分(详情如下):

   Clock Configuration:

   PLL Source Mux:HSE

   *PLL Mul:x9

   System Clock Mux:PLLCLK

   APB1 Prescaler:/2

4. 配置项目其他部分:

   打开工程文件后仿照在Keil5中直接新建工程的5.

   在项目文件夹中建立Roc文件夹,并在其中建立README.txt,并添加说明文字如"该程序是STM32CubeMX新建的MDK工程"以方便使用,并在工程中添加,仿照在Keil5中直接新建工程的3.

   

***

但是我在建立工程的时候遇到了以下问题:

![Snipaste_2025-11-04_22-07-28](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-04_22-07-28.png)

但这似乎只影响我的文件不一定是updated的所以它并不影响我的使用

***

## 三.第一次点灯

接下来就是最激动人心的时刻了,不仅仅是对前期漫长的准备的验证也是学习STM32的第一步------点灯!

***

### 1.通过配置寄存器来点亮PC13LED

代码:

```c
#include "stm32f10x.h"                  // Device header

int main()
{
	RCC->APB2ENR = 	0x00000010; //使能GPIOC时钟
	GPIOC->CRH = 0x00300000; //PC13通用推挽输出模式,最大速度50MHz
	GPIOC->ODR = 0x00000200; //数据输出,00000000为低电平(亮),00002000为高电平(灭)
	while(1){
	
	}
	return 0;
}

```

效果(以亮为例):

![QQ20251104-011218](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/QQ20251104-011218.png)

**但是这种编写代码的方式是很低效的(因为还得对照手册看寄存器中每一位对应什么),用标准库则更加的方便**

***

### 2.通过标准库函数来点亮PC13LED

代码:

```c
#include "stm32f10x.h"                  // Device header

int main()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE); //使能GPIOC时钟
	GPIO_InitTypeDef GPIO_InitStructure; //定义结构体类型,下面对其配置参数
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//通用推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;//GPIO端口
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//GPIO端口速度
	GPIO_Init(GPIOC,&GPIO_InitStructure); 
	//GPIO_ResetBits(GPIOC,GPIO_Pin_13); //低电平(亮)
	GPIO_SetBits(GPIOC,GPIO_Pin_13); //高电平(灭)
	while(1){
	
	}
	return 0;
}

```

效果(以灭为例):![QQ20251104-011250](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/QQ20251104-011250.png)

***

### 3.利用HAL库函数来点亮PC13LED

首先根据原理图找出所需的GPIO,且得知输出低电平可以点亮PC13(原理图上述已给出):

![Snipaste_2025-11-04_23-16-40](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-04_23-16-40.png)![Snipaste_2025-11-04_23-16-27](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-04_23-16-27.png)

然后选择相应的GPIO,并选择GPIO_OutPut模式,推挽输出,低电平![Snipaste_2025-11-04_23-25-56](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-04_23-25-56.png)

生成代码后我们可以发现其与我们写的标准库函数很像,但无论是标准库还是HAL库都是通过配置寄存器来操作的,它们只是将其封装

![Snipaste_2025-11-04_23-21-03](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-04_23-21-03.png)

*效果图这里不再展示*

***

### 4.对比与总结

**从以上三种方式可以看出,配置寄存器的方法最基础,但也最繁琐,而基于标准库和HAL库的方法更加的公式化易于理解,并且基于HAL库的方法比基于标准库的方法更加的简易与友好**

***

# 1.1 闪烁LED

## 基础知识

* GPIO（General Purpose Input Output）通用输入输出口,可配置为8种输入输出模式,引脚电平：0V~3.3V，部分引脚可容忍5V
  输出模式下可控制端口输出高低电平，用以驱动LED、控制蜂鸣器、模拟通信协议输出时序等
  输入模式下可读取端口的高低电平或电压，用于读取按键输入、外接模块电平信号输入、ADC电压采集、模拟通信协议接收数据等

* GPIO基本结构:各个寄存器和驱动器等统称为GPIOA,GPIOB等并连接16个引脚,如PA0~PA15,引脚的电平有寄存器决定并由驱动器赋予

* 配置GPIO模式:

  ![Snipaste_2025-11-06_01-12-46](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-06_01-12-46.png)

  *其中复用的意思是由单片机上的外设控制而非单片机内核*

  *高阻态既不是高电平也不是低电平,相当于断开,对电路无影响*

  *上拉默认高电平,下拉默认低电平,这与输入部分也很相似*

***

## 工程建立

我们利用HAL_GPIO_WritePin和HAL_Dely这两个函数即可实现,我这里以PC13LED为例

***

建立工程后我们可以在头文件中得知HAL_GPIO_WritePin和HAL_Dely这两个函数的用法以及作用:

```c
/**
  * @brief  Sets or clears the selected data port bit.
  *
  * @note   This function uses GPIOx_BSRR register to allow atomic read/modify
  *         accesses. In this way, there is no risk of an IRQ occurring between
  *         the read and the modify access.
  *
  * @param  GPIOx: where x can be (A..G depending on device used) to select the GPIO peripheral
  * @param  GPIO_Pin: specifies the port bit to be written.
  *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
  * @param  PinState: specifies the value to be written to the selected bit.
  *          This parameter can be one of the GPIO_PinState enum values:
  *            @arg GPIO_PIN_RESET: to clear the port pin
  *            @arg GPIO_PIN_SET: to set the port pin
  * @retval None
  */
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
  /* Check the parameters */
  assert_param(IS_GPIO_PIN(GPIO_Pin));
  assert_param(IS_GPIO_PIN_ACTION(PinState));

  if (PinState != GPIO_PIN_RESET)
  {
    GPIOx->BSRR = GPIO_Pin;
  }
  else
  {
    GPIOx->BSRR = (uint32_t)GPIO_Pin << 16u;
  }
}
```

```c
/**
  * @brief This function provides minimum delay (in milliseconds) based
  *        on variable incremented.
  * @note In the default implementation , SysTick timer is the source of time base.
  *       It is used to generate interrupts at regular time intervals where uwTick
  *       is incremented.
  * @note This function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @param Delay specifies the delay time length, in milliseconds.
  * @retval None
  */
__weak void HAL_Delay(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;

  /* Add a freq to guarantee minimum wait */
  if (wait < HAL_MAX_DELAY)
  {
    wait += (uint32_t)(uwTickFreq);
  }

  while ((HAL_GetTick() - tickstart) < wait)
  {
  }
}
```

HAL_GPIO_WritePin就是把GPIO端口位(GPIOA,B,C等)和端口内的引脚与电平相对应

HAL_Dely就是等待多少毫秒,它的实现方式还是很简单但是也蛮巧妙的就是通过while死循环来不断等待知道等待的时间达到了要求,这一时间通过获取系统的ticks做差来计算

***

*~~接下来是STM32CubeMX的魅力时刻~~*

```c
/* Private defines -----------------------------------------------------------*/
#define LEDPC13_Pin GPIO_PIN_13
#define LEDPC13_GPIO_Port GPIOC
```

```c
/*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LEDPC13_GPIO_Port, LEDPC13_Pin, GPIO_PIN_SET);
```

它已经帮我们定义好了一个便于记忆和理解的名字

并且给了我们一个便于copy的模板(这与建立工程时的选择有关)

***

于是我们可以自然地写出以下代码:

```c
	  HAL_GPIO_WritePin(LEDPC13_GPIO_Port, LEDPC13_Pin, GPIO_PIN_SET);
	  HAL_Delay(500);
	  HAL_GPIO_WritePin(LEDPC13_GPIO_Port, LEDPC13_Pin, GPIO_PIN_RESET);
	  HAL_Delay(500);
```

*注:以上内容需写在STM32CubeMX为我们在main中准备好的while(1){}死循环中,且亮-灭一个周期一共一秒(这个我一开始搞错了)*

于是我们便实现了1Hz的PC13LED闪烁

***

# 1.2 按键控制

*我经历了一次既让人红温又收获满满的debug*

***

## 基础知识

* 中断:在主程序运行过程中，出现了特定的中断触发条件（中断源），使得CPU暂停当前正在运行的程序，转而去处理中断程序,即回调函数，处理完成后又返回原来被暂停的位置继续运行

* 中断优先级:就是有多个中断要处理时它们的优先级

* 中断嵌套:高优先级的中断中断了中断~~*好拗口*~~

* NVIC:帮助cpu接受中断请求并排序输出给cpu

  ![Snipaste_2025-11-06_00-15-17](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-06_00-15-17.png)

  NVIC优先级分组:NVIC的中断优先级由优先级寄存器的4位（0~15）决定，这4位可以进行切分，分为高n位的抢占优先级和低4-n位的响应优先级,抢占优先级高的可以中断嵌套，响应优先级高的可以优先排队，抢占优先级和响应优先级均相同的按中断号排队

![Snipaste_2025-11-06_00-17-48](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-06_00-17-48.png)

* EXTI（Extern Interrupt）外部中断,EXTI可以监测指定GPIO口的电平信号，当其指定的GPIO口产生电平变化时，EXTI将立即向NVIC发出中断申请，经过NVIC裁决后即可中断CPU主程序，使CPU执行EXTI对应的中断程序
  支持的触发方式：上升沿/下降沿/双边沿/软件触发
  支持的GPIO口：所有GPIO口，但相同的Pin不能同时触发中断
  通道数：16个GPIO_Pin，外加PVD输出、RTC闹钟、USB唤醒、以太网唤醒
  触发响应方式：中断响应/事件响应

  ![Snipaste_2025-11-06_00-21-54](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-06_00-21-54.png)

***

## 工程建立

* 模式配置:PB3引脚:GPIO_EXTI3  外部中断模式,下降沿触发,上拉输入(为了保证稳定,且在我的配置下按下按钮			     输入低电平)

  ​				 PC13引脚:GPIO_Output,推挽输出

* 实物图:

  ![IMG_0937](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/IMG_0937.JPG)

***

第一次代码:

```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == KEYPB3_Pin)
  {
    HAL_Delay(20);
    if (HAL_GPIO_ReadPin(KEYPB3_GPIO_Port, KEYPB3_Pin) == GPIO_PIN_RESET)
    {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
    while (HAL_GPIO_ReadPin(KEYPB3_GPIO_Port, KEYPB3_Pin) == GPIO_PIN_RESET);
  }
}
```

对回调函数重定义,判断提交中断申请的引脚,并利用delay来消除抖动,**似乎一切都很合理且美好**

但是当我按下按键的时候却毫无反应,我又宕机了,因为这在我看来是不可理喻的!

于是我便开始debug从配置到硬件链接,然后去各个blog找案例,问ai但都没什么作用,以至于我最后甚至怀疑是引脚的问题并换了一个引脚,**但是问题依然存在**

最后我在deepseek的指导下写了5,6个检验程序,并测量了引脚电压,最后把问题锁定在了我写的程序上

在一次次检测中有一次的代码成功实现了目标,这也是最终的代码:

```c
/* USER CODE BEGIN PV */
volatile uint8_t key_pressed = 0;
/* USER CODE END PV */

while (1)
  {
	 if (key_pressed) {
        key_pressed = 0; //方便下一次按键监测
        HAL_Delay(20);  // 消抖
        
        // 确认按键确实按下
        if (HAL_GPIO_ReadPin(KEYPB3_GPIO_Port, KEYPB3_Pin) == GPIO_PIN_RESET) {
            HAL_GPIO_TogglePin(PC13LED_GPIO_Port, PC13LED_Pin); //反转引脚电平
            
            // 等待按键释放
            while (HAL_GPIO_ReadPin(KEYPB3_GPIO_Port, KEYPB3_Pin) == GPIO_PIN_RESET) {
                HAL_Delay(10);
            }
            HAL_Delay(20);  // 释放消抖
        }
    }
    
        
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == KEYPB3_Pin)
  {
    key_pressed = 1;
  }
}
/* USER CODE END 4 */

```

由此我终于明白了我一开始问题的所在:**我在中断程序中使用了delay函数,而EXTI中断会将Systicks中断,使delay函数无法结束!程序停止在这里!**~~我顿时感觉我之前3h的努力像个小丑~~

至此bug被修复了,**而这也是为什么中断程序要简洁,只去做设置标志位,更新状态,复制数据这些简单的操作,因为在中断的情况下很多复杂的操作都会产生bug**

***

**一次红温的debug往往收获比一次顺利的编程更有收获**

***

# 1.3 呼吸灯

*一次成功,最顺利的一次*

***

## 基础知识

***

* TIM（Timer）定时器可以对输入的时钟进行计数，并在计数值达到设定值时触发中断
  主要构件为16位计数器、预分频器、自动重装寄存器的时基单元，在72MHz计数时钟下可以实现最大59.65s的定时(这是由寄存器所能存储的数据的最大值所决定的)
  其不仅具备基本的定时中断功能，而且还包含内外时钟源选择、输入捕获、输出比较、编码器接口、主从触发模式等多种功能
  根据复杂度和应用场景分为了高级定时器、通用定时器、基本定时器三种类型

  ![Snipaste_2025-11-06_20-07-05](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-06_20-07-05.png)

 	   STM32F103C8T6定时器资源：TIM1、TIM2、TIM3、TIM4

* 通用定时器组成结构:

  ![Snipaste_2025-11-06_19-37-14](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-06_19-37-14.png)

  ![Snipaste_2025-11-06_20-32-47](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-06_20-32-47.png)

通用定时器接在APB1总线上,接收系统主频,进入到PSC中进行分频,CNT计数器以该频率计数,计数达到自动重装载寄存器(ARR)后输出中断或者输出PWM波形等

*注:因为计算机内部是从0开始记录数据的所以配置数据是要减1,如:psc分频想要做到72倍分频则需配置71*

* PWM Generation channel Mode配置中mode 1,2分别代表cnt高于捕获/比较寄存器(ccr)时输出和cnt低于ccr时输出,输出的电平需额外配置

* PWM(脉宽调制):是一种控制空占比来模拟连续电压的方法,PWM频率=时钟频率/(ARR+1)*(PSC+1)

* 占空比:PWM波形中高电平在一个周期内的占比,占空比=CCR/(ARR+1)

  *注:CNT还分为递增递减两种模式*

***

## 工程建立

代码:

```c
/* USER CODE BEGIN PV */
	uint16_t pwm_val=0; //定义ccr的值
	int8_t cha=10; //定义每次ccr的改变量
/* USER CODE END PV */

/* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2); //启动计时器PWM模式
  /* USER CODE END 2 */

while (1)
  {
	  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,pwm_val);//配置相应的ccr的初值
	  pwm_val +=cha; //改变ccr的值
	  if(pwm_val>=1000){
		cha=-cha; //占空比为100%即灯最亮时,使每次ccr改变量为负值
	  }
	  if(pwm_val<=0){
		cha=-cha; //占空比为0%即灯最暗时,使每次ccr改变量为正值
	  }
		HAL_Delay(10); //每10ms多一点点改变一次ccr的值
	  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
```

实物图:![IMG_0949](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/IMG_0949.JPG)

在我的配置下TIM2接入72MHz的主频并分频到1MHz,**在while主循环中每隔10ms改变一次acc的值,也就改变了平均电压,而PWM波形一直在输出,从而实现了呼吸灯的效果**

***

# 1.4 舵机控制

*这个题有点水?*

***

## 基础知识

* 舵机:舵机可以在程序控制下,在一定范围内连续改变输出轴角度并且可以保持住

  接收50Hz的PWM波形,转动角度依据空占比-90°\~2.5%;90°~12.5%,线性分布

  PWM信号线:橙黄色

  正极:红色(舵机一般要求5v电压输入所以我将其连接到了ST-Link上)

  负极:棕色

* 模拟舵机,如我所使用的SG90需要持续的外部信号输入

***

## 工程建立

代码:

```c
/* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  /* USER CODE END 2 */

while (1)
  {
	  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,500);
	  HAL_Delay(1000);
	  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,1500);
	  HAL_Delay(1000);
	  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,2500);
	  HAL_Delay(1000);
    //delay是为了留给舵机转动的时间
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
```

实物图:

![IMG_0952](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/IMG_0952.JPG)

***

于是上电后舵机便在-90°,0°,90°之间不断旋转

~~*感觉跟上个大同小异*~~

***

# 1.5 高级按键识别

*这个还蛮有意思的,而且做出来很有成就感*

***

## 基础知识

* 状态机:即状态机编程,针对某个对象,分析其各个不同的状态并考虑不同状态的变换,以此为核心将代码封装在状态机函数(主体上是一个switch分支结构)中从而实现特定的功能,使代码模块化,易于维护和阅读
* 非阻塞编程:我们平时会让单片机同时进行多个任务,但是像delay函数等会让cpu卡在某个地方进行等待而无法执行其他任务浪费资源,这称为"阻塞",而非阻塞编程则是在避免这种情况出现的前提下进行的编程

***

## 工程建立

*STM32CubeMX的配置与1.4很像,这里不再提及,值得注意的是,我这里把按键接到了PB3上*

***

**我在这里先把我的代码贴出来,然后再分析我的思考流程和存在的问题**

```c
/* USER CODE BEGIN PV */
	uint16_t press=0;
	uint16_t press_mode=0;      //0:初始,1:单击,2:双击,3:长按
	uint32_t last_time = 0;
	uint32_t current_time = 0;
	uint32_t press_time = 0;
	uint32_t press_time_sec = 0;
	uint32_t release_time = 0;
	uint32_t waiting_time = 0;
	uint32_t end_time = 0;
	typedef enum{
	key_vacant=0,
	key_true_press,
	key_single,
	key_release,
	key_press_shake,
	key_press_shake_sec,
	key_release_shake,
	key_record_time,
	key_end,
	}key_state_t;
	volatile key_state_t key_state = key_vacant;
	typedef enum{
	servo_free=0,
	servo_busy,
	servo_define_time,
	}servo_state_t;
	volatile servo_state_t servo_state = servo_free;
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
	void key_sm(void);
	void servo_sm(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	current_time = HAL_GetTick();
  /* USER CODE END 2 */

while (1)
  {
	  key_sm();
	  servo_sm();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == KEY_Pin)
  {
    press = 1;
  }
}

void key_sm(void){
	current_time = HAL_GetTick();
	if (current_time - last_time < 5){
		return;
	}
    last_time = current_time;    //每5ms扫描一次
	
	switch(key_state){
	case (key_vacant):{
		if(press == 1){
			press_time = current_time;
			key_state=key_press_shake;
			press=0;
			}
		break;
		}                      //记录按键按下时间并跳转
	case (key_press_shake):{
		if(current_time - press_time >=20){
				if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin)== GPIO_PIN_RESET){
					key_state=key_true_press;
				}
				else{
					key_state=key_vacant;
				}
			}
		break;
	}                            //消抖并判断是否真正按下
	case (key_true_press):{
		if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin)== GPIO_PIN_SET){	
			release_time=current_time;
			key_state=key_release_shake;
		}
	break;
	}                           //记录按键松开时间并跳转
	case (key_release_shake):{
		if(current_time - release_time >=20){									
				if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin)== GPIO_PIN_SET){
					key_state=key_release;
				}
				else{
					key_state=key_true_press;
				}
			}
		break;
	}                         //消抖并判断是否真正松开
	case (key_release):{
		if(release_time - press_time >=700){
			press_mode=3; 
			key_state=key_record_time;
		}
		else{
			key_state=key_single;
		}
		break;    
		}                        //判断是否为长按
	case(key_single):{
		if(current_time - release_time <= 350){
			if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin)== GPIO_PIN_RESET){
			press_time_sec=current_time;
			key_state=key_press_shake_sec;
			} 
		}
		else {
			press_mode=1;
			key_state=key_record_time;
		}
		break;
		}                               //判断是否在双击时间内按下了第二次按键,否则为单击
	case (key_press_shake_sec):{
		if(current_time - press_time_sec >=20){
				if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin)== GPIO_PIN_RESET){
					press_mode=2;                
					key_state=key_record_time;
					}                               //消抖并判断第二次是否真的按下,若按下则为双击
				else{
					key_state=key_single;
				}
			}
		break;
	}
	case (key_record_time):{
		end_time=HAL_GetTick();
		key_state = key_end;
		break;
	}
	case (key_end):{
		current_time = HAL_GetTick();
		if(current_time - end_time >=500){
			key_state = key_vacant;
		}
		break;
	}                  //提供500ms的时间供所有设置复原
	}
}
void servo_sm(void){
	switch (servo_state){
		case (servo_free):{
			if(press_mode == 1){
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,500);
		press_mode=0;
		servo_state=servo_define_time;
		}	
		if(press_mode == 2){
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,1500);
			press_mode=0;
			servo_state=servo_define_time;
			}
		if(press_mode == 3){
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,2500);
			press_mode=0;
			servo_state=servo_define_time;
			}
	break;
		}                         //单击转到-90°,双击转到0°,长按转到90°
		case (servo_define_time):{
			waiting_time=current_time;
			servo_state=servo_busy;
			break;
		}
		case (servo_busy):{
			current_time = HAL_GetTick();
			if(current_time - waiting_time >=200){
				servo_state=servo_free;
			}
			break;                      //使舵机完成转向
		}
	}
}
/* USER CODE END 4 */
```

实物图:

![IMG_0958](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/IMG_0958.JPG)

**思路分析:**

* 首先状态机分为两个一个为按键状态机,一个为舵机状态机
* 判断按键类型的流程:一次按键的长短,长为长按,短为单击或双击,区分单机和双击可以检测在一定时间内有没有再次按下按键
* 非阻塞式消抖(按下消抖和释放消抖),**这一点我感觉我做的有点麻烦,我是采用了两个状态来进行消抖,第一个状态记录初次改变的时间,第二个状态通过gettick来判断延迟时间进行消抖,我这么做主要是考虑到状态机是不断运行的要是只用一个状态,初次改变时间会被gettick不断改写,当然也可以把第一个状态机的内容写入到上一个状态,但是我感觉这样不够清晰,虽然我这样做增加了一些复杂性,但是有没有不同于这二者更高的方法呢?**
* 每次按键状态机成功检测到按键变化后我留出了500ms的时间供所有状态复位(这样也可以使我不对双击的第二次击进行释放消抖)
* 舵机状态机的功能在1.4中有提及,构建思路和按键状态机相似,这里不再赘述

***

# 2.1 串口通信

*代码部分很简单,但是要学的基础知识比较多*

***

## 基础知识

* 串口通信(Serial Communication):一种设备间的串行通信方式,即一根数据线/通道传输数据

  ![Snipaste_2025-11-08_20-55-49](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-08_20-55-49.png)

  ***全双工意为可以同时双向通信,异步通信为二者时钟频率不一致,因此需要统一波特率,单端为绝对电平(即与GND的电平差)代表1与0,因此需要连接GND线,差分为数据线的电平差代表1与0)***

* USART:USART（Universal Synchronous/Asynchronous Receiver/Transmitter）通用同步/异步收发器
  USART是STM32内部集成的硬件外设，可根据数据寄存器的一个字节数据自动生成数据帧时序，从TX引脚发送出去，也可自动接收RX引脚的数据帧时序，拼接为一个字节数据，存放在数据寄存器里
  自带波特率发生器，最高达4.5Mbits/s
  可配置数据位长度（8/9）、停止位长度（0.5/1/1.5/2）
  可选校验位（无校验/奇校验/偶校验）
  支持同步模式、硬件流控制、DMA、智能卡、IrDA、LIN

  STM32F103C8T6 USART资源： USART1、 USART2、 USART3

  ![aaa](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/aaa.png)

  ***值得注意的是由于移位寄存器的工作原理(向右移位),其实数据是低位先行的,即数据位倒着传输***

  *ps:这里的移位寄存器出了移位外可以提供二次缓冲,提高效率*

* 通信电平标准:

  1. TTL标准: 逻辑1:2.4v~5v

     ​				逻辑0:0v~0.5v

  2. RS-232标准:逻辑1:-15v~-3v

     ​					 逻辑0:+3v~+15v
     
  3. RS485标准:逻辑1:两线压差+2v~+6v
  
     ​					逻辑0:两线压差-2v~-6v(差分信号)
  
* 串口通信的数据包(数据帧)由发送设备的TXD(TX)接口传输到接收设备的RXD(RX)接口,接收双方要规定数据包一致才能进行通信,数据包包含以下内容

![Snipaste_2025-11-08_20-31-58](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-08_20-31-58.png)

1. 空闲时为高电平,起始位为0(低电平),即可以检测到下降沿,数据位的个数可以自定义最大为9个(即占用掉校验位的位置)
2. 校验位的值与校验的类型有关,即无校验,奇校验,偶校验,无校验直接去掉校验位,奇/偶校验为校验位加上数据位的1的个数为奇数还是偶数
3. 停止位为1(高电平),进行回位

* 波特率:数据信号对载波的调制频率(其实就是对电平的切分规则,每多少时间的电平状态代表一位)

  对于二进制而言波特率和比特率相同

  USATR的发送器和接收器使用相同的波特率,公式为:波特率=USART时钟频率/(16*USARTDIV)

  其中USARTDIV是我们可以修改的一个量

* 起始位侦测,数据采样与噪声处理:

  ![bb](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/bb.png)

  每一位对于16个采样时钟(这也是波特率计算公式中16的来源)为了避免噪音的干扰,会在一些在点进行采样,只有采样点处有多数的0才会判断为起始位,数据采集与之同理,在每一位的中间进行采样

  若存在不正常的采样结果,但不影响最终的判断(如起始位侦测的时候采样出了少量的1)会给出NE标识符,代表数据收到了噪音影响

***

## 工程建立

***

我的STM32CubeMX配置:

![Snipaste_2025-11-08_17-42-20](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-08_17-42-20.png)

代码:

```c
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN 0 */
char message[]="Hello,world!\r\n";
/* USER CODE END 0 */

 while (1)
  {
	  HAL_UART_Transmit (&huart1,(uint8_t *)message,strlen(message),HAL_MAX_DELAY);
	  HAL_Delay(1000); //通信间隔为1000ms
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
```

代码部分很简单,利用transmit函数打印字符串即可,当然也可以利用printf的重定向

```c
/* USER CODE BEGIN 0 */
/**
 * @brief  fputc函数重定向 - 将标准输出重定向到串口
 * 
 * @param  ch : 要发送的字符（ASCII码）
 * @param  f  : 文件指针（标准输出流，通常不需要使用）
 * @return int : 返回发送的字符（成功时返回ch本身）
 * 
 * @note 这个函数会被printf、putchar等标准输出函数自动调用
 *       实现了将控制台输出重定向到STM32的串口1
 */
 int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
/* USER CODE END 0 */

while (1)
  {
	  printf("Hello,world!\r\n");
	  HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
```

**(推荐此类方法)**

实物图:

![IMG_0971](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/IMG_0971.JPG)

VOFA+配置及输出效果:

![Snipaste_2025-11-08_19-56-01](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-08_19-56-01.png)

***

# 2.2 定时发送

*~~有点水?~~*

***

## 基础知识

* 与1.3中学习到的通用定时器等类似,ARM Cortex-M存在一个系统定时器Sys Tick,其被配置为了每1ms产生一次中断,其也有自己的回调函数可以被我们改写

* ```c
  /**
    * @brief This function is called to increment  a global variable "uwTick"
    *        used as application time base.
    * @note In the default implementation, this variable is incremented each 1ms
    *       in SysTick ISR.
    * @note This function is declared as __weak to be overwritten in case of other
    *      implementations in user file.
    * @retval None
    */
  __weak void HAL_IncTick(void)
  {
    uwTick += uwTickFreq;
  }
  ```

  我们还知道HAL库中存在这样一个函数,其中帮我们给出了一个全局变量uwTick即为系统运行时间

***

## 工程建立

***

代码:

main.c:

```c
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* USER CODE BEGIN 0 */
/**
 * @brief  fputc函数重定向 - 将标准输出重定向到串口
 * 
 * @param  ch : 要发送的字符（ASCII码）
 * @param  f  : 文件指针（标准输出流，通常不需要使用）
 * @return int : 返回发送的字符（成功时返回ch本身）
 * 
 * @note 这个函数会被printf、putchar等标准输出函数自动调用
 *       实现了将控制台输出重定向到STM32的串口1
 */
 int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
/* USER CODE END 0 */
```

主要是为了重定向printf函数

stm32f1xx_it.c:

```c
/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	static int count =0;
	count++;
	if(count >= 100){
		count = 0;
		printf("系统运行了:%d ms\r\n",uwTick+1);
	}
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}
```

这里对uwTick+1主要是因为HAL_IncTick()函数在我写的代码的后方,所以使输出时的uwTick比实际的系统运行时间满了1ms,当然交换一下代码的顺序就不用+1了

效果图:

![Snipaste_2025-11-08_23-22-15](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-08_23-22-15.png)

***

# 2.3 指令控制

*~~又要debug了(悲),这次的bug发生在了一个我意想不到的位置,我甚至很奇怪我竟然能找到这个bug~~*

***

## 基础知识

***

* 串口接收模式主要分为:阻塞接收,非阻塞接收(轮询),中断接收,DMA接收,这里我主要介绍阻塞接收和中断接收,轮询即为不断的重复接收,这里不做介绍,MDA接收之后会学习到

* 阻塞接收:

  ```c
  /**
    * @brief  Receives an amount of data in blocking mode.
    * @note   When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
    *         the received data is handled as a set of u16. In this case, Size must indicate the number
    *         of u16 available through pData.
    * @param  huart Pointer to a UART_HandleTypeDef structure that contains
    *               the configuration information for the specified UART module.
    * @param  pData Pointer to data buffer (u8 or u16 data elements).
    * @param  Size  Amount of data elements (u8 or u16) to be received.
    * @param  Timeout Timeout duration
    * @retval HAL status
    */
  HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
  ```

  主要是由这个函数实现,但是我们总是不希望阻塞的,所以更好的是中断接收

* 中断接收:当串口接收到预定数量的数据后会产生一个中断,可以在回调函数中处理数据

  ```c
  /**
    * @brief  Receives an amount of data in non blocking mode.
    * @note   When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
    *         the received data is handled as a set of u16. In this case, Size must indicate the number
    *         of u16 available through pData.
    * @param  huart Pointer to a UART_HandleTypeDef structure that contains
    *               the configuration information for the specified UART module.
    * @param  pData Pointer to data buffer (u8 or u16 data elements).
    * @param  Size  Amount of data elements (u8 or u16) to be received.
    * @retval HAL status
    */
  HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)  //启动接收中断,设置缓存区和长度
  
  
  HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//回调函数
  ```

* 当然发送也有阻塞和非阻塞式的与接收类似,这里不做介绍

***

## 工程建立

代码:

```c
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
uint8_t Rxbuff[2]={0};  //定义缓存区数组,因为最长的指令为"OFF"所以数组长度为3即可
uint8_t count=0;  //定义计数
/* USER CODE END PV */

/* USER CODE BEGIN 0 */
/**
 * @brief  fputc函数重定向 - 将标准输出重定向到串口
 * 
 * @param  ch : 要发送的字符（ASCII码）
 * @param  f  : 文件指针（标准输出流，通常不需要使用）
 * @return int : 返回发送的字符（成功时返回ch本身）
 * 
 * @note 这个函数会被printf、putchar等标准输出函数自动调用
 *       实现了将控制台输出重定向到STM32的串口1
 */
 int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
/* USER CODE END 0 */

/* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1,Rxbuff,1); //启动接收中断
  /* USER CODE END 2 */

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart ->Instance == USART1 ){              //判断中断是否来自我们所要的串口USART1
    count++;									//计数,因为我们一次只读一位因此要通过计数来确定下一位放在哪
    if(count == 2 && strncmp((char *)Rxbuff,"ON",2) == 0){
      printf("\r\nLED ON!\r\n");
      HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
      count = 0;
      memset(Rxbuff,0,sizeof(Rxbuff));             //要记得将缓存区和计数恢复到初始状态
    }
    else if(count == 3 && strncmp((char *)Rxbuff,"OFF",3) == 0){
      printf("\r\nLED OFF!\r\n");
      HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
      count = 0;
      memset(Rxbuff,0,sizeof(Rxbuff));
    }
    else if(count == 3){
      count = 0;
      memset(Rxbuff,0,sizeof(Rxbuff));      //若输入了未知指令则也要清除
    }
    HAL_UART_Receive_IT(&huart1,&Rxbuff[count],1);  //这一个很关键每次receive到的字节要放到下一个位置防止覆盖
  }
}
/* USER CODE END 4 */

```

这次的代码编写思路还是很巧妙的,每一个字节就触发一次中断,并进行if判断,这样就可以避免指令长度不一样的问题,但是有很多小细节需要注意,我会以注释的形式写在代码中

**而且还需要注意,与定时器产生的中断不同,串口产生的中比如说接收中断,是需要在产生中断后重新使能的,这是由HAL库的设计所决定的**

***

**~~你是不是在想"说好的debug呢,我是来看你受苦的,你是不是标题党啊?!"别急,bug来了~~**

***

这是我的初次VOFA+设置:

![Snipaste_2025-11-09_18-04-55](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-09_18-04-55.png)

可以看到第一次的指令读取成功,但是后续指令有的成功有的不成功,于是我就去检查我的代码却怎样也发现不了问题,于是我又猜测是不是我发送的指令有问题呢,毕竟它确实有时候能接收到一些指令,比如我这里可能出现了像覆盖那样的问题

**后来我在看VOFA的串口收发内容的时候,一个本来让我感到欣慰的东西引起了我的注意,我的代码里并没有换行操作(上述的代码并不是我的初次代码,初次中确实没有换行操作)但是VOFA却自动给我换了行,我一开始以为这只是VOFA这么显示为了方便我们去阅读,现在来看它极有可能自己偷偷加上了换行符!**

然后我就看见了右下角的\n图标,把它切换为无追加后再发送指令(并在代码中添加了换行),结果就正确了:

![Snipaste_2025-11-09_17-30-25](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-09_17-30-25.png)

***

**看来有时候bug并不一定来自于你的代码中**

***

# 2.4 空闲中断

*~~一遍过,嘻嘻~~*

***

## 基础知识

***

* 串口接收不定长数据主要是通过UART空闲中断实现的,即检测到空闲或者设定的长度的字节的时候产生中断

* 主要涉及以下两个函数:

  ```c
  /**
    * @brief Receive an amount of data in interrupt mode till either the expected number of data is received or an IDLE event occurs.
    * @note   Reception is initiated by this function call. Further progress of reception is achieved thanks
    *         to UART interrupts raised by RXNE and IDLE events. Callback is called at end of reception indicating
    *         number of received data elements.
    * @note   When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M = 01),
    *         the received data is handled as a set of uint16_t. In this case, Size must indicate the number
    *         of uint16_t available through pData.
    * @param huart UART handle.
    * @param pData Pointer to data buffer (uint8_t or uint16_t data elements).
    * @param Size  Amount of data elements (uint8_t or uint16_t) to be received.
    * @retval HAL status
    */
  HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
  
  void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)   //注意这里的size为有效的字节长度,函数已经帮我们计算好了,如果后续要用到直接引用就好
  ```

  其用法与定长中断的用法一致,这里不再介绍

  一些注意事项我以注释的形式写在了上述代码中

***

## 工程建立

***

代码:

```c
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
uint8_t Rxbuff[10]={0};
/* USER CODE END PV */

/* USER CODE BEGIN 0 */
/**
 * @brief  fputc函数重定向 - 将标准输出重定向到串口
 * 
 * @param  ch : 要发送的字符（ASCII码）
 * @param  f  : 文件指针（标准输出流，通常不需要使用）
 * @return int : 返回发送的字符（成功时返回ch本身）
 * 
 * @note 这个函数会被printf、putchar等标准输出函数自动调用
 *       实现了将控制台输出重定向到STM32的串口1
 */
 int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
/* USER CODE END 0 */

/* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_IT (&huart1,Rxbuff,10);
  /* USER CODE END 2 */

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if(huart ->Instance == USART1 ){              //判断中断是否来自我们所要的串口USART1
    if(strncmp((char *)Rxbuff,"ON",2) == 0){
      printf("\r\nLED ON!\r\n");
      HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
    }
    else if(strncmp((char *)Rxbuff,"OFF",3) == 0){
      printf("\r\nLED OFF!\r\n");
      HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
    }
    memset(Rxbuff,0,sizeof(Rxbuff));
    HAL_UARTEx_ReceiveToIdle_IT (&huart1,Rxbuff,10);
  }
}
/* USER CODE END 4 */

```

**我们可以看出,代码明显精简了很多,而且避免了count计数的复杂逻辑,体现出空闲中断的优越性!**

***

VOFA+效果:

![Snipaste_2025-11-09_19-54-58](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-09_19-54-58.png)

根据上述的函数原理我们也可以知道,即使在命令的末尾加上\n换行符也不会出现像2.3中的bug

***

*~~我的天啊,空闲中断大人!~~*

***

# 2.5DMA优化

*尽管比较简单,但是功能强大*

***

## 基础知识

***

* DMA:直接存储器存取(Direct Memory Access)
  DMA可以提供外设和存储器或者存储器和存储器之间的高速数据传输，无须CPU干预，节省了CPU的资源
  12个独立可配置的通道： DMA1（7个通道）， DMA2（5个通道）
  每个通道都支持软件触发和特定的硬件触发
  STM32F103C8T6 DMA资源：DMA1（7个通道）
  
* 在一.STM32概述中的系统结构图中我们可以很清晰地看到DMA与cpu和其他外设的关系

* DMA结构简图:

  ![Snipaste_2025-11-09_22-06-30](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-09_22-06-30.png)

* DMA每个通道都分为软件触发和硬件触发

  软件触发:由软件指令启动,调用函数等

  硬件触发:由外设事件自动启动

  ![ddd](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/ddd.png)

* M2M位为控制内存之间搬运数据还是内存与外设之间搬运数据,NE为为开关

* DMA请求也有优先级,可以配置,优先级相同时顺序由上图决定,在DMA仲裁器中完成

* 传输计数器为递减计数,直到为0,自动重装器作用是在传输计数器为0时将其回位到初始值

  它的启用与否决定了是否循环

* 我们还可以配置受否地址自增,如外设的地址固定一般不自增保证始终从正确的外设地址获取数据,内存地址一般自增,防止覆盖先前记录的数据

* 数据传输过程中,当源端宽度和目标宽度不一致时,采用高位舍弃和高位补零的方法进行传输

  传输宽度分类:

  字节(Byte):每次8位

  半字(Half Word):每次16位

  字(Word):每次32位

* 主要涉及以下两个函数:

  ```c
  /**
    * @brief Receive an amount of data in DMA mode till either the expected number of data is received or an IDLE event occurs.
    * @note   Reception is initiated by this function call. Further progress of reception is achieved thanks
    *         to DMA services, transferring automatically received data elements in user reception buffer and
    *         calling registered callbacks at half/end of reception. UART IDLE events are also used to consider
    *         reception phase as ended. In all cases, callback execution will indicate number of received data elements.
    * @note   When the UART parity is enabled (PCE = 1), the received data contain
    *         the parity bit (MSB position).
    * @note   When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M = 01),
    *         the received data is handled as a set of uint16_t. In this case, Size must indicate the number
    *         of uint16_t available through pData.
    * @param huart UART handle.
    * @param pData Pointer to data buffer (uint8_t or uint16_t data elements).
    * @param Size  Amount of data elements (uint8_t or uint16_t) to be received.
    * @retval HAL status
    */
  HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)   //启用了DMA接收并支持空闲中断
  
  
  /**
    * @brief  Sends an amount of data in DMA mode.
    * @note   When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
    *         the sent data is handled as a set of u16. In this case, Size must indicate the number
    *         of u16 provided through pData.
    * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
    *                the configuration information for the specified UART module.
    * @param  pData Pointer to data buffer (u8 or u16 data elements).
    * @param  Size  Amount of data elements (u8 or u16) to be sent
    * @retval HAL status
    */
  HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)
  ```

  二者的用法与之前的函数用法相同,这里不再介绍

***

## 工程建立

***

STM32CubeMX配置:

![Snipaste_2025-11-09_21-28-23](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-09_21-28-23.png)

代码:

```c
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
uint8_t Rxbuff[10]={0};
/* USER CODE END PV */

  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_DMA (&huart1,Rxbuff,10);
  /* USER CODE END 2 */

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if(huart ->Instance == USART1 ){              //判断中断是否来自我们所要的串口USART1
    if(strncmp((char *)Rxbuff,"ON",2) == 0){
      HAL_UART_Transmit_DMA (&huart1,(uint8_t *)"LED ON!",strlen("LED ON!"));  //重定向的printf函数是阻塞且由cpu控制的这里换成HAL_UART_Transmit_DMA,非阻塞且由DMA控制
      HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
    }
    else if(strncmp((char *)Rxbuff,"OFF",3) == 0){
      HAL_UART_Transmit_DMA (&huart1,(uint8_t *)"LED OFF!",strlen("LED OFF!"));
      HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
    }
    memset(Rxbuff,0,sizeof(Rxbuff));
    HAL_UARTEx_ReceiveToIdle_DMA (&huart1,Rxbuff,10);
  }
}
/* USER CODE END 4 */
```

代码与之前的相似,不过多介绍,小细节以注释的形式在代码中给出

VOFA+效果:

![Snipaste_2025-11-09_21-45-50](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-09_21-45-50.png)

***

# 2.6 Wi-Fi通信

***

**我不知道经历了多少次bug了**

***

## 基础知识

***

* ESP8266是一款高集成度,低成本的Wi-Fi模块,内置32位处理器,可独立运行简单程序,支持AT命令与自定义固件开发(固件是写入到储存器中的程序,如在单片机中就是flash中的程序)

* AT命令是终端设备与PC或MCU之间的通信标准化命令,ESP8266可以自行解读AT命令并进行相应的操作

  **值得注意的是指令结尾通常要回车换行:\r\n**

* AT指令默认波特率为115200,使用双引号表示字符串数据"string"

* ESP8266的常用引脚说明:

  RST:复位,低电平有效,默认上拉

  EN:芯片使能,高电平有效,默认上拉

  VCC:3.3v供电

  GND:接地

  TX/RX:串口引脚

  IO0:默认上拉为运行模式,外部拉低为下载模式

  IO2:多功能引脚,默认上拉保证正常启动

* ESP8266总共有3种工作模式STA,AP,STA+AP

  STA模式下,ESP8266相当于客户端,接入别人的网络

  AP模式下ESP8266相当于服务端,自己创建网络供别人接入

  STA+AP模式即为可以同时实现两种模式

* TCP传输协议:是一种面向连接的(先建立连接在传输数据)提供可靠交付服务的(没有发送成功数据就一直发送)全双工通信的(这个之前在串口提到过)基于字节流的(并不以数据包的形式发送,而是顺序发送,可能随意中断或拼接),端到端的传输协议

  (具体很复杂,这里只简略介绍)ESP8266需要先与PC处在同一网络环境,然后建立TCP连接到服务器

* 与TCP相应的还有UDP传输协议,这里不详细介绍

* 透传模式:即透明传输,只关心数据的来源和去向地址只把ESP8266当作数据传输的通道,不对数据进行任何修改和解读

  值得注意的是,透传模式因为不会对数据进行解读,所以也不会对AT指令做出反应

  而退出透传模式需要发送"+++"指令**注:不要加上\r\n回车换行符,这跟一般的AT命令有区别**

***

## 工程建立

***

~~*保持好心态,要开始debug喽,*~~

~~*保持不了一点心态,红温了不知道多少次了*~~

***

因为代码部分的内容比较多,这里先给出实物图:

![IMG_0998](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/IMG_0998.JPG)

第一次代码:

```c
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
uint8_t AT_command_buffer [128]={0};
uint8_t AT_response_buffer [128]={0};    //建立AT命令的缓冲区
/* USER CODE END PV */

  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_DMA (&huart1,AT_command_buffer,128);
  HAL_UARTEx_ReceiveToIdle_DMA (&huart2,AT_response_buffer,128);
  /* USER CODE END 2 */

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
  if(huart ->Instance ==  USART1){
    HAL_UART_Transmit_DMA(&huart2,AT_command_buffer,Size);
    memset(AT_command_buffer,0,sizeof(AT_command_buffer));
    HAL_UARTEx_ReceiveToIdle_DMA (&huart1,AT_command_buffer,128);
  }
  else if(huart ->Instance ==  USART2){
    HAL_UART_Transmit_DMA(&huart1,AT_response_buffer,Size);
    memset(AT_response_buffer,0,sizeof(AT_response_buffer));
    HAL_UARTEx_ReceiveToIdle_DMA (&huart2,AT_response_buffer,128);
  }
}
/* USER CODE END 4 */
```

但是当我输入AT命令时,总是只能接收到A

经过了一段相当痛苦的查资料,问AI,调试之后终于发现了问题所在

**DMA的数据搬运工作是自己在后台完成的,并不会影响CPU代码的执行(这个我其实前面学到了,但是真的没有往这个方向去想),而DMA的搬运需要时间,所以DMA刚开始搬运的时候我就用memset清空了缓存区,所以就出现了bug!**

**知道了bug的来源之后我想到了一个办法不仅可以避免bug的出现还实现了非阻塞,DMA搬运,代码的效果,那就是在接收的回调函数中删去重置缓存区和重新启用DMA空闲中断的代码,转而去发送的中断回调函数中去清除对方的缓存区并启用对方的空闲中断,这样不仅解决了bug,还是非阻塞形式,而且还避免了命令输入过多过快的情况,可谓是我能想到的非常完美的方法了**

~~*然而思路是对的,但是吧,一落实到代码上就寄了*~~

第二次代码:

```c
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */


/* USER CODE BEGIN PV */
uint8_t AT_command_buffer [128]={0};
uint8_t AT_response_buffer [128]={0};    //建立AT命令的缓冲区
/* USER CODE END PV */

  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_DMA (&huart1,AT_command_buffer,128);
  HAL_UARTEx_ReceiveToIdle_DMA (&huart2,AT_response_buffer,128);
  /* USER CODE END 2 */

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
  if(huart ->Instance ==  USART1){
    HAL_UART_Transmit_DMA(&huart2,AT_command_buffer,Size);
  }
  else if(huart ->Instance ==  USART2){
    HAL_UART_Transmit_DMA(&huart1,AT_response_buffer,Size);
  }
}


void HAL_UARTEx_TxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
  if(huart ->Instance == USART2){
    memset(AT_command_buffer,0,sizeof(AT_command_buffer));
    HAL_UARTEx_ReceiveToIdle_DMA (&huart1,AT_command_buffer,128);
  }
  else if(huart ->Instance == USART1){
    memset(AT_response_buffer,0,sizeof(AT_response_buffer));
    HAL_UARTEx_ReceiveToIdle_DMA (&huart2,AT_response_buffer,128);
  }
}
/* USER CODE END 4 */
```

当我信心满满地测试的时候,又寄了,ESP8266只能响应我的第一次命令

**后来经过排查发现是我的发送中断回调函数用错了(这个错误真的很不应该,确实是用少了,一不小心就调用错了,我还一直没怀疑,自罚三巴掌!)**

```c
HAL_UART_TxCpltCallback ()//这个才是与HAL_UART_Transmit_DMA()对应的回调函数
```

现在我们终于有了正确的代码可以与ESP8266建立连接了

```c
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
uint8_t AT_command_buffer [128]={0};
uint8_t AT_response_buffer [128]={0};    //建立AT命令的缓冲区
/* USER CODE END PV */

  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_DMA (&huart1,AT_command_buffer,128);
  HAL_UARTEx_ReceiveToIdle_DMA (&huart2,AT_response_buffer,128);
  /* USER CODE END 2 */

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
  if(huart ->Instance ==  USART1){
    HAL_UART_Transmit_DMA(&huart2,AT_command_buffer,Size);
  }
  else if(huart ->Instance ==  USART2){
    HAL_UART_Transmit_DMA(&huart1,AT_response_buffer,Size);
  }
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  if(huart ->Instance == USART2){
    memset(AT_command_buffer,0,sizeof(AT_command_buffer));
    HAL_UARTEx_ReceiveToIdle_DMA (&huart1,AT_command_buffer,128);
  }
  else if(huart ->Instance == USART1){
    memset(AT_response_buffer,0,sizeof(AT_response_buffer));
    HAL_UARTEx_ReceiveToIdle_DMA (&huart2,AT_response_buffer,128);
  }
}
/* USER CODE END 4 */

```

VOFA+上位机与ESP8266的串口通信内容:

![Snipaste_2025-11-10_20-24-21](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-10_20-24-21.png)

另外需要注意一点,当使用"AT+RST"命令时,ESP8266会输出一个波特率为74880的启动信息,所以会产生一个error中断,所以还要配置一下错误中断回调函数,将帧错误抛除

```c
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
  if(huart ->Instance == USART2){
    __HAL_UART_CLEAR_FEFLAG(huart);   //清除帧错误标志
    HAL_UARTEx_ReceiveToIdle_DMA (&huart2,AT_response_buffer,128);
  }
```



现在,我们可以开始对ESP8266进行配置了!

***

~~**本以为逃离了火坑,其实又跳进了大海**~~

***

**我的思路是ESP8266使用STA模式,采用透传**

首先第一步便是使PC和ESP8266位于同一网络环境下

这里我打算利用手机热点建立一个局域网

但是当我使用AT命令试图连接到手机热点时,出现了error

![Snipaste_2025-11-10_21-59-49](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-10_21-59-49.png)

报错代码3的意思是没有找到该网络

我通过AT指令搜寻网络也确实没有找到我对热点

但是我的热点确实启动了

在搜集各方资料后我发现了问题所在:

**手机的热点是5G频段而ESP8266只支持2.4G频段**

怎么解决呢?别担心,手机厂家为我们提供了方法:

![IMG_0999(20251111-001520)](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/IMG_0999(20251111-001520).jpg)

打开最大兼容性按钮即可

现在ESP8266成功连上了手机热点

***

下一步是让我的PC连上手机热点

看起来很简单对吧,但是我又遇到了问题

**我的电脑连不上我的手机热点,而且重启无效**

后来搜集资料,我对我对网络配置进行了重置

**现在终于连上了手机热点,与ESP8266处在相同的网络环境之下**

***

下一步是建立TCP服务端

其实我一开始试图用手机建立,然后用手机与ESP8266通信,但由于我的手机是苹果的,没有合适的软件所以经尝试后只能放弃~~***苹果不行***~~

然后我转而去在电脑上建立TCP服务器,然而在相继使用过,包括但不限于Apifox,Termius等等4,5个软件均无法建立后,**我只得自己搭建TCP服务器**

所以我又去下了Python和PythonPharm(Pryhon IDE)

在AI的指导下搭建了自己的TCP服务器

以下是服务器代码:

```python
import socket
import threading
import time


class TCPServer:
    def __init__(self, host='0.0.0.0', port=8080):
        self.host = host
        self.port = port
        self.clients = []
        self.running = True

    def start(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.host, self.port))
        self.socket.listen(5)
        print(f"🚀 TCP服务器启动在 {self.host}:{self.port}")
        print("等待ESP8266连接...")

        # 启动接受连接的线程
        accept_thread = threading.Thread(target=self.accept_connections)
        accept_thread.daemon = True
        accept_thread.start()

        # 主线程处理用户输入
        self.command_interface()

    def accept_connections(self):
        while self.running:
            try:
                client_socket, addr = self.socket.accept()
                print(f"✅ 新连接: {addr}")
                self.clients.append((client_socket, addr))

                # 为新客户端创建线程
                thread = threading.Thread(target=self.handle_client, args=(client_socket, addr))
                thread.daemon = True
                thread.start()

            except Exception as e:
                if self.running:
                    print(f"接受连接错误: {e}")

    def handle_client(self, client_socket, addr):
        try:
            # 发送欢迎消息
            welcome_msg = "Welcome to TCP Server! Commands: LED=ON, LED=OFF, STATUS\r\n"
            client_socket.send(welcome_msg.encode())

            while True:
                data = client_socket.recv(1024).decode('utf-8').strip()
                if not data:
                    break

                print(f"📨 来自 {addr}: {data}")

                # 处理命令
                response = self.process_command(data)
                client_socket.send(f"{response}\r\n".encode())
                print(f"📤 发送响应: {response}")

        except Exception as e:
            print(f"❌ 客户端 {addr} 错误: {e}")
        finally:
            client_socket.close()
            self.remove_client(client_socket)
            print(f"🔌 客户端 {addr} 断开连接")

    def remove_client(self, client_socket):
        self.clients = [client for client in self.clients if client[0] != client_socket]

    def process_command(self, command):
        command = command.upper()
        if "LED=ON" in command:
            return "LED_ON_OK"
        elif "LED=OFF" in command:
            return "LED_OFF_OK"
        elif "STATUS" in command:
            return f"SERVER_STATUS: Clients={len(self.clients)}, Time={time.strftime('%Y-%m-%d %H:%M:%S')}"
        else:
            return f"ECHO: {command}"

    def send_to_client(self, client_index, message):
        """向特定客户端发送消息"""
        if 0 <= client_index < len(self.clients):
            client_socket, addr = self.clients[client_index]
            try:
                client_socket.send(f"{message}\r\n".encode())
                print(f"📤 发送给 {addr}: {message}")
                return True
            except:
                print(f"❌ 发送失败，客户端可能已断开")
                return False
        else:
            print(f"❌ 客户端索引 {client_index} 无效")
            return False

    def list_clients(self):
        """显示所有连接的客户端"""
        print(f"📊 连接中的客户端: {len(self.clients)}")
        for i, (_, addr) in enumerate(self.clients):
            print(f"  {i}. {addr}")

    def command_interface(self):
        """命令行界面，用于手动发送命令给ESP8266"""
        print("\n💡 服务器命令界面:")
        print("  'list' - 显示所有连接的客户端")
        print("  'send <index> <message>' - 向特定客户端发送消息")
        print("  'broadcast <message>' - 向所有客户端广播消息")
        print("  'quit' - 关闭服务器")

        while self.running:
            try:
                cmd = input("\n>>> ").strip()

                if cmd.lower() == 'quit':
                    self.stop()
                    break
                elif cmd.lower() == 'list':
                    self.list_clients()
                elif cmd.startswith('send '):
                    parts = cmd.split(' ', 2)
                    if len(parts) == 3:
                        client_index = int(parts[1])
                        message = parts[2]
                        self.send_to_client(client_index, message)
                    else:
                        print("❌ 用法: send <客户端索引> <消息>")
                elif cmd.startswith('broadcast '):
                    message = cmd[10:]
                    for i in range(len(self.clients)):
                        self.send_to_client(i, message)
                else:
                    print("❓ 未知命令")

            except KeyboardInterrupt:
                self.stop()
                break
            except Exception as e:
                print(f"命令错误: {e}")

    def stop(self):
        print("\n🛑 正在关闭服务器...")
        self.running = False
        for client_socket, addr in self.clients:
            try:
                client_socket.close()
            except:
                pass
        self.socket.close()
        print("✅ 服务器已关闭")


if __name__ == "__main__":
    server = TCPServer()
    try:
        server.start()
    except KeyboardInterrupt:
        server.stop()
```

**现在我终于可以让我对ESP8266连接到我的TCP服务器了**

***

那么接下来就是要进一步编写代码,使其能分析命令并控制LED(PC13)的亮灭

以下是代码:

```c
/* USER CODE BEGIN PV */
uint8_t AT_command_buffer [128]={0};
uint8_t AT_response_buffer [128]={0};    //建立AT命令的缓冲区
uint8_t cmd[128]={0};            //建立命令缓存区
uint8_t cmd_ready=1; //1代表没有命令正在执行,0代表有命令正在执行
/* USER CODE END PV */

  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_DMA (&huart1,AT_command_buffer,128);
  HAL_UARTEx_ReceiveToIdle_DMA (&huart2,AT_response_buffer,128);
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if(cmd_ready == 0){
      if(strstr((const char *)cmd,"LED_ON") != NULL){
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      }
      if(strstr((const char *)cmd,"LED_OFF") != NULL){
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      }
      memset(cmd,0,sizeof(cmd));
      cmd_ready=1;
    }
  }
  /* USER CODE END 3 */


/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
  if(huart ->Instance ==  USART1){
    HAL_UART_Transmit_DMA(&huart2,AT_command_buffer,Size);
  }
  else if(huart ->Instance ==  USART2){
    if(cmd_ready){
      memcpy(cmd,AT_response_buffer,Size);
      cmd[Size] = '\0';    // 末尾添加字符串终止符
      cmd_ready=0;
    }
    HAL_UART_Transmit_DMA(&huart1, AT_response_buffer, Size);
  }
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  if(huart ->Instance == USART2){
    memset(AT_command_buffer,0,sizeof(AT_command_buffer));
    HAL_UARTEx_ReceiveToIdle_DMA (&huart1,AT_command_buffer,128);
  }
  else if(huart ->Instance == USART1){
	memset(AT_command_buffer,0,sizeof(AT_command_buffer));
    HAL_UARTEx_ReceiveToIdle_DMA (&huart2,AT_response_buffer,128);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
  if(huart ->Instance == USART2){
    __HAL_UART_CLEAR_FEFLAG(huart);   //清除帧错误标志
    HAL_UARTEx_ReceiveToIdle_DMA (&huart2,AT_response_buffer,128);
  }
}
```

这里有几处改动要注意:

1. 在命令缓存区接收到的命令末尾我加上了'\0'终止符,这是因为string系列的命令大多要以其作为终止,否则容易发生越界现象
2. 我去掉了memset命令,这主要是由于接收和发送缓存区如果既要复制又要发送又要清空很容易造成混乱,所以干脆不清空,也不会影响下一次的使用***(比如我有在过不合适的位置添加过memset,导致上位机上看到的命令只有一部分)***

***

**你是不是以为这就完了?当时的我也是这么以为的,但是当我通过vsc上的keil assistant把程序下载到STM32上时并进行检测时,发现没有任何现象**

***

**于是我问AI,查资料,改代码,反复折腾了好久,可是不管怎样,都是没现象**

就在我快要爆炸的时候,我脑海中突然想起了一件事,并有了一个大胆的想法------**我有一次在改代码的时候不小心把strstr输成了strsrt,但是vsc的编译并没有帮我检测出来**

**那么有没有一种可能,vsc崩了?!**

于是我在keil 5中进行了编译和下载,并进行了检测

**程序完美运行!**

![Snipaste_2025-11-11_13-13-22](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-11_13-13-22.png)

![Snipaste_2025-11-11_13-13-30](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-11_13-13-30.png)

**后来经过我的排查,发先vsc没崩,keil assistant插件崩了,我在卸载重装后,终于一切正常!**

***

不想评价,无语了,反反复复折腾了好几个小时,半夜2点半睡不着在想这个,这些bug真是一个比一个离谱,我的代码反而是出问题最少的部分

***

**AP模式**

这个其实比TCP简单很多,主要是对ESP8266的配置,而且经历过上述bug洗礼的我已经对bug无感了

***

* 首先是对ESP8266的配置:

  1. AT+CWMODE=2 设置为AP模式(这一步没有也可以)

  2. AT+CWSAP="ESP_AP","12345678",6,3 配置AP参数,分别为(SSID)接入点名称(Wi-Fi名称),密码,信道(ESP8266使用2.4GHz频段,大频段内又分为几个使用不同频段的信道),加密方式(3代表WPA2_PSK加密,常用)

  3. AT+CIPSERVER=1,8080 启动TCP服务器,端口号8080

  4. AT+CIFSR 查看IP地址

* 电脑连接ESP8266的Wi-Fi:

  ![Snipaste_2025-12-03_23-56-37](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-12-03_23-56-37.png)

* 配置TCP Client,建立连接,发送指令***==我真傻,真的,做完2.6好久在玩VOFA+的时候才发现它自己提供了TCP的Server和Client服务,傻了直接==***

  ![Snipaste_2025-12-03_23-56-25](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-12-03_23-56-25.png)

* **其实这里也有一个小bug,一开始我连接不到ESP8266的TCP Server,后来发现是我的梯子的问题,关了梯子就好了**

# 3.1多任务整合

本来以为要学好久,结果发现一点点皮毛就够实现目标了

***

## 基础知识

***

* FreeRTOS是免费开源的实时操作系统内核,来实现多任务处理功能

* 多任务处理:受硬件的限制,一个逻辑处理器(一般情况下也就是一个内核)只能处理一个任务,而通过在短时间内快速切换任务可以达到多任务同时处理的效果,同时可以提高CPU的利用率

* 调度器:操作系统中负责切换任务的组件,决定在哪个时刻执行哪个任务,一个任务可以多少次暂停和恢复

* 调度策略:

  1. 时间片轮询:为每个任务分配时间片,依次循环执行

  2. 优先级调度:为每个任务分配优先级,优先级高的任务队列优先进行时间片轮询

     在无高优先级队列或高优先级队列主动释放的情况下低优先级的队列才进行时间片轮询

* 任务的状态:
  1. Running(运行中):任务真正被执行时所处的状态,通常极短
  2. Ready(就绪):一个任务正在队列中等待被执行,现在正在执行的可能是高优先级队列也可能是同队列的其他任务
  3. Blocked(阻塞):任务正在等待某个事件,例如正在等待延迟,任务就会自动进入阻塞状态,不参与调度
  4. Suspended(挂起):用户可以通过vTaskSuspend()和xTaskResume()API令任务进入挂起状态或者从挂起状态中恢复,被挂起的任务不参与调度

* 任务状态的转换关系图:

  ![Snipaste_2025-11-11_21-01-23](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-11_21-01-23.png)

* 就绪态中的任务实际上以二维数组的形式存储,分别代表优先级和队列里的位置

***

* FreeRTOS使用的几个中断:
  1. Systick中断:即系统时钟中断,维护系统时钟,每次中断FreeRTOS就会判断是否需要切换任务,如果有则会进行任务的上下文切换(Context Switch)(通过触发PendSV中断)
  2. PendSV中断:用于执行实际的上下文切换操作
  3. SVC中断:将第一个任务的上下文加载到CPU

​	***注* : **

* ***上下文切换 :把当前任务的CPU状态保存下来(包括通用寄存器,程序计数器^1^,堆栈指针^2^等),再将新任务的 CPU状态恢复回去***

* ***由于FreeRTOS使用了Systick中断,所以最好将时间基源从SysTick换为TIM***

1. ***程序计数器 :用来指向当前CPU正在执行哪一条代码***
2. ***堆栈指针 :指向当前堆栈地址,即接下来的内存数据要放到哪***

***

* 基础代码的介绍:

  ```c
  vTaskStartScheduler()   //启动调度器,此代码之后的所有代码均无效,因为已经进入到了FreeRTOS的任务运行方式
  
  xTaskCreate(	TaskFunction_t pxTaskCode,
  							const char * const pcName,
  							const configSTACK_DEPTH_TYPE usStackDepth,
  							void * const pvParameters,
  							UBaseType_t uxPriority,
  							TaskHandle_t * const pxCreatedTask )  //任务创建函数
      //第1个参数:任务的函数指针(指向用户自己写的函数)
      //第2个参数:任务的名称
      //第3个参数:任务所占的内存大小(注意单位是字,位*4=字节)
      //第4个参数:初始传入任务的指针(因为这只是创建任务,所以一般是NULL)
      //第5个参数:任务的优先级
      //第6个参数:任务句柄,为指向任务结构体指针的指针(用来区分不同的任务)
      
  ```

  ***注1:任务优先级可以自行更改,也可以查看系统默认的配置,而且数字越大优先级越高***

  ```c
  #define configMAX_PRIORITIES                     ( 56 )
  //这是我从FreeRTOSConfig.h中copy过来的,我这里系统默认最大优先级为56
  ```

  ***注2:vTaskStartScheduler()函数会自己建立一个空闲任务,防止没有任务的情况出现,而它的优先级是0***

  *或许这也就解释了为什么优先级数字越大优先级越高这一反常识的规律,因为最大优先级用户可以自定义,而空闲任务的优先级一定得是最低的*

  ***注3:xTaskCreate()函数是动态分配内存地址的,也就是说系统会自动帮你找到存放数据的地方(当然这个地方要多大需要你自己定义),而xTaskCreateStatic()则是静态分配内存地址,你还需要指定一个内存地址存放数据***

***

* FreeRTOS的程序编写规范:
  1. 首先创建FreeRTOS的启动文件与头文件,启动文件中包含了用户编写的函数(当然这些也可以分别写到不同的其他文件中),FreeRTOS的任务管理,以及启动调度器等,这些应该写在一个函数内例如:FreeRTOS_Start
  2. 主函数中只调用FreeRTOS_Start函数
  3. 记得在keil 5中把自己创建的文件加入到项目中去

**具体实例可以参考下方的工程建立中的代码**

***

## 工程建立

***

main.c

```c
/* USER CODE BEGIN Includes */
#include <FreeRTOS_F.h>
/* USER CODE END Includes */

  /* USER CODE BEGIN 2 */
  FreeRTOS_Start();  //启动FreeRTOS
  /* USER CODE END 2 */
```

FreeRTOS_F.c(自己建立的文件)

```c
#include <FreeRTOS_F.h>

TaskFunction_t task_breathing_led_ptr = task_breathing_led;  //呼吸灯任务参数
TaskFunction_t task_serial_communications_ptr = task_serial_communications;//上位机指令任务参数

#define task_breathing_led_stack_size 128  //呼吸灯任务栈大小 128*4=512字节
#define task_breathing_led_priority 4      //呼吸灯任务优先级
TaskHandle_t task_breathing_led_handle_ptr;  //呼吸灯任务句柄

uint8_t Rxbuff[32]={0};  //命令缓冲区
uint8_t command =0; //0代表可以接收命令,1代表不能接收命令
#define task_serial_communications_stack_size 128
#define task_serial_communications_priority 4
TaskHandle_t task_serial_communications_handle_ptr;

void FreeRTOS_Start(void){





    xTaskCreate(task_breathing_led_ptr,"task_breathing_led",task_breathing_led_stack_size,NULL,task_breathing_led_priority,&task_breathing_led_handle_ptr);  //创建任务
    xTaskCreate(task_serial_communications_ptr,"task_serial_communications",task_serial_communications_stack_size,NULL,task_serial_communications_priority,&task_serial_communications_handle_ptr);

    vTaskStartScheduler();  //启动调度器
}

void task_breathing_led(void *pointer){
    uint16_t pwm_val=0; //定义ccr的值
	int8_t cha=10; //定义每次ccr的改变量

    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2); //启动计时器PWM模式

    while (1)
  {
	  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,pwm_val);//配置相应的ccr的初值
	  pwm_val +=cha; //改变ccr的值
	  if(pwm_val>=1000){
		cha=-cha; //占空比为100%即灯最亮时,使每次ccr改变量为负值
	  }
	  if(pwm_val<=0){
		cha=-cha; //占空比为0%即灯最暗时,使每次ccr改变量为正值
	  }
		HAL_Delay(10); //每10ms多一点点改变一次ccr的值
  }
}

void task_serial_communications(void *pointer){

	HAL_UARTEx_ReceiveToIdle_DMA (&huart1,Rxbuff,10);

	while(1){
		if(command){
			if(strncmp((char *)Rxbuff,"ON",2) == 0){
      	HAL_UART_Transmit_DMA (&huart1,(uint8_t *)"LED ON!",strlen("LED ON!"));
      	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
		}
		else if(strncmp((char *)Rxbuff,"OFF",3) == 0){
      	HAL_UART_Transmit_DMA (&huart1,(uint8_t *)"LED OFF!",strlen("LED OFF!"));
      	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
    	}
		HAL_UARTEx_ReceiveToIdle_DMA (&huart1,Rxbuff,10);
		command=0;
		}
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	command=1;
}

```

FreeRTOS_F.h(自己建立的文件)

```c
#include <FreeRTOS.h>
#include <task.h>
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>

void FreeRTOS_Start(void);
void task_breathing_led(void *pointer);
void task_serial_communications(void *pointer);

```

通过FreeRTOS的多任务处理,呼吸灯和响应上位机的任务快速切换执行,从而同时实现了呼吸灯和上位机控制PC13LED的功能

***

**FreeRTOS真是博大精深,这里只做了最初步的了解**

**==注意:我这里不知道STM32CubeMX为我们生成好了FreeRTOS文件,因此我们可以不用自己创建文件,规范的做法请见4.1!!!!!另外我会额外新建一章讲讲FreeRTOS该怎么规范地使用==**

***

# 3.2 FreeRTOS补充

~~*主要是我太抽象了,那STM32CubeMX配置完FreeRTO后忘了它会生成给我用来放代码的freertos.c文件,更可气的是我都看见它好几次了也没点进去看看(我还以为是初始化啥的呢)*~~

***

在freertos.c文件中STM32CubeMX已经为我们划分好了相应的放不同代码的区域,按照注释来即可,这里着重介绍一下它为我们封装好的API**(其实需要的参数还是那么几个,原理都是一样的但是代码写出来的形式不一样)**

```c
//这里以4.1中我写的一个task为例
osThreadId_t TASK_MPU_HANDLE; //定义任务句柄

  TASK_MPU_HANDLE = osThreadNew(TASK_MPU_F,NULL,&(osThreadAttr_t){
    .name = "TASK_MPU",
    .stack_size = TASK_MPU_STACK_SIZE,
    .priority = TASK_MPU_PRIORITY,

  });
//首先三个参数分别为函数指针,初始输入的指针(这两个同上)
//第三个参数为结构体指针,这种写法在参数处直接建立了一个临时结构体,定义了任务名称,堆栈大小,优先级
//osThreadNew,会帮我们建立好结构体并调用xTaskCreate,来创建任务,并返回句柄
```

***

# 4.1 传感器驱动

**这次的收获真的很大,不仅仅是学习了MPU6050,更对FreerRTOS的理解更深了,并懂得了代码规范的重要性,以及必要性**

***

## 基础知识

***

~~*真的好多,真的!(黄豆哭泣.jpg)*~~

***

### 一.I2C协议

* I2C（Inter IC Bus）是由Philips公司开发的一种通用数据总线
  有两根通信线：SCL（Serial Clock）、SDA（Serial Data）
  同步，半双工
  支持数据应答
  支持总线挂载多设备（一主多从、多主多从）

  *注:多主多从的情况下若有多个设备想要获取总线的控制权则I2C协议会进行仲裁,获胜的一方会获得主线控制权*

  同步时序,可以支持中断后继续进行数据交换

* 所有I2C设备的SCL连在一起，SDA连在一起
  设备的SCL和SDA均要配置成开漏输出模式
  SCL和SDA各添加一个上拉电阻，阻值一般为4.7KΩ左右

* 硬件电路:

  ![a1](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/a1.png)

所有I2C设备的SCL连在一起，SDA连在一起
设备的SCL和SDA均要配置成开漏输出模式
SCL和SDA各添加一个上拉电阻，阻值一般为4.7KΩ左右(防止开漏造成的悬空电平不稳定)

*注1:这么做同时也是为了安全,如果没有上拉电阻,对于SDA线,如果没有协调好主机和从机的输入输出时机,比如两个设备都是输出而正好输出一个低电平一个输出高电平那么就会出现短路,这是应该完全避免的,**所以通过上拉电阻实现弱上拉,所有的设备开漏输出,只能输出低电平,那么如果设备像输出高电平就什么都不输出,想输出低电平就正常输出低电平)***

*注2:这样总线存在"线与"现象即有一个设备输出低电平总线就为低电平,所有设备都输出高电平,总线才为高电平*

SCL只能由主机控制,用来统一时钟频率,从机只能接收

SDA大部分时间由主机控制,只有在主机释放的情况下从机才会获得SDA的控制权(从机发送数据,从机应答)

* I2C时序基本单元

  1. 起始条件：SCL高电平期间，SDA从高电平切换到低电平

     终止条件：SCL高电平期间，SDA从低电平切换到高电平

     ![Snipaste_2025-11-13_23-22-10](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-13_23-22-10.png)

​			*注:主机在发送起始/终止条件后/前,会下拉/不下拉SCL来方便后续的操作*

​				*一个完整的数据帧总是以起始条件开始,终止条件结束,且只允许主机发送*

  2. 发送一个字节

     SCL低电平期间，主机将数据位依次放到SDA线上（高位先行）(0的话拉低SDA,1的话不拉低SDA)，然后释放SCL，从机将在SCL高电平期间读取数据位，所以SCL高电平期间SDA不允许有数据变化，依次循环上述过程8次，即可发送一个字节

​		![Snipaste_2025-11-13_23-30-12](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-13_23-30-12.png)

*注1:由于SCL由主机控制,所以从机在检测到SCL高电平时要尽快读取SDA上的电位,一般在上升沿时从机就已经读取完成*

*注2:如果中断数据的发送,SDA和SCL都会暂停电平变化,这样保证了中断后可以恢复数据传输(不会在正在发送数据/读取数据的时候暂停)*

3. 接收一个字节

   SCL低电平期间，从机将数据位依次放到SDA线上（高位先行），然后释放SCL，主机将在SCL高电平期间读取数据位，所以SCL高电平期间SDA不允许有数据变化，依次循环上述过程8次，即可接收一个字节（主机在接收之前，需要释放SDA）(与发送字节类似,只是主从机的身份互换)

   *注:由于SCL由主机控制,所以从机在检测到SCL低电平的时候要尽快改变SDA上的电位,一般在下降沿刚结束的时候就已经改变完成*

4. 发送应答：主机在接收完一个字节之后，在下一个时钟发送一位数据，数据0表示应答，数据1表示非应答,从机在发送完一个字节之后要通过主机的应答判断主机是否还需要下一个字节的发送

   接收应答：主机在发送完一个字节之后，在下一个时钟接收一位数据，判断从机是否应答，数据0表示应答，数据1表示非应答（主机在接收之前，需要释放SDA）主机在发送完一个字节之后要通过从机的应答判断从机是否收到了数据

   ***注:由于在主机释放SDA的一瞬间从机就下拉了SDA来进行应答所以波形应该一直为低电平***

   ![Snipaste_2025-11-14_00-20-22](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-14_00-20-22.png)

* 主机在发送起始条件后要发送一个字节来呼叫要建立连接的设备,从机将这个字节与自己的地址进行比对

  I2C协议中设备地址分为7位和10位,这里只介绍7位地址,设备的地址由厂家在生产时决定,同一个型号的设备也可以通过自身的配置来改变地址从而达到一个总线上挂在多个相同型号的设备(一般地址高位由厂家决定,低位可以由引脚切换)

  ***注:发送的这个字节前7位为设备地址,最后一位代表读或者写(0为写入,1为读取)***

* 指定地址写

  ![bb2](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/bb2.png)

  这里第一个字节即为呼叫设备,第二个字节即以后的字节可以由从机自己定义用途

  MPU6050定义第二个字节为寄存器地址,第三个字节为写入的内容

* 当前地址读

  ![bb6](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/bb6.png)

  即读取当前地址指针下的地址的内容

  第二个字节为主机读取的从机发出的字节

  *注:当前地址指针:上电后初始化指向0地址,寄存器的地址是线性排布的,当前地址指针在每写入一个字节和读取一个字节之后都会自增一次*

* 指定地址读

  ![c1](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/c1.png)

  把指定地址写的指定地址部分和当前地址读的读的部分拼接在一起就是指定地址读

  ***因为要切换读写所以要在中间再加入一个起始条件(Sr)***

***注:由于当前地址指针的自增,所以把最后的时序重复几遍即可实现连续写入多个字节和连续读取多个字节***

* I2C有三种传输模式:标准模式100kb/s,快速模式400kb/s,高速模式3.4mb/s,不过目前大多I2C设备不支持高速模式

* I2C结构图

  ![Snipaste_2025-11-12_17-14-48](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-12_17-14-48.png)

  *注:其中PEC计算是一种判断数据在传输过程中有没有出差的方法*

* I2C常用HAL库API:

  ```c
  HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
  HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
      
      
      //写入/读取寄存器(参数由上到下分别是)
      //I2C外设句柄(在STM32中用来指定是I2C1还是I2C2)
      //设备地址
      //寄存器地址
      //寄存器地址大小
      //数据缓冲区指针
      //要写入/读取的字节数
      //超时时间
  //使用DMA
  HAL_I2C_Mem_Write_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
  HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
  
      
  //还有直接传输数据流与寄存器地址无关的API(当然非DMA的版本也有)
  HAL_I2C_Master_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
  HAL_I2C_Master_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
      //传输/接收数据流(参数分别是)
      //I2C外设句柄(在STM32中用来指定是I2C1还是I2C2)
      //设备地址
      //数据缓冲区指针
      //要写入/读取的字节数
  ```
  
  **值得注意的是当写入/读取的字节数大于1时会继续写入/读取之后的寄存器的数据**

***

### 二.MPU6050

***

* MPU6050模块可以同时检测三轴加速度,三轴陀螺仪(角速度),以及温度,并且内置了滤波,融合等处理,通过I2C输出数据

* 特殊引脚介绍:

  INT引脚:中断输出引脚,如数据就绪中断和FIFO(即MPU6050的内部缓存)溢出中断(当然要使用中断需要提前使能中断),0x37寄存器配置INT引脚,0x38寄存器配置中断

  FSYNC引脚:接收外部同步信号,这个信号来临时,MPU6050会同步采样所有数据(在0x1A寄存器配置)

  AD0引脚:默认下拉,低电平MPU60507位地址为0x68,高电平MPU60507位地址为0x69

* 常用寄存器介绍:

  1. 0x19寄存器:陀螺仪采样率分频寄存器,采样频率=陀螺仪输出频率/(1+SMPLRT_DIV)

     其中陀螺仪输出频率与数字低通滤波器(DLPF)的配置有关,当DLPF_CFG的配置为0或7时,陀螺仪输出频率为8kHz,否则为1kHz

  2. 0x1B寄存器:陀螺仪配置寄存器,触发陀螺仪自检和配置量程
  3. 0x1C寄存器:加速度计配置寄存器,触发加速度计自检和配置量程
  4. 0x3B~0x40寄存器:加速度计测量寄存器,储存加速度计的最近测量值,每两个寄存器储存加速度值的高八位和低八位,组合起来为16为的加速度数据
  5. 0x43~48寄存器:陀螺仪测量寄存器,储存陀螺仪的最近测量值,储存方法同加速度计测量寄存器
  6. 0x1A寄存器:配置寄存器,配置DLPF,配置FSYNC引脚,主要作用是配置带宽和噪声滤波

* 重要概念介绍:

  1. 采样率(Sampling Rate):传感器每秒采集的数据次数
  2. 输出率(Output Rate):每秒通过I2C输出的数据个数,(DLPF启用时输出率均为1kHz,DLPF禁用时加速度输出率1kHz,陀螺仪输出率8kHz)
  3. 带宽(Bandwidth):传感器能准确测量的最高信号频率,比如当带宽为44时,传感器能精确测量0~44Hz的信号而频率大于44Hz的信号会被衰减或滤除(测量不准确)

  因此我们不难得出采样率≥输出率

  有奈奎斯特定理知采样率≥2*带宽,否则会出现混叠失真

  ***注:混叠失真,可以类比于对一个有一个圆孔的转动圆盘进行一定频率的拍照,在一定的拍照频率下只看照片你可能会误以为圆盘在倒转,混叠失真与这个同理***

  *~~奈奎斯特定理高中的时候还会证来着,可是我现在是大一,我已经忘干净了~~*

* 给出几个DLPF配置的不同情况:

  1. DLPF_CFG=0(无滤波)

     采样率和输出率都为加速度1kHz,陀螺仪8kHz,带宽260Hz(高速响应但是噪声大)

  2. DLPF_CFG=3(中等滤波)

     所有的采样率和输出率都为1kHz,带宽44Hz(性能平衡)

  3. DLPF_CFG=6(强滤波)

     所有的采样率和输出率都为1kHz,带宽5Hz(高稳定性但是响应慢)

  **总的来说,带宽越大能测量更快的动作但是噪声也会变大;输出率越大数据越实时,但是处理负担会变大;采样率越大,精度越大,但是受限于硬件**

* 16位ADC采集传感器的模拟信号，量化范围：-32768~32767
  加速度计满量程选择：±2、±4、±8、±16（g）
  陀螺仪满量程选择： ±250、±500、±1000、±2000（°/sec）

* MPU6050框图:

  ![hhh](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/hhh.png)

* MPU6050的初始化:MPU6050上电后默认休眠,而且各个参数也没有初始化,需要外部进行初始化

***

## 工程建立

***

代码很多,先给出实物图:

![IMG_1030](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/IMG_1030.JPG)

代码(我自己建立了MPU6050.c/h,data.c/h):

data.h

```c
#include <stdint.h>
#ifndef DATA_H
#define DATA_H

#define MPU6050_ADDR    (0x68 << 1)  // 若AD0接地，7位地址0x68，左移1位得到8位地址0xD0
#define WHO_AM_I_REG    0x75        // WHO_AM_I寄存器地址，默认值0x68
#define PWR_MGMT_1_REG  0x6B        // 电源管理寄存器1
#define SMPLRT_DIV_REG  0x19        // 采样率分频寄存器
#define CONFIG_REG      0x1A        // 配置寄存器（含DLPF设置）
#define GYRO_CONFIG_REG 0x1B        // 陀螺仪配置寄存器
#define ACCEL_CONFIG_REG 0x1C       // 加速度计配置寄存器


extern int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
extern int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;
extern uint8_t DMA_ready;

#endif

```

**值得注意的是,设备地址不必纠结于末位的读写位,在你调用对应的API时其会帮你自动转化**

data.c

```c
#include <data.h>

int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;
uint8_t DMA_ready = 1;

```

MPU6050.h

```c
#include <data.h>
#include "main.h"
#include "i2c.h"


HAL_StatusTypeDef MPU6050_Init(void);
void MPU6050_Read(void);

```

MPU6050.c

```c
#include <MPU6050.h>

HAL_StatusTypeDef MPU6050_Init(void) {
    uint8_t check, data;
    HAL_StatusTypeDef res;
    // 1. 读取 WHO_AM_I 寄存器，检查设备ID是否正确 (0x68)
    res = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 100);
    if(res != HAL_OK || check != 0x68) {
        return HAL_ERROR;  // 通信失败或ID不符
    }
    // 2. 解除休眠，将 PWR_MGMT_1 寄存器写0
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 100);
    HAL_Delay(10);  // 小延迟，等待芯片唤醒稳定
    // 3. 设置采样率分频器 SMPLRT_DIV (比如设置成7获得1kHz采样率)
    data = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 100);
    // 4. 配置DLPF，在CONFIG寄存器中设置数字低通滤波器 (例如0x03,Accel带宽44Hz)
    data = 0x03;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, CONFIG_REG, 1, &data, 1, 100);
    // 5. 配置陀螺仪满量程范围 ±250°/s (0x00) 和加速度计满量程范围 ±2g (0x00)
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 100);
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 100);
    return HAL_OK;
}

void MPU6050_Read(void) {
    uint8_t buf[6];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, buf, 6, 100);
    // 拼接高低字节为 16 位有符号值
    Accel_X_RAW = (int16_t)(buf[0] << 8 | buf[1]);
    Accel_Y_RAW = (int16_t)(buf[2] << 8 | buf[3]);
    Accel_Z_RAW = (int16_t)(buf[4] << 8 | buf[5]);
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43, 1, buf, 6, 100);
    Gyro_X_RAW = (int16_t)(buf[0] << 8 | buf[1]);
    Gyro_Y_RAW = (int16_t)(buf[2] << 8 | buf[3]);
    Gyro_Z_RAW = (int16_t)(buf[4] << 8 | buf[5]);
}

```

freertos.c

```c
/* USER CODE BEGIN Includes */
#include <MPU6050.h>
#include <data.h>
#include <string.h>
#include <usart.h>
#include <stdio.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */
void TASK_MPU_F(void *ptr);
void TASK_TRANS_F(void *ptr);
/* USER CODE END PTD */

/* USER CODE BEGIN Variables */
osThreadId_t TASK_MPU_HANDLE,TASK_TRANS_HANDLE;
#define TASK_MPU_STACK_SIZE 256
#define TASK_MPU_PRIORITY osPriorityNormal
#define TASK_TRANS_STACK_SIZE 512
#define TASK_TRANS_PRIORITY osPriorityNormal
/* USER CODE END Variables */

  /* USER CODE BEGIN Init */
  MPU6050_Init();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_THREADS */
  TASK_MPU_HANDLE = osThreadNew(TASK_MPU_F,NULL,&(osThreadAttr_t){
    .name = "TASK_MPU",
    .stack_size = TASK_MPU_STACK_SIZE,
    .priority = TASK_MPU_PRIORITY,

  });

  TASK_TRANS_HANDLE = osThreadNew(TASK_TRANS_F,NULL,&(osThreadAttr_t){
    .name = "TASK_TRANS",
    .stack_size = TASK_TRANS_STACK_SIZE,
    .priority = TASK_TRANS_PRIORITY,
  });

  /* USER CODE END RTOS_THREADS */

/* USER CODE BEGIN Application */
void TASK_MPU_F(void *ptr){
  while (1)
  {
    MPU6050_Read();
    osDelay(1000);
  }
}

void TASK_TRANS_F(void *ptr){
  while(1){
    if(DMA_ready){
    DMA_ready = 0;
    char buff[256];
    int length = sprintf(buff,"\r\nAccel_x:%d\r\nAccel_y:%d\r\nAccel_z:%d\r\nGyro_x:%d\r\nGyro_y:%d\r\nGyro_z:%d\r\n",Accel_X_RAW,Accel_Y_RAW,Accel_Z_RAW,Gyro_X_RAW,Gyro_Y_RAW,Gyro_Z_RAW);
    HAL_UART_Transmit_DMA(&huart1,(const uint8_t *)buff,length);
    }
    osDelay(1000);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  if(huart ->Instance == USART1){
    DMA_ready = 1;
  }
}
/* USER CODE END Application */

```

效果图:

![Snipaste_2025-11-13_22-30-21](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-13_22-30-21.png)

***

**4.1给我的收获真的蛮多的,一开始我的代码思路和架构与最终的没什么区别,但是当时我的代码规范性特别差,建立了各个文件,一眼看过去非常的头疼,而且也无法正常运行,==但是当我以一种规范化的形式去写之后,还是同样的思路和基本架构,代码立刻就生效了==而且非常方便我去看去找我每一步写的是什么干的什么,可读性非常好**

***~~我说这代码规范真王朝了吧~~***

**另外我一开始输出的内容其实是中文,但是不知道为什么六个"度"里面只有两个"度"编码错误,显示乱码,非常的奇怪,我也找不出问题,后来全换成英文就好了,==所以以后向上位机等发送字符数据的时候最好使用英文==**

***

# 4.2 数据滤波/4.4姿态解算

***

*~~我真的不会算法啊,大哭()~~*

~~**这debug更是要了我的老命了,不过也确实加经验了**~~

***

## 基础知识

***

* 陀螺仪测量的数据存在零偏和误差需要后续的校准

* 数据由于抖动等原因会上下跳跃,这时通过滤波可以消除这种抖动

* 卡尔曼滤波(其中的数学知识太深奥,看不懂,这里给出一个形象化的例子便于理解):

  > 假设我们要研究的对象是一个房间的温度。根据你的经验判断，这个房间的温度是恒定的，也就是下一分钟的温度等于现在这一分钟的温度（假设我们用一分钟来做时间单位）。假设你对你的经验不是100%的相信，可能会有上下偏差几度。我们把这些偏差看成是高斯白噪声（White Gaussian Noise），也就是这些偏差跟前后时间是没有关系的而且符合高斯分布（Gaussian Distribution）。另外，我们在房间里放一个温度计，但是这个温度计也不准确的，测量值会比实际值偏差。我们也把这些偏差看成是高斯白噪声。
  >
  > 好了，现在对于某一分钟我们有两个有关于该房间的温度值：你根据经验的预测值（系统的预测值）和温度计的值（测量值）。下面我们要用这两个值结合他们各自的噪声来估算出房间的实际温度值。
  >
  > 假如我们要估算k时刻的实际温度值。首先你要根据k-1时刻的温度值，来预测k时刻的温度。因为你相信温度是恒定的，所以你会得到k时刻的温度预测值是跟k-1时刻一样的，假设是23度，同时该值的高斯噪声的偏差是5度（5是这样得到的：如果k-1时刻估算出的最优温度值的偏差是3，你对自己预测的[不确定度](https://baike.baidu.com/item/不确定度/0?fromModule=lemma_inlink)是4度，他们平方相加再开方，就是5）。然后，你从温度计那里得到了k时刻的温度值，假设是25度，同时该值的偏差是4度。
  >
  > 由于我们用于估算k时刻的实际温度有两个温度值，分别是23度和25度。究竟实际温度是多少呢？相信自己还是相信温度计呢？究竟相信谁多一点，我们可以用他们的协方差(covariance)来判断。因为Kg=5^2/(5^2+4^2)，所以Kg=0.61，我们可以估算出k时刻的实际温度值是：23+0.61*(25-23)=24.22度。可以看出，因为温度计的协方差(covariance)比较小（比较相信温度计），所以估算出的最优温度值偏向温度计的值。
  >
  > 现在我们已经得到k时刻的最优温度值了，下一步就是要进入k+1时刻，进行新的最优估算。到现在为止，好像还没看到什么自回归的东西出现。对了，在进入k+1时刻之前，我们还要算出k时刻那个最优值（24.22度）的偏差。算法如下：((1-Kg)*5^2)^0.5=3.12。这里的5就是上面的k时刻你预测的那个23度温度值的偏差，得出的3.12就是进入k+1时刻以后k时刻估算出的最优温度值的偏差（对应于上面的3）。
  >
  > 就是这样，卡尔曼滤波器就不断的把协方差(covariance)递归，从而估算出最优的温度值。他运行的很快，而且它只保留了上一时刻的协方差(covariance)。上面的Kg，就是卡尔曼增益（Kalman Gain）。他可以随不同的时刻而改变他自己的值，是不是很神奇！

* 滤波后响应太慢可以适当减小r值

  若仍有太多噪声增大r值

  需要更快追踪变换增大q值

* 四元数法求姿态角:数学知识好复杂,有点超出能力范围(也没什么时间仔细研究了,以后有时间说不定会补上),这里我专注于现象的实现
* justfloat协议数据包:无包头,包尾为0x00 00 80 7f,数据包为uint_8类型,每个浮点数占4字节

***

## 工程建立

***

*我这里发现单纯的卡尔曼滤波对陀螺仪数据的滤波效果没有特别好,而且也为了消除零偏所以加上了平均滤波(主要是为了应对一些极端的噪音)*

***

代码(这里仅给出重要的代码):

kalman_filter.c

```c
#include "kalman_filter.h"
#include <string.h>
#include <stdint.h>
#include <math.h>

// 卡尔曼滤波器结构体
typedef struct {
    float Q;
    float R;
    float x;
    float P;
    float K;
    
    // 滤波参数
    float last_measurement;
    float last_5_measurements[5];
    uint8_t history_index;
    uint32_t outlier_count;
    uint32_t consecutive_outliers;
    float adaptive_R_multiplier;
    uint32_t stable_count;
} SuperKalmanFilter;

// 静态变量
static SuperKalmanFilter kf_ax, kf_ay, kf_az;
static SuperKalmanFilter kf_gx, kf_gy, kf_gz;
static uint8_t initialized = 0;

// 初始化
static void Kalman_Init(SuperKalmanFilter *kf, float Q, float R, float initial_value) {
    kf->Q = Q;
    kf->R = R;
    kf->x = initial_value;
    kf->P = 1.0f;
    kf->K = 0.0f;
    kf->last_measurement = initial_value;
    kf->history_index = 0;
    kf->outlier_count = 0;
    kf->consecutive_outliers = 0;
    kf->adaptive_R_multiplier = 1.0f;
    kf->stable_count = 0;
    
    for (int i = 0; i < 5; i++) {
        kf->last_5_measurements[i] = initial_value;
    }
}

// 计算标准差
static float Calculate_StdDev(SuperKalmanFilter *kf) {
    float mean = 0.0f;
    for (int i = 0; i < 5; i++) {
        mean += kf->last_5_measurements[i];
    }
    mean /= 5.0f;
    
    float variance = 0.0f;
    for (int i = 0; i < 5; i++) {
        float diff = kf->last_5_measurements[i] - mean;
        variance += diff * diff;
    }
    variance /= 5.0f;
    
    return sqrtf(variance);
}

// 计算中位数
static float Calculate_Median(SuperKalmanFilter *kf) {
    float temp[5];
    memcpy(temp, kf->last_5_measurements, sizeof(temp));
    
    for (int i = 0; i < 4; i++) {
        for (int j = i + 1; j < 5; j++) {
            if (temp[i] > temp[j]) {
                float swap = temp[i];
                temp[i] = temp[j];
                temp[j] = swap;
            }
        }
    }
    return temp[2];
}

// 卡尔曼滤波更新
static float Kalman_Update(SuperKalmanFilter *kf, float measurement, int is_gyro) {
    // 更新历史数据
    kf->last_5_measurements[kf->history_index] = measurement;
    kf->history_index = (kf->history_index + 1) % 5;
    
    float diff = fabsf(measurement - kf->last_measurement);
    
    // 针对陀螺仪的强力滤波
    if (is_gyro) {
        float std_dev = Calculate_StdDev(kf);
        float median = Calculate_Median(kf);
        float mean_diff_from_median = fabsf(measurement - median);
        
        // 多重噪声检测阈值
        float outlier_threshold = 8.0f;     // 降低到8°/s
        float extreme_threshold = 20.0f;    // 极端噪声20°/s
        float statistical_threshold = std_dev * 5.0f;
        
        // 多重条件检测
        int is_outlier = 0;
        int is_extreme = 0;
        
        if (diff > extreme_threshold) {
            is_extreme = 1;
            is_outlier = 1;
        } else if (diff > outlier_threshold || mean_diff_from_median > statistical_threshold) {
            is_outlier = 1;
        }
        
        // 连续异常检测
        if (is_outlier) {
            kf->consecutive_outliers++;
            kf->stable_count = 0;
        } else {
            kf->consecutive_outliers = 0;
            kf->stable_count++;
        }
        
        // 超强噪声处理
        if (is_extreme) {
            // 极端噪声：完全忽略，使用中位数，只预测不更新
            kf->adaptive_R_multiplier = 50000.0f;
            kf->outlier_count++;
            kf->P = kf->P + kf->Q;
            kf->last_measurement = median;
            return kf->x;
        }
        else if (is_outlier || kf->consecutive_outliers > 0) {
            // 噪声：大幅增加噪声协方差，使用中位数平滑
            kf->adaptive_R_multiplier = 2000.0f;
            kf->outlier_count++;
            measurement = 0.8f * median + 0.2f * measurement;
        }
        else {
            // 正常数据
            if (kf->stable_count > 20) {
                kf->adaptive_R_multiplier = kf->adaptive_R_multiplier * 0.7f + 0.3f;
            }
            if (kf->adaptive_R_multiplier < 1.0f) kf->adaptive_R_multiplier = 1.0f;
        }
    } else {
        // 加速度计温和滤波
        float accel_threshold = 0.2f;
        if (diff > accel_threshold) {
            kf->adaptive_R_multiplier = 5.0f;
        } else {
            kf->adaptive_R_multiplier = kf->adaptive_R_multiplier * 0.9f + 0.1f;
            if (kf->adaptive_R_multiplier < 1.0f) kf->adaptive_R_multiplier = 1.0f;
        }
    }
    
    kf->last_measurement = measurement;
    
    // 限制范围
    if (kf->adaptive_R_multiplier > 50000.0f) kf->adaptive_R_multiplier = 50000.0f;
    if (kf->adaptive_R_multiplier < 1.0f) kf->adaptive_R_multiplier = 1.0f;
    
    // 卡尔曼滤波核心
    kf->P = kf->P + kf->Q;
    float current_R = kf->R * kf->adaptive_R_multiplier;
    kf->K = kf->P / (kf->P + current_R);
    kf->x = kf->x + kf->K * (measurement - kf->x);
    kf->P = (1 - kf->K) * kf->P;
    
    return kf->x;
}

// 三重移动平均滤波
static float Triple_MovingAverage(float new_value, int sensor_id) {
    static float history[6][15] = {{0}};
    static uint8_t indices[6] = {0};
    static uint8_t fill_count[6] = {0};
    
    history[sensor_id][indices[sensor_id]] = new_value;
    indices[sensor_id] = (indices[sensor_id] + 1) % 15;
    if (fill_count[sensor_id] < 15) fill_count[sensor_id]++;
    
    // 三重平均：短期、中期、长期
    float short_sum = 0.0f, mid_sum = 0.0f, long_sum = 0.0f;
    int short_count = 0, mid_count = 0, long_count = 0;
    
    for (int i = 0; i < fill_count[sensor_id]; i++) {
        int idx = (indices[sensor_id] - 1 - i + 15) % 15;
        float val = history[sensor_id][idx];
        
        if (i < 5) {
            short_sum += val;
            short_count++;
        }
        if (i < 10) {
            mid_sum += val;
            mid_count++;
        }
        long_sum += val;
        long_count++;
    }
    
    float short_avg = short_sum / short_count;
    float mid_avg = mid_sum / mid_count;
    float long_avg = long_sum / long_count;
    
    // 加权组合
    return 0.6f * short_avg + 0.3f * mid_avg + 0.1f * long_avg;
}

// 初始化函数
void KalmanFilter_Init(void) {
    if (initialized) return;
    
    // 加速度计参数
    Kalman_Init(&kf_ax, 0.01f, 0.1f, 0.0f);
    Kalman_Init(&kf_ay, 0.01f, 0.1f, 0.0f);
    Kalman_Init(&kf_az, 0.01f, 0.1f, 1.0f);
    
    // 陀螺仪超强参数
    Kalman_Init(&kf_gx, 0.0001f, 0.005f, 0.0f);  // 极低的Q和R
    Kalman_Init(&kf_gy, 0.0001f, 0.005f, 0.0f);
    Kalman_Init(&kf_gz, 0.0001f, 0.005f, 0.0f);
    
    initialized = 1;
}

// 更新函数
void KalmanFilter_Update(float ax, float ay, float az, 
                        float gx, float gy, float gz,
                        float *ax_filt, float *ay_filt, float *az_filt,
                        float *gx_filt, float *gy_filt, float *gz_filt) {
    
    if (!initialized) {
        KalmanFilter_Init();
    }
    
    // 加速度计滤波
    *ax_filt = Kalman_Update(&kf_ax, ax, 0);
    *ay_filt = Kalman_Update(&kf_ay, ay, 0);
    *az_filt = Kalman_Update(&kf_az, az, 0);
    
    // 陀螺仪滤波：卡尔曼 + 三重移动平均
    float gx_kalman = Kalman_Update(&kf_gx, gx, 1);
    float gy_kalman = Kalman_Update(&kf_gy, gy, 1);
    float gz_kalman = Kalman_Update(&kf_gz, gz, 1);
    
    *gx_filt = Triple_MovingAverage(gx_kalman, 0);
    *gy_filt = Triple_MovingAverage(gy_kalman, 1);
    *gz_filt = Triple_MovingAverage(gz_kalman, 2);
    
    // 输出限制
    #define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
    
    *ax_filt = CLAMP(*ax_filt, -2.0f, 2.0f);
    *ay_filt = CLAMP(*ay_filt, -2.0f, 2.0f);
    *az_filt = CLAMP(*az_filt, -2.0f, 2.0f);
    *gx_filt = CLAMP(*gx_filt, -250.0f, 250.0f);
    *gy_filt = CLAMP(*gy_filt, -250.0f, 250.0f);
    *gz_filt = CLAMP(*gz_filt, -250.0f, 250.0f);
}

```

kalman_filter.h

```c
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

// 卡尔曼滤波初始化 (只需要调用一次)
void KalmanFilter_Init(void);

// 卡尔曼滤波更新函数 (在循环中调用)
// 输入: 原始传感器数据 (加速度g, 角速度°/s)
// 输出: 滤波后的传感器数据
void KalmanFilter_Update(float ax, float ay, float az, 
                        float gx, float gy, float gz,
                        float *ax_filt, float *ay_filt, float *az_filt,
                        float *gx_filt, float *gy_filt, float *gz_filt);

#ifdef __cplusplus
}
#endif

#endif

```

attitude.c

```c
#include "attitude.h"
#include <math.h>
#include <string.h>
#include <stdint.h>

#define PI 3.14159265358979323846f
#define RAD_TO_DEG 57.2957795130823208768f
#define DEG_TO_RAD 0.01745329251994329577f

// 四元数结构体
typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion;

// 姿态解算器结构体
typedef struct {
    Quaternion quat;            // 当前四元数
    float beta;                 // 梯度下降法步长
    float sample_freq;          // 采样频率 (Hz)
    float dt;                   // 采样周期 (秒)
    float gyro_bias[3];         // 陀螺仪零偏 [gx, gy, gz]
    float bias_alpha;           // 零偏估计系数
    float bias_threshold;       // 零偏更新阈值
    uint32_t calibration_count; // 校准计数
    uint8_t calibrated;         // 校准完成标志
    uint32_t update_count;      // 更新计数器
} AttitudeSolver;

// 静态变量，内部使用
static AttitudeSolver attitude;
static uint8_t initialized = 0;

// 四元数转欧拉角
static void Quaternion_ToEuler(const Quaternion *q, float *roll, float *pitch, float *yaw) {
    float q0 = q->q0, q1 = q->q1, q2 = q->q2, q3 = q->q3;
    
    // 横滚角 (x轴旋转)
    *roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 
                   1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;
    
    // 俯仰角 (y轴旋转)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f) {
        *pitch = copysignf(90.0f, sinp);
    } else {
        *pitch = asinf(sinp) * RAD_TO_DEG;
    }
    
    // 偏航角 (z轴旋转)
    *yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 
                  1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;
}

// 改进的零偏校准函数
static void Attitude_CalibrateGyro(float gx, float gy, float gz) {
    static uint32_t stable_count = 0;
    static float last_gx = 0, last_gy = 0, last_gz = 0;
    
    if (attitude.calibration_count < 2000) {  // 增加到2000个样本
        // 检查数据稳定性
        if (fabsf(gx - last_gx) < 0.1f && fabsf(gy - last_gy) < 0.1f && fabsf(gz - last_gz) < 0.1f) {
            stable_count++;
        } else {
            stable_count = 0;
        }
        
        // 只有稳定时才采集数据
        if (stable_count > 100) {
            attitude.gyro_bias[0] += gx;
            attitude.gyro_bias[1] += gy;
            attitude.gyro_bias[2] += gz;
            attitude.calibration_count++;
        }
        
        last_gx = gx;
        last_gy = gy;
        last_gz = gz;
        
        if (attitude.calibration_count == 2000) {
            // 计算平均值
            attitude.gyro_bias[0] /= 2000.0f;
            attitude.gyro_bias[1] /= 2000.0f;
            attitude.gyro_bias[2] /= 2000.0f;
            attitude.calibrated = 1;
            
            
        }
    }
}

// 姿态解算初始化 (只需要调用一次)
void AttitudeSolver_Init(void) {
    if (initialized) return;
    
    // 初始化四元数
    attitude.quat.q0 = 1.0f;
    attitude.quat.q1 = 0.0f;
    attitude.quat.q2 = 0.0f;
    attitude.quat.q3 = 0.0f;
    
    // 初始化参数
    attitude.beta = 1.0f;               // 梯度下降法步长
    attitude.sample_freq = 100.0f;      // 100Hz采样频率
    attitude.dt = 1.0f / attitude.sample_freq;
    
    // 零偏参数
    attitude.gyro_bias[0] = 0.0f;
    attitude.gyro_bias[1] = 0.0f;
    attitude.gyro_bias[2] = 0.0f;
    attitude.bias_alpha = 0.01f;        // 零偏估计系数
    attitude.bias_threshold = 0.5f;     // 零偏更新阈值(°/s)
    
    attitude.calibration_count = 0;
    attitude.calibrated = 0;
    attitude.update_count = 0;
    
    initialized = 1;
}

// 姿态解算更新函数 (在循环中调用)
void AttitudeSolver_Update(float ax, float ay, float az, 
                          float gx, float gy, float gz,
                          float *roll, float *pitch, float *yaw) {
    
    // 确保已经初始化
    if (!initialized) {
        AttitudeSolver_Init();
    }
    
    // 零偏校准（前2000次采样）
    if (!attitude.calibrated) {
        Attitude_CalibrateGyro(gx, gy, gz);
        *roll = *pitch = *yaw = 0.0f;
        return;  // 校准期间不更新姿态
    }
    
    // 应用初始零偏校正并转换为弧度/秒
    gx = (gx - attitude.gyro_bias[0]) * DEG_TO_RAD;
    gy = (gy - attitude.gyro_bias[1]) * DEG_TO_RAD;
    gz = (gz - attitude.gyro_bias[2]) * DEG_TO_RAD;
    
    Quaternion *q = &attitude.quat;
    float q0 = q->q0, q1 = q->q1, q2 = q->q2, q3 = q->q3;
    float recipNorm;
    float vx, vy, vz;
    float ex, ey, ez;
    float halfT = attitude.dt * 0.5f;
    
    // 归一化加速度计数据
    recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    
    // 估计重力和向量的方向
    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    
    // 计算加速度计测量的误差 (叉积)
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);
    
    // 积分误差比例增益
    ex *= attitude.beta;
    ey *= attitude.beta;
    ez *= attitude.beta;
    
    // 动态零偏估计 - 只在静止或低速时更新
    if (fabsf(gx * RAD_TO_DEG) < attitude.bias_threshold &&
        fabsf(gy * RAD_TO_DEG) < attitude.bias_threshold &&
        fabsf(gz * RAD_TO_DEG) < attitude.bias_threshold) {
        
        // 缓慢更新零偏估计
        attitude.gyro_bias[0] += attitude.bias_alpha * ex * RAD_TO_DEG;
        attitude.gyro_bias[1] += attitude.bias_alpha * ey * RAD_TO_DEG;
        attitude.gyro_bias[2] += attitude.bias_alpha * ez * RAD_TO_DEG;
        
        // 限制零偏范围，防止过度补偿
        for (int i = 0; i < 3; i++) {
            if (attitude.gyro_bias[i] > 5.0f) attitude.gyro_bias[i] = 5.0f;
            if (attitude.gyro_bias[i] < -5.0f) attitude.gyro_bias[i] = -5.0f;
        }
    }
    
    // 调整陀螺仪测量值
    gx += ex;
    gy += ey;
    gz += ez;
    
    // 四元数微分方程
    q0 += (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 += (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 += (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 += (q0 * gz + q1 * gy - q2 * gx) * halfT;
    
    // 归一化四元数
    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q->q0 = q0 * recipNorm;
    q->q1 = q1 * recipNorm;
    q->q2 = q2 * recipNorm;
    q->q3 = q3 * recipNorm;
    
    // 更新欧拉角
    Quaternion_ToEuler(q, roll, pitch, yaw);
    
    attitude.update_count++;
}

// 获取校准状态
uint8_t AttitudeSolver_IsCalibrated(void) {
    return attitude.calibrated;
}

// 获取当前零偏估计
void AttitudeSolver_GetBias(float *bias_x, float *bias_y, float *bias_z) {
    if (bias_x) *bias_x = attitude.gyro_bias[0];
    if (bias_y) *bias_y = attitude.gyro_bias[1];
    if (bias_z) *bias_z = attitude.gyro_bias[2];
}

```

attitude.h

```c
#ifndef ATTITUDE_H
#define ATTITUDE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// 姿态解算初始化 (只需要调用一次)
void AttitudeSolver_Init(void);

// 姿态解算更新函数 (在循环中调用)
// 输入: 传感器数据 (加速度g, 角速度°/s)
// 输出: 欧拉角 (度)
void AttitudeSolver_Update(float ax, float ay, float az, 
                          float gx, float gy, float gz,
                          float *roll, float *pitch, float *yaw);

// 获取校准状态
uint8_t AttitudeSolver_IsCalibrated(void);

// 获取当前零偏估计
void AttitudeSolver_GetBias(float *bias_x, float *bias_y, float *bias_z);


#ifdef __cplusplus
}
#endif

#endif

```



freertos.c

```c
/* USER CODE BEGIN Application */
void TASK_MPU_F(void *ptr){
  while (1)
  {
    MPU6050_Read();
    KalmanFilter_Update(Ax,Ay,Az,Gx,Gy,Gz,&ax_filt,&ay_filt,&az_filt,&gx_filt,&gy_filt,&gz_filt);
    AttitudeSolver_Update(ax_filt,ay_filt,az_filt,gx_filt,gy_filt,gz_filt,&roll,&pitch,&yaw);
    memcpy(package+4*0,&Ax,4);
    memcpy(package+4*1,&Ay,4);
    memcpy(package+4*2,&Az,4);
    memcpy(package+4*3,&Gx,4);
    memcpy(package+4*4,&Gy,4);
    memcpy(package+4*5,&Gz,4);
    memcpy(package+4*6,&ax_filt,4);
    memcpy(package+4*7,&ay_filt,4);
    memcpy(package+4*8,&az_filt,4);
    memcpy(package+4*9,&gx_filt,4);
    memcpy(package+4*10,&gy_filt,4);
    memcpy(package+4*11,&gz_filt,4);
    memcpy(package+4*12,&roll,4);
    memcpy(package+4*13,&pitch,4);
    memcpy(package+4*14,&yaw,4);
    
    osDelay(10);
  }
}

void TASK_TRANS_F(void *ptr){
  while(1){
    if(DMA_ready){
    DMA_ready = 0;
    
    HAL_UART_Transmit_DMA(&huart1,(const uint8_t *)package,channel*4+4);
    }
    osDelay(10);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  if(huart ->Instance == USART1){
    DMA_ready = 1;
  }
}

void package_Init(void){
  package[channel*4+0]=0x00;
  package[channel*4+1]=0x00;
  package[channel*4+2]=0x80;
  package[channel*4+3]=0x7f;
}

```

***注:因为要先采集2000个数据来消除零偏,所以一开始MPU6050要静止平放,并等待一段启动时间***

***

效果图:

**先来看一下消除零偏效果:10minyaw角零偏0.2°,其实还可以更优化,不过够用了我就不搞了**

![Snipaste_2025-11-16_05-19-49](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-16_05-19-49.png)

![IMG_1042](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/IMG_1042.PNG)

加速度(x):

![Snipaste_2025-11-16_03-58-27](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-16_03-58-27.png)

陀螺仪(y)(整体和局部):

![Snipaste_2025-11-16_03-59-06](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-16_03-59-06.png)

![Snipaste_2025-11-16_03-59-16](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-16_03-59-16.png)

姿态角:

![Snipaste_2025-11-16_03-27-15](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-16_03-27-15.png)

![Snipaste_2025-11-16_03-31-04](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-16_03-31-04.png)

**==这个时候就会有人说了:"欸,你这姿态角的演示数据为什么这么小啊?"==**

**==别急,这个才是我们的重头戏-----bug与debug(之一)==**

***

## bug与debug

因为bug太过多与奇怪所有单开一个章节(前面的题目也有好多bug的说)

***

**bug之一:**

​	因为算法我不是很会所以是copy的,但是也就是因为不是自己写的,所以有些参数跟我的设置并不是很匹配,一开始我的滤波后的数据经常很奇怪**什么好几个数据为零,好几个数据相等,还有什么0和最大值来回跳等等**,当时我还以为是算法的问题所以换了好几个不同版本的算法,后来在我的逐渐排查下,其实就是读算法程序(虽然很多看不懂),发现了两个关键点:**算法的采样频率和我的实际的采样频率不对应**,更改后效果明显,但是数据还是不对劲

**bug之二:**

​	我在一行行检查我的代码的时候(这个真的很痛苦,问ai它也不会,还是得自己来),**发现我的原始数据定义的类型为无符号16位整数,而事实上他应该是有符号16位整数(这其实在4.1中就有用到过)**,修改后我的数据终于正常了

**bug之三:**

​	现在让我们来看一下为什么我上面的演示姿态角都很小吧,**==前排提示,胆小勿入,这个bug折磨我到凌晨3:40才解决==**

​	bug是这样的:**我在移动我的MPU6050(也就是我的面包板)的时候我的三个姿态角会突然归零,然后一段时间后又恢复正常**

​	我一开始以为是算法的问题**(因为算法不是我写的,所以我始终对其保持怀疑)**,**而且我通过看几次归零的曲线发现,归零总与滤波数据的突变发生在一起**,这更加坚定了我的想法:**数据大幅度突变造成了四元数归一化发散**,所以我更换了无数的滤波算法和姿态角求解算法,去各个渠道更换了好多算法都有这样的问题,所以我陷入了停滞

​	**==后来我知道事实并非如此==**,**因为我发现了一些反例-----滤波数据没有突变而姿态角却归零**,突破进展是在一次实验中,我偶然看见,**TTL转USB的数据传输指示灯闪了一下**,于是我在造成闪烁的附近多晃悠了几下,**发现在一些特定的位置下甚至STM32的电源灯也灭了(我会把视频传到github上)**,我立即认识到了事情的严重性,**我的ST-Link明明插在我的电脑上,但是却掉电了!我的脑海中闪过了一个可怕的念头---不会是接触不良吧?!我立刻放大了姿态角归零处的数据曲线:**

***(粉线是一个轴角速度原始数据曲线,橙线是一个roll角数据曲线)***

![Snipaste_2025-11-16_04-45-05](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-16_04-45-05.png)

![Snipaste_2025-11-16_04-45-47](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-16_04-45-47.png)

**==在这一瞬间我顿悟了:是接触不良造成了这个bug!==**

**下面让我详细解释一下:**

1. 由上图知,姿态角在归零后,**一段时间后**都是零,然后重新有正常值,陀螺仪数据**很快**就有了值,还记的我前面说过的话吗,我的程序**需要一定时间初始化来消除零偏**,我们有理由推测**==在很短的时间内(这个图由于数据变换过快所以有误差)比如大概0.1s左右发生了接触不良,快速的断电再上电相当于reset,而姿态角在初始化阶段为0,原始数据和滤波数据不受影响,几乎无间断==**
2. ==**由于我的程序初始化姿态角为0所以姿态角在reset的时候会变为0而不是其他值**==
3. ==**由于在我较快速移动的时候更容易接触不良,所以大多数的归零都与数据的突变伴随,当然有些时候即使没有快速移动也会造成接触不良,这也就解释了前文的现象**==
4. **==当reset后,初始化完成了以后,姿态角就正常了,所以一段时间后正常==**
4. ==**还有一些辅助条件:我平时不小心碰到我的ST-LINK的时候会听到笔记本的usb插入/断开提示声,不过当时因为没影响所以就没在意;以及很久之前的一天晚上我的一个朋友一肘把我的整个面包板全干地上了,当时几个连接ST-LINK的针脚都弯了,后来还是我拿镊子掰回来的**==

**真是一场酣畅淋漓的推导啊!**不过虽说如此,我却没有好的解决办法,所以我只能小心翼翼地测量,所以演示的姿态角都比较小.

***

尽管debug没有思路的时候很红温,但是当找到问题所在的时候还是很爽的,奇怪的debug经验+1(但是真的花了我好多时间啊,大哭)

另外总感觉有些懵懵的,毕竟没了解算法原理就拿来用还是怪怪的

***

# 4.3 屏幕驱动

*~~难道不用......不用再debug了吗?!~~*

***

## 基础知识

***

* OLED:OLED即有机发光管(Organic Light-Emitting Diode,OLED),屏的大小为0.96寸，像素点为 128*64，所以我们称为0.96oled屏或者12864屏,地址默认为0x3C

* OLED本身是没有显存的，他的现存是依赖SSD1306提供的，而SSD1306提供一块显存,SSD1306显存总共为128*64bit大小，SSD1306将这些显存分成了8页。每页包含了128个字节

  ![c761d38b4e37a77c15c9b2df61cdc6dc](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/c761d38b4e37a77c15c9b2df61cdc6dc.png)

* STM32内部建立一个缓存(共128*8个字节),每次修改的时候,只是修改STM32上的缓存(实际上就是SRAM),修改完后一次性把STM32上的缓存数据写入到OLED的GRAM

* **上面我们在学习I2C协议的时候,说过第二个字节的含义是有从机自己决定的,对于OLED来说,第二个字节是标志判断,主要是用来设置连续模式(每个data byte前都有control byte还是只有一个control byte),以及写入的是指令还是数据,一般要写入命令就是0x00,数据就是0x40**

* 第三个字节即为写入的命令或数据,然后终止(也可以不终止,连续写入)

  ![7588e004ca3d0ed2fd13212142564031](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/7588e004ca3d0ed2fd13212142564031.png)

* SSD1306芯片提供三种寻址模式:
  1. 页寻址模式:写入数据后,列指针自增,在当前页循环**(默认模式,常用)**
  2. 水平寻址模式:可以换页的页寻址模式
  3. 垂直寻址模式:写入数据,页地址指针自增,自动换列

* OLED通过显示字符点阵来显示字符
* 写入原理是在起始点的位置写入传输的数据

***

## 工程建立

代码(这里只给出有关OLED的代码,字库代码,字库的代码太长了不给出):

screen.h

```c
#include <fonts.h>
#include <i2c.h>

#define OLED_ADD (0x3C << 1)
#define CMD 0x00
#define DAT 0x40

void WriteCmd(unsigned char I2C_Command);
void WriteDat(unsigned char I2C_Data);
void OLED_Init(void);
void OLED_SetPos(unsigned char x, unsigned char y);
void OLED_Fill(unsigned char fill_Data);
void OLED_CLS(void);
void OLED_ON(void);
void OLED_OFF(void);
void OLED_ShowChar(uint8_t x,uint8_t y,char ch);
void OLED_ShowString(unsigned char x, unsigned char y,char ch[]);

```

screen.c

```c
#include <screen.h>

void WriteCmd(unsigned char I2C_Command) 
 {
	HAL_I2C_Mem_Write(&hi2c2, OLED_ADD, CMD, I2C_MEMADD_SIZE_8BIT, &I2C_Command, 1, 1000);
 }
		
void WriteDat(unsigned char I2C_Data)    
 {
		HAL_I2C_Mem_Write(&hi2c2,OLED_ADD,DAT,I2C_MEMADD_SIZE_8BIT,&I2C_Data,1,100);
  }
 
void OLED_Init(void)
{
	HAL_Delay(100); 
	
	WriteCmd(0xAE); //display off
	WriteCmd(0x20);	//Set Memory Addressing Mode	
	WriteCmd(0x10);	//00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	WriteCmd(0xb0);	//Set Page Start Address for Page Addressing Mode,0-7
	WriteCmd(0xc8);	//Set COM Output Scan Direction
	WriteCmd(0x00); //---set low column address
	WriteCmd(0x10); //---set high column address
	WriteCmd(0x40); //--set start line address
	WriteCmd(0x81); //--set contrast control register
	WriteCmd(0xff); //亮度调节 0x00~0xff
	WriteCmd(0xa1); //--set segment re-map 0 to 127
	WriteCmd(0xa6); //--set normal display
	WriteCmd(0xa8); //--set multiplex ratio(1 to 64)
	WriteCmd(0x3F); //
	WriteCmd(0xa4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	WriteCmd(0xd3); //-set display offset
	WriteCmd(0x00); //-not offset
	WriteCmd(0xd5); //--set display clock divide ratio/oscillator frequency
	WriteCmd(0xf0); //--set divide ratio
	WriteCmd(0xd9); //--set pre-charge period
	WriteCmd(0x22); //
	WriteCmd(0xda); //--set com pins hardware configuration
	WriteCmd(0x12);
	WriteCmd(0xdb); //--set vcomh
	WriteCmd(0x20); //0x20,0.77xVcc
	WriteCmd(0x8d); //--set DC-DC enable
	WriteCmd(0x14); //
	WriteCmd(0xaf); //--turn on oled panel
}
 
void OLED_SetPos(unsigned char x, unsigned char y) //设置起始点坐标
{ 
	WriteCmd(0xb0+y);
	WriteCmd(((x&0xf0)>>4)|0x10);
	WriteCmd((x&0x0f)|0x01);
}
 
void OLED_Fill(unsigned char fill_Data)//全屏填充
{
	unsigned char m,n;
	for(m=0;m<8;m++)
	{
		WriteCmd(0xb0+m);		//page0-page1
		WriteCmd(0x00);		//low column start address
		WriteCmd(0x10);		//high column start address
		for(n=0;n<128;n++)
			{
				WriteDat(fill_Data);
			}
	}
}
 
 
void OLED_CLS(void)//清屏
{
	OLED_Fill(0x00);
}
 
void OLED_ON(void)
{
	WriteCmd(0X8D);  //设置电荷泵
	WriteCmd(0X14);  //开启电荷泵
	WriteCmd(0XAF);  //OLED唤醒
}
 
void OLED_OFF(void)
{
	WriteCmd(0X8D);  //设置电荷泵
	WriteCmd(0X10);  //关闭电荷泵
	WriteCmd(0XAE);  //OLED休眠
}

//显示单个字符
void OLED_ShowChar(uint8_t x,uint8_t y,char ch){
    uint8_t i = 0;
    uint8_t c = ch-32;

    if(c >= 95){
        return;
    }

    OLED_SetPos(x,y);
    for(;i<8;i++){
        WriteDat(Font8x16[c][i]);
    }
    OLED_SetPos(x,y+1);
    for(;i<16;i++){
        WriteDat(Font8x16[c][i]);
    }
}

//显示字符串
void OLED_ShowString(unsigned char x, unsigned char y,char ch[]){
    uint8_t j = 0;
    while(ch[j] != '\0'){
        if(x <= 127-8){
            OLED_ShowChar(x,y,ch[j]);
            x+=8;
            j++;
        }		
	}
}

```

freertos.c

```c
void TASK_TRANS_F(void *ptr){
  while(1){
    char buff[16];
    count++;
    if(count >= 50){
      OLED_CLS();
      count = 0;
    }
    sprintf(buff,"roll:%.1f",roll);
    OLED_ShowString(0,0,buff);
    
    sprintf(buff,"pitch:%.1f",pitch);
    OLED_ShowString(0,2,buff);

    sprintf(buff,"yaw:%.1f",yaw);
    OLED_ShowString(0,4,buff);

    osDelay(10);
  }
}
```

***

**从上文中可以看出I2C的Mem_Write API的第三个参数并不是寄存器地址,其实是第二个字节的内容,在MPU6050中它是寄存器地址,在OLED中它是用来配置的**

在这个基础上WriteCmd和WriteDat函数就好理解了



然后再介绍一下OLED_ShowChar和OLED_ShowString~~*(因为这些是我写的)*~~

我的字库包含了从空格开始的ASCII中的字符,所以要减32并判断是否在字库内

因为我的字体是8*16所以中间要换页,以及每个字符之间列要加8,同时防止超过最大显示范围

(由于使用的是页寻址模式),所以只需要在换页的时候,移动起始点

值得注意的是,在打印不同行的字符串的时候,页每次变化二

**以及加上了定时清屏,防止上一次的数据残留**

***

效果图:

![IMG_1043](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/IMG_1043.JPG)

***

# 5.1 PID基础理论

**我会实现电机定速以及倒立摆**(我使用的是江协的开发板,原理图我会传到Github上)

***

## 基础理论部分

* PID是比例（Proportional）、积分（Integral）、微分（Differential）的缩写,对应着公式中的三个部分

* PID是一种闭环控制算法，它动态改变施加到被控对象的输出值（Out），使得被控对象某一物理量的实际值（Actual），能够快速、准确、稳定地跟踪到指定的目标值（Target）;PID是一种基于误差（Error）调控的算法，其中规定：误差=目标值-实际值，==**PID的任务是使误差始终为0**==;PID对被控对象模型要求低，无需建模，即使被控对象内部运作规律不明确，PID也能进行调控

  *(其中闭环控制的意为存在反馈机制,作用结果将作为影响下一次作用的因素)*

* PID公式:
  $$
  误差:
  error(t) = target(t) - actual(t)
  $$

  $$
  标准形式:
  out(t) = K_p \left(error(t) + \frac{1}{T_i} \int_0^t error(t)dt + T_d \cdot \frac{derror(t)}{dt}\right)
  $$

  $$
  常用形式:
  out(t) = K_p \cdot error(t) + K_i \cdot \int_0^t error(t)dt + K_d \cdot \frac{derror(t)}{dt}
  $$
  
  ![Snipaste_2025-11-18_21-51-19](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-18_21-51-19.png)
  
  1. P(比例项):
  
     * 比例项的输出值仅取决于当前时刻的误差，与历史时刻无关。当前存在误差时，比例项输出一个与误差呈正比的值，当前不存在误差时，比例项输出0
     * K~p~越大，比例项权重越大，系统响应越快，但超调也会随之增加(就是补过头了),以及对噪声敏感等
     * 纯比例项控制时，系统一般会存在稳态误差，K~p~越大，稳态误差越小
  
     **注:理想状态下(只有P项且不考虑其他因素)当K~p~＞2时,系统甚至会出现自激振荡**
  
     ​	 **P项的缺点主要在于稳态误差,这一概念在I项中在介绍**
  
  2. I(积分项):
  
     * 稳态误差:让我们想象这样一个场景:只利用P项来调控PWM占空比,电机的加速度与误差成正比,理想状态下有:
       $$
       \dot{V} = k (V_0 - V) \
       $$
  
       $$
       即 V(t) = V_0 + (V(0) - V_0)e^{-kt}
       $$
  
       显然速度是趋近于我们设定的速度V~0~的,**但是实际情况下存在摩擦等阻力,使在未达到V~0~的时候就停止了加速,这就是稳态误差**
  
       1. PID稳态误差：系统进入稳态时，实际值和目标值存在始终一个稳定的差值
       2. 稳态误差产生原因：纯比例项控制时，若误差为0，则比例项结果也为0。被控对象输入0时，一般会自发地向一个方向偏移，产生误差。产生误差后，误差非0，比例项负反馈调控输出，当调控输出力度和自发偏移力度相同时，系统达到稳态
       3. 判断是否会产生稳态误差：给被控对象输入0，判断被控对象会不会自发偏移(电机是否会自发变化速度)
       4. 判断稳态误差的方向：给被控对象输入0，自发偏移方向即为稳态误差方向(电机的速度是自发增加还是减少)
  
     * 积分项的主要作用就是为了消除稳态误差,从上文知道,为了消除稳态误差,我们还要引入新的项,而这个项有两个要求:**第一,要能反映系统的稳态误差;第二,在稳态误差消失后仍然能保持固定的值**,于是积分概念的引入就显得很自然了.**存在稳态误差的时候,积分项会不断增大,逐渐弥补稳态误差,直到稳态误差不存在,积分值也不变**
  
       1. 积分项的输出值取决于0到t所有时刻误差的积分，与历史时刻有关。积分项将历史所有时刻的误差累积，乘上积分项系数K~i~后作为积分项输出值
       2. K~i~越大，积分项权重越大，稳态误差消失越快，但也会产生超调,震荡对噪声敏感等缺点,总体上体现为响应变慢
  
  3. D(微分项):
  
     * 微分项的输出值取决于当前时刻误差变化的斜率，与当前时刻附近误差变化的趋势有关。当误差急剧变化时，微分项会负反馈输出相反的作用力，阻碍误差急剧变化
     * 斜率一定程度上反映了误差未来的变化趋势，这使得微分项具有 “预测未来，提前调控”的特性
     * 微分项给系统增加阻尼，可以,加快震荡结束的过程有效防止系统超调，处理自激震荡,尤其是惯性比较大的系统
     * K~d~越大，微分项权重越大，系统阻尼越大，但由于微分项阻碍误差变化系统卡顿现象也会随之增加
  
     
  
* 离散形式的PID公式:

  由于CPU的工作是离散的,程序也只能编写成离散的形式,所以我们需要使用离散形式的PID公式
  $$
  标准离散形式:out(k) = K_p \cdot error(k) + K_i \cdot T \sum_{j=0}^k error(j) + K_d \cdot \frac{error(k) - error(k-1)}{T}
  $$

  $$
  简化形式(将调控周期T并入系数):out(k) = K_p \cdot error(k) + K_i' \cdot \sum_{j=0}^k error(j) + K_d' \cdot (error(k) - error(k-1))
  $$

  实际上就是对积分和微分进行了离散处理(T为调控周期)

  上述公式其实就是位置式PID公式
  $$
  增量式PID公式:\Delta out(k) = K_p \cdot (error(k) - error(k-1)) + K_i \cdot error(k) + K_d \cdot (error(k) - 2error(k-1) + error(k-2))
  $$
  

  **==其实连续形式的PID公式是函数,离散形式的PID公式是数列,位置式是数列通项公式,增量式是递推公式==**

* 位置式PID和增量式PID计算时产生的中间变量不同，如果对这些变量加以调节，可以实现不同的特性,另外增量式PID只需要附近3次的error值进行计算

* 增量式PID算法适合自动与手动的切换,因为是在每次out值的基础上加减

***

* 编码器电机:

  1. 减速电机:使用5~12V供电,其运动控制由TB6612电机驱动模块控制,运动参数由解码器记录输出,减速比为2783/300=9.27666.....
  
  2. TB6612电机模块:
     * 分为A,B两套输入输出,我这里介绍A组,原理一致
     * PWMA:通过修改PWM波形占空比来控制电机转速
     * AIN1,AIN2:通过修改两引脚电平来控制正转还是反转(高低为反,低高为正,***这个要与你的编码器的速度正负相对应***,低低滑行,高高制动)
     * AO1,AO2:连接电机输出
     * VM:连接高压电源
     * STBY:使能引脚
  
     ![Snipaste_2025-11-21_00-08-37](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-21_00-08-37.png)
     
  3. 编码器:
  
     * 内部集成两个霍尔元件(YS2402,区分N/S极,锁存即磁场消失时也可以继续输出信号),分别通过EA,EB两个引脚输出,S极低电平,N极高电平
     * 磁铁为多级径向磁铁,一共有11对极
     
     ![Snipaste_2025-11-21_00-28-24](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-21_00-28-24.png)
     
     * AB向正交波形输出:两个霍尔元件输出方波,二者相位相差90°,即霍尔元件一个正对N/S极,一个位于N/S极交界处
     
     * 在这种情况下基础脉冲数为11PPR(磁铁转一圈A/B相输出11个脉冲)
     
     * 在STM32的配置中利用定时器的Encoder模式可以对其进行计数**(实际上就是利用编码器产生的脉冲作为时钟)**,一般配置为A/B相每个边沿都计数,所以一个脉冲CNT会＋4,即磁铁转一圈CNT+44
     
       由减速比知,输出轴每旋转一圈,CNT相差408.173左右
     
     * **IC Filter:STM32CubeMX为我们提供了输入消抖功能,其消抖原理与串口通信中起始位侦测等数据采样的方法一致**
     
     * 之前提到CNT有自增和自减不同的模式,利用这个来做到正负的区别
     
     * 编码器速度方向判断依赖于在特定边沿时刻的另一相信号电平(判断A,B相电平谁的相位更大),两通道polarity相同时逆时针为正值,相反时顺时针为正

***

**因为代码比较多所以这里就不展示了,我会把它传到我的Github上,这里只说一说注意事项和我遇到的bug,参数的调整我单独讲**

实物图:

![IMG_1098](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/IMG_1098.JPG)

***

## 代码实现1

* STM32CubeMX配置中,将Encoder Mode选为Encoder Mode TI1 and TI2,即为上述的一次脉冲CNT+4,*(保证分辨率)* **而这也就决定了arr的值不能太小,最起码要能储存一次读取周期内的CNT变量,并且长时间使用时必须读取每一次CNT的增量,防止CNT在达到arr的值后归零**
* 无需对Encoder模式的定时器分频,因为脉冲相当于一般定时器所用的系统时钟,分频后将是几次脉冲才记录一次
* 编码器数据读取周期与PID控制周期一致即可,这样二者均不会重复操作产生浪费
* PID算法中输入的是转速,但输出的是PWM占空比,二者单位等等都不一样,需要进行转化,这里我测量了电机的最大转速,并结合ccr最大值,将PID算法与归一化结合

***

* OLED显示部分,我使用了sprintf格式化字符串,但是这个函数占用的内存比较大,我一开始在freertos的堆栈分配中给少了,产生了显示不全的bug,增大堆栈分配后解决
* 在进行单位转换时(把边沿数每40ms转化为弧度每秒)我一开始使用的是长式子,但是由于中间的浮点误差,除只保留整数部分等等原因导致结果总为0,把长式子自己计算为一个常数后解决
* 一开始我想在配置为Encoder模式的TIM3中断回调函数中写PID==**(真不知道我当时怎么想的)**==,由上知这个中断永远不会被触发,所以PID不会执行,新建了TIM1后在TIM的回调函数中写PID解决了问题**(其实根本原因是当时对原理还是不是很熟悉,改完这个bug之后大彻大悟)**
* TB6612电机驱动对输入的PWM波形频率有要求,我一开始没有考虑这个所以电机一直不动,后来把频率改为100kHz后解决

***因为是第一次把学到的各个模块的代码组合在一起,外加上还有新的知识,所以bug非常多,这里只挑几个代表性的讲***

***另外值得注意的是Get_Cur_Spead函数由于会对CNT清零,所以只能调用一次,否则输出的数据不正确***

***

## PID算法改进

***

1. 积分限幅(针对位置式PID,增量式PID在限制输出的时候就已经完成了积分限幅):

   * 针对问题:执行器因为卡住等原因无法消除误差,积分项会无限加大,进入深度饱和状态,PID持续输出最大值,即使后续执行器恢复正常,PID也会一直输出最大值,直到从深度饱和状态中退出,且执行器卡住时间越长,从深度饱和状态中退出的越慢

   * 改进措施:对积分项给一个最大值,最小值进行限制就行,这个很容易实现,这里就不展示了

     **另一个推荐的方法是对Ki*积分项进行总体限幅,这样限制的幅度就有了实际的物理意义,方便设置**

2. 积分分离

   * 针对问题:当仅依靠P项不足以消除较小的误差的时候就要进入I项,但是I项会持续累计,如果我们不需要I项的累计去抵消什么而是仅仅希望消除误差的化就会出现由引入I项产生的超调

     **通过缩短积分项累计的时间,只在调控后期累计,那么就不会产生超调**

   * 改进措施:在积分项前加个变量C,误差大于某个值时C=0,不引入积分,小于某个值时C=1,引入积分

3. 变速积分(积分分离的升级版)

   * 针对问题:积分分离中要是误差突然超过了阈值,那么积分项作用会瞬间消失,产生突变,为了解决这个情况,你可以增大阈值,但是更好的是利用变速积分,建立误差大小与积分项强度的关系

   * 改进措施,与积分分离类似,不过C的变化由另外的函数决定

     如:![Snipaste_2025-11-26_01-02-10](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-26_01-02-10.png)

4. 微分先行

   * 针对问题:当目标值大幅度跳变时，误差也会瞬间大幅度跳变，这会导致微分项突然输出一个很大的调控力,而且这个调控力起到了阻尼的反作用

     <img src="https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-26_01-11-20.png" alt="Snipaste_2025-11-26_01-11-20" style="zoom:50%;" />

   * 改进措施:将对误差的微分替换为对实际值的微分

     $$ d_{out}(k) = -K_g \times (actual(k) - actual(k-1)) $$

   * 注:非微分先行时,D项输出的大的正向调控力会增强P项的作用,所以微分先行后,一般系统的反应速度会变慢一点

5. 不完全微分(其实就是滤波)

   * 针对问题:系统存在噪声时,对P,I项的影响都不大,但是对D项的影响很大,所以需要对D项的输入进行滤波使其更平滑**(不对整体数据进行滤波的原因是对整体数据进行滤波会产生延迟)**

     ![Snipaste_2025-11-26_01-24-20](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-26_01-24-20.png)

   * 改进措施:**这里其实完全可以利用4.2中学到的各种滤波方法**,不过我就介绍一种最简单的低通滤波方法

     $$ d_{out}(k) = (1 - \alpha) \times K_g \times (error(k) - error(k-1)) + \alpha \times d_{out}(k-1) $$

     **这里$$ \alpha $$相当于置信度,是更加相信上一次的测量值作为无噪声值还是这一次的实际测量值作为无噪声值**

6. 输出偏移

   * 针对问题:执行器需要一定的力度才能启动(比如受到了摩擦力的影响),这样可能会引起调控误差,并减慢响应速度

   * 改进措施:在非零输出时加入初始的基值偏移

   * 注:输出偏移会使PID的每次调控都会引起实际的改变,由于误差会出现最终不断抖动的状态
     $$
     \text{out}(k) = 
     \begin{cases} 
     0 & (\text{out}(k) = 0) \\ 
     \text{out}(k) + \text{offset} & (\text{out}(k) > 0) \\ 
     \text{out}(k) - \text{offset} & (\text{out}(k) < 0) 
     \end{cases}
     $$
     

7. 输入死区

   * 针对问题:误差在容忍范围内时,没有必要继续调控

   * 改进措施,判断误差小于某个值时不进行调控,大于某个值时才进行调控
     $$
     \text{out}(k) = 
     \begin{cases} 
     0 & (|\text{error}(k)| < A) \\ 
     \text{out}(k) & (|\text{error}(k)| \geq A) 
     \end{cases}
     $$

**==值得注意的是,输出偏移+输入死区可以把误差限定在我们设定的一个范围内,这与积分分离/变速积分的目的是一致的,这是也是两套解决微小误差的方案==**

***

## 双环PID控制

* 单环PID控制只能控制一个物理量,所以需要多环/串级PID同时控制多个物理量,同时多环PID可以有更高的精准度和响应速度,稳定性(内环PID响应快速,外环PID可以加大调节)

* 双环PID结构图:

  ![Snipaste_2025-11-26_22-38-04](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-26_22-38-04.png)

* 一般内环控制反应更快的物理量(比如速度),外环控制反应更慢的物理量(比如位置)

  **所以,一般外环调控周期>=内环调控周期(还有一个原因是如果外环调控周期更短,但是内环并没有用到新的外环输出,那么外环较小的调控周期也是没有意义的)**

***

## 代码实现2

***

**调参太痛苦了,调了好几天都没什么大的进展,==所以这里只得暂时跳过,代码为未调参完成的情况==**

***

* 实物图:

  ![IMG_1121](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/IMG_1121.JPG)

* 我先对PID代码进行了封装,利用结构体使PID函数可以复用

* 双环PID概念图:

  ![Snipaste_2025-11-27_23-42-17](C:/Users/xiaohuangyu/Desktop/Snipaste_2025-11-27_23-42-17.png)

  1. 角度环PID:角度传感器检测到摆杆偏移了竖直位置,就会调整PWM输出来移动横杆来"接住"摆杆,使其保持竖直

  2. 位置环PID:当横杆要变化位置时,通过调整摆杆的目标角度,使摆杆前倾/后倾,那么横杆为了维持这一前倾/后倾的角度就会朝着对应的方向移动,从而到达目标位置

     **值得注意的是,位置环PID的输出角度要在中心角度(也就是摆杆竖直角度)的基础上改变,保证横杆达到目标位置的时候,摆杆能够保证竖直**

* 角度传感器使用的是电位器传感器,所以需要注意的是,会用角度盲区的存在**位于角度盲区时,电位器悬空,值会向2048靠拢,所以我设计了盲区检测,确保摆杆下落的时候电机不再工作**

* **这里对角度传感器数据的处理涉及到了ADC,这部分的代码我参考了Deepseek的回答,*如果之后有时间会回过头来补齐这块知识*,目前我只需要了解角度传感器根据摆杆角度的不同输出不同的电压,ADC将电压转换为0~4095中的一个数值,由此我可以将其转化为角度**

## 我的困境

1. 若K~p~满足了调控所需的速度要去,那么它的值就太大了难以通过K~d~来消除剧烈的抖动
2. 降低K~p~后为了弥补只得引入一个极小的K~i~,但是只要K~i~存在,系统就难以稳定
3. 中心角度难以调整

**最终我只能保证倒立摆能不倒,但是会有抖动和旋转,以及无法实现外层PID控制**

***

## 破除困境(吗?)

**调了好几天总算把内层PID的参数调的差不多了**

并有了新的发现,**D项如果过大也会产生震荡**,整体的调参过程其实是这样

1. 先确定K~p~的大概范围
2. 然后确定K~i~的大概范围
3. 最后确定K~d~的大概范围
4. 细致调整K~i~
5. 细致调整K~p~和K~d~
6. 再次调整K~i~

**==另外要说明的是,由于机械结构的问题即我的摆杆的质量分布及其不均,上方的质量占据了大部分的比例,所以惯性的影响会比较大,体现出来就是横杆会一直动(但我可以确定不是抖动),可能在加上外环PID控制后可以好一些==**

**==主要还是超调难以完美解决==**

## 参数调整

* 一般按照kp,ki,kd的顺序调整(以电机定速为例)

* 只使用P项的时候会有稳态误差,要先把P项调节到震荡较小

  前:![Snipaste_2025-11-25_23-25-41](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-25_23-25-41.png)

  后:

  ![Snipaste_2025-11-25_23-42-17](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-25_23-42-17.png)

*  在此基础上调节I项消除稳态误差

  ![Snipaste_2025-11-26_00-09-39](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-26_00-09-39.png)

* 如果存在明显的过调可以加入D项(不过一般不需要)

最终效果:

![Snipaste_2025-11-26_00-36-53](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-26_00-36-53.png)

**==但是我不论怎么调整参数都会有上下1.5左右的毛刺波动,而且这个1.5左右的数值与我的Spead_Tar无关,而且似乎也不是抖动,因为我开启了IC滤波器也会有这样的毛刺,暂时我无法解决,或许是机械结构导致的?==**

***

# 5.2CAN通信理论

**不得不感慨CAN通信协议从硬件层到协议层设计的都太绝妙了,不过升级之后就有点屎山代码的样子了,我估计设计者也没招了,竟然还能在协议中出现一个无意义位,不过瑕不掩瑜,第一代的CAN协议真的太美丽了**

***

## CAN总线特征

* CAN总线(Controller Area Network Bus)即控制器局域网总线

* 两根通信线(CAN_H、CAN_L),差分信号通信，抗干扰能力强***(实际上,为了防止收发器承受较高的电压,一般CAN通信是会共地的,另外如果还需要供电则还需要一条供电线路,所以如果想要同时实现供电和通信一般需要4根线)***

* 高速CAN(ISO11898):125k~1Mbps, <40m;低速CAN(ISO11519):10k~125kbps, <1km

* 异步，无需时钟线，通信速率由设备各自约定

* 半双工，可挂载多设备，多设备同时发送数据时通过仲裁判断先后顺序

* 11位/29位报文ID，用于区分消息功能，同时决定优先级

* 可配置1~8字节的有效载荷

* 可实现广播式和请求式两种传输方式

* 应答、CRC校验、位填充、位同步、错误处理等特性

* **之前所学习的串口通信和I2C通信分别主要用于"点对点"式通信和一个主控与多个模块的通信**

  **而CAN总线则适用于多个主控直接互相通信的情况**

***

## CAN总线硬件电路

这里我着重介绍并学习高速CAN总线

***

* 每个设备通过CAN收发器挂载在CAN总线网络上
  CAN控制器引出的TX和RX与CAN收发器相连，CAN收发器引出的CAN_H和CAN_L分别与总线的CAN_H和CAN_L相连**(这里都是对应相连,不需要交叉)**

* ![can1](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/can1.png)

  ![can2](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/can2.png)

* 对于闭环CAN总线,两端的120Ω电阻一是起到了防止回波反射,二是**在没有设备操作CAN总线时,保证CAN_H和CAN_L电压一致,此时代表"1",CAN总线被操作时,出现差分电压,代表"0",(这个跟I2C通信有些类似,想发送1就不去操作,默认发送1,发送0时才操作总线)**

* ![Snipaste_2025-11-30_15-26-02](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-30_15-26-02.png)

  CAN总线采用差分信号,即两线电压差(V~CAN_H~-V~CAN_L~)传输数据位
  高速CAN规定:
  	电压差为0V时表示逻辑1(隐性电平)
  	电压差为2V时表示逻辑0(显性电平)
  低速CAN规定:
  	电压差为-1.5V时表示逻辑1(隐性电平)
  	电压差为3V时表示逻辑0(显性电平)

***

## CAN总线帧格式

* ![Snipaste_2025-11-30_15-41-59](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-30_15-41-59.png)

1. 数据帧:

   ![CAN4](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/CAN4.png)

   * 起始帧:发送0用于打破总线宁静(与串口通信类似)

   * 报文ID:用于表示后面数据的功能,不同功能的数据帧ID都不同(即配置好某个节点接收什么类型的报文ID的数据,确保数据只会被需要的节点使用),同时报文ID还参与仲裁机制,ID小的优先发送

   * RTR:远程请求标志位数据帧为0,遥控帧为1,数据帧和遥控帧的ID是可以相同的,当ID相同时,数据帧的优先级大于遥控帧

   * **11位报文ID与RTR共同组成了仲裁段,这与I2C通信中的7位从机地址+1位读写位的设置很像,==I2C中通过从机地址指定来保证数据能被正确的设备接收,而CAN通信中,数据帧是广播的,通过配置其他设备的过滤来保证数据被正常的设备接收;,结合前面对数据帧和遥控帧的介绍,数据帧可以看作是写入,遥控帧可以看作是读取==**

   * IDE:区分扩展格式还是标志格式,标准格式为0,扩展格式为1

   * r0:保留位,无实际意义,为了后期升级等预留出来的位置

   * DLC:表示后续数据的长度

   * CRC,CRC界定符:数据校验;界定CRC结束

   * ACK槽,ACK界定符:数据应答**(这又与I2C的应答机制很像,发送1实际上是不对CAN总线操作,数据接收方如果接收到了数据会在ACK槽这一位控制总线输出0,发送方也会接收判断这一位是否为0,来判断是否接收到了数据)**

     另外CRC界定符位发送方就会释放总线,为ACK做准备,ACK界定符位接收方也会释放总线

     **注:ACK槽时,允许多个设备同时输出0,代表多个设备接收到了数据,这与I2C通信不同**

   * EOF:连续的七个"1"代表发送结束(与串口通信中的停止位类似)

   当标准格式的11位ID不够用时,就出现了扩展格式

   * SRR:填补原来RTR的缺失,**这个与仲裁机制有关,仲裁中,先根据ID大小判断,再根据RTR判断,所以RTR必须在所有ID的后面,那么这一个原本的RTR就空出来了,而标准格式的优先级高于扩展格式,所以SRR写为"1"**
   * r1:其实在标准格式中,IDE本来就是r1,与r0起相同的作用,不过现在他有了实际的意义,而在扩展格式中就可以恢复它原本的保留位的身份了
   * **值得注意的是报文ID只有11位,所以会按照3/4/4的形式划分,使用时需要注意ID范围**

2. 遥控帧:

   ![Snipaste_2025-11-30_16-43-26](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-30_16-43-26.png)

   * 遥控帧无数据段,RTR电平为1,其他部分与数据帧相同
   * 请求被接收后,接收端发送的数据帧与遥控帧有相同的ID

3. 错误帧:

   ![CAN5](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/CAN5.png)

   * 总线上所有设备都会监督总线的数据,一旦发现"位错误"或“填充错误”或“CRC错误”或“格式错误”或“应答错误” ,这些设备便会发出错误帧来破坏数据,同时终止当前的发送设备
   * 主动错误状态:检测到错误时发送0即操作总线,所以设备接收到的数据都是被破坏的
   * 被动错误状态:检测到错误时发送1即不去操作总线,只有自己的数据是被破坏的
   * 而一个设备的错误帧发出可能会引起其他的错误帧的发出,所以实际上错误帧会有0~6位的延长

4. 过载帧:

   ![CAN6](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/CAN6.png)

   * 当接收方收到大量数据而无法处理时,其可以发出过载帧,延缓发送方的数据发送,以平衡总线负载,避免数据丢失(其本质与错误帧类似)

5. 帧间隔:

   ![CAN8](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/CAN8.png)

   * 将数据帧和遥控帧与前面的帧分离开
   * 也分为主动错误状态和被动错误状态,下方为被动错误状态,代表设备不可靠,降低设备的发送数据的频率,使其在仲裁中处于不利地位

**==注意:为了区分扩展格式帧和遥控帧,在接收第13位为1时会在向下读取一位,即IDE,若IDE为1,则为扩展格式帧,至于是数据帧还是遥控帧则需继续向下读取到RTR,若IDE为0,则为标准格式遥控帧==**

6. 位填充:
   * 发送方每发送5个相同电平后,自动追加一个相反电平的填充位,接收方检测到填充位时,会自动移除填充位,恢复原始数据
   * 当添加的位与后面的四位又组成了连续的5个相同的位时会再次进行位填充
   * 当有多个相同的位时,会每五个添加一个填充位
   * 如:     即将发送:	100000110	   10000011110	0111111111110
               实际发送:	1000001110	   1000001111100	011111011111010
               实际接收:	1000001110	   1000001111100	011111011111010
               移除填充后: 100000110	   10000011110	0111111111110
   * 位填充作用:
     1. 增加波形的定时信息,利于接收方执行“再同步”,防止波形长时间无变化,导致接收方不能精确掌握数据采样时机,即不能进行误差补偿
     2. 将正常数据流与“错误帧”和“过载帧”区分开,标志“错误帧”和“过载帧”的特异性
     3. 保持CAN总线在发送正常数据流时的活跃状态,防止被误认为总线空闲(CAN通信协议规定,连续11个1认为是总线空闲)

***

## 数据采样与位同步

CAN总线没有时钟线，总线上的所有设备通过约定波特率的方式确定每一个数据位的时长,但是存在采样点对齐和误差积累等问题

***

### 位时序

![wsx1](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/wsx1.png)

* 为了灵活调整每个采样点的位置,使采样点对齐数据位中心附近,CAN总线对每一个数据位的时长进行了更细的划分,分为同步段(SS),传播时间段(PTS),相位缓冲段1(PBS1)和相位缓冲段2(PBS2),每个段又由若干个最小时间单位(Tq)构成
* 其中Tq是可以自行指定的,另外SS:1Tq,PTS:1\~8Tq,PBS1:1\~8Tq,PBS2:2\~8Tq
* 当跳变沿出现在SS段的时候就说明位时序和波形达成了同步,在PBS1和PBS2之间采样的数据是准确的
* PTS段:为了抵消各种延迟
* PBS1,PBS2:采样点位于PBS1和PBS2之间,通过调整PBS1,2的长短就可以调整采样点位置

### 硬同步

![Snipaste_2025-11-30_18-12-50](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-30_18-12-50.png)

* 每个设备都有一个位时序计时周期,当某个设备(发送方)率先发送报文,其他所有设备(接收方)收到SOF的下降沿时,接收方会将自己的位时序计时周期拨到SS段的位置,与发送方的位时序计时周期保持同步
* 硬同步只在帧的第一个下降沿(SOF下降沿)有效
* 经过硬同步后,若发送方和接收方的时钟没有误差,则后续所有数据位的采样点必然都会对齐数据位中心附近

***

### 再同步

![ztb1](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/ztb1.png)

* 若发送方或接收方的时钟有误差,随着误差积累,数据位边沿逐渐偏离SS段,则此时接收方根据再同步补偿宽度最大值(SJW)通过加长PBS1段,或缩短PBS2段,以调整同步

* 再同步可以发生在第一个下降沿之后的每个数据位跳变边沿
* SJW:1~4Tq,存在SJW主要是为了防止噪声的干扰,防止出现噪声时,误把一位数据看作是时钟误差过大的多位数据

***

### 波特率计算

* 波特率 = 1 / 一个数据位的时长 = 1 / (T~SS~ + T~PTS~ + T~PBS1~ + T~PBS2~)
* 如:SS = 1Tq，PTS = 3Tq，PBS1 = 3Tq，PBS2 = 3Tq
  	Tq = 0.5us
  	波特率 = 1 / (0.5us + 1.5us + 1.5us + 1.5us) = 200kbps

***

## 总线资源分配

### 先占先得

* 若当前已经有设备正在操作总线发送数据帧/遥控帧,则其他任何设备不能再同时发送数据帧/遥控帧(可以发送错误帧/过载帧破坏当前数据)
* 任何设备检测到连续11个隐性电平,即认为总线空闲,只有在总线空闲时,设备才能发送数据帧/遥控帧

### 非破坏性仲裁

***当几个设备想同时发送数据的时候就会进行非破坏性仲裁(如在某个设备A发送数据中多个设备B,C都想发送数据,当A数据发送结束时,B与C之间就会发生非破坏性仲裁,或者很巧合地两个设备想同时发送数据)***

***

* CAN总线协议会根据ID号(仲裁段)进行非破坏性仲裁,ID号小的(优先级高)取到总线控制权,ID号大的(优先级低)仲裁失利后将转入接收状态,等待下一次总线空闲时再尝试发送

* 实现非破坏性仲裁的两个要求:
  1. 线与特性
  2. 回读机制:每个设备发出一个数据位后,都会读回总线当前的电平状态,以确认自己发出的电平是否被真实地发送出去了,根据线与特性,发出0读回必然是0,发出1读回不一定是1

* 仲裁过程:

  ![zc1](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/zc1.png)

  * 两个设备同时发出仲裁段波形并进行回读,当两个设备发送的位一致时,二者的回读都正确,当一个发送1一个发送0时,由于线与特性发送1的设备回读是0,产生了差异,该设备就会退出,等待总线空闲继续尝试发送,仲裁由此产生
  * 仲裁过程解释了为什么ID越小仲裁优先级越高
  * 根据反证很容易得到位填充不影响仲裁过程
  * 仲裁过程中仲裁胜利的设备正常发送了数据所以是非破坏性仲裁
  * 扩展格式与标准格式的仲裁有时只用仲裁段无法判断,这时IDE会解决这个问题(具体情况前文提到过)

***

## 错误处理

* 错误类型:

  ![cw1](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/cw1.png)

* 只要有设备检测到了错误就会发送错误帧破坏数据,直到总线空闲各个设备再次发送数据

* 错误状态:

  主动错误状态:设备正常参与通信并在检测到错误时发出主动错误帧
  被动错误状态:设备正常参与通信但检测到错误时只能发出被动错误帧
  总线关闭状态:设备不能参与通信

* 每个设备内部管理一个TEC和REC，根据TEC和REC的值确定自己的状态

* TEC(Transmit Error Counter 发送错误计数器),设备在发送时每发现一个错误就会自增1,每次正常发送就会自减1

  REC(Receive Error Counter 接收错误计数器),设备在接收时每发现一个错误就会自增1,每次正常接收就会自减1

* 错误状态转换图:

  ![cw2](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/cw2.png)

  ![cw3](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/cw3.png)

***

## STM32CAN外设

### STM32CAN外设简介

* STM32内置bxCAN外设(CAN控制器),支持CAN2.0A和2.0B,可以自动发送CAN报文和按照过滤器自动接收指定CAN报文,程序只需处理报文数据而无需关注总线的电平细节
* 波特率最高可达1兆位/秒,有3个可配置优先级的发送邮箱,2个3级深度的接收FIFO,14个过滤器组
* 特色功能:时间触发通信,自动离线恢复, 自动唤醒,禁止自动重传, 接收FIFO溢出处理方式可配置,发送优先级可配置,双CAN模式

***

### CAN外设基本结构

![Snipaste_2025-11-30_21-44-58](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-30_21-44-58.png)

***注:CPU读取FIFO(First Input First Output 先入先出寄存器)时遵循先来后到先入先出的规则,另外我们可以配置  哪个过滤器输出的报文进入哪个FIFO以及FIFO满了的时候丢弃原有数据还是新接收的数据等,CPU写入邮箱时从邮箱0开始写入,若邮箱占满则等待或放弃,以及发送数据时是按照先来后到还是ID号大小***

***

**发送流程:选择一个空置邮箱→写入报文 →请求发送**

![fs1](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/fs1.png)RQCP(Request Completed):请求完成位

TXOK(Transmission OK):发送成功位

TME(Transmission Mailbox Empty):发送邮箱空位(1代表空置)

TXRQ(Transmit Mailbox Request):发送请求控制位(1代表产生发送请求)

NART(No Automatic Retransmission):禁止自动重传位(0代表使用自动重传)

ABRQ(Abort Request):中止发送

IDLE:空闲

***

**接收流程:接收到一个报文→匹配过滤器后进入FIFO 0或FIFO 1→CPU读取**

![js2](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/js2.png)

FMP(FIFO Message Pending):报文数目

FOVR(FIFO Overrun):FIFO溢出

RFOM(Release FIFO Output Mailbox):释放邮箱

**==注意,溢出与挂号3状态很接近,溢出状态释放邮箱后直接进入挂号2状态==**

***

**发送和接收配置位**

* NART(No Automatic Retransmission):置1，关闭自动重传,CAN报文只被发送1次,不管发送的结果如何(成功、出错或仲裁丢失);置0,自动重传,CAN硬件在发送报文失败时会一直自动重传直到发送成功
* TXFP(Transmit FIFO Priority):置1,优先级由发送请求的顺序来决定,先请求的先发送;置0,优先级由报文标识符来决定,标识符值小的先发送(标识符值相等时，邮箱号小的报文先发送)
* RFLM(Receive FIFO Locked Mode):置1,接收FIFO锁定,FIFO溢出时,新收到的报文会被丢弃;置0,禁用FIFO锁定,FIFO溢出时,FIFO中最后收到的报文被新报文覆盖

***

**标识符过滤器**

* 每个过滤器的核心由两个32位寄存器组成:R1[31:0]和R2[31:0]
* FSCx(Filter Scale Configuration):位宽设置,置0,16位;置1,32位
* FBMx(Filter Mode):模式设置,置0,屏蔽模式;置1,列表模式
* FFAx(Filter FIFO Assignment):关联设置,置0,FIFO 0;置1,FIFO 1
* FACTx(Filter Active):激活设置,置0,禁用;置1,启用

![Filter1](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Filter1.png)

* STID(Standard ID):标准格式ID号

  EXID(Extended ID):扩展格式ID号

  IDE:为1代表过滤扩展ID,为0代表过滤标准ID

  RTR:为0过滤数据帧,为1过滤遥控帧

* 列表模式:直接在寄存器中写入目标ID,只有报文ID与目标ID一致时才能通过过滤器
* 屏蔽模式:一个32位寄存器写目标ID,另一个寄存器写1/0,1代表对应位置的值与目标ID的值必须相等,0代表任意,这样便可以过滤一系列的报文ID(要注意IDE,RTR的值的配置)(其实就是掩码)
* 32位过滤器一个可以过滤2个标准/扩展ID,一个32位过滤器可以拆为两个16位过滤器,但是一般只能过滤标准ID
* 由于赋值是右对齐,所以向寄存器写入数据时记得左移与映像对齐

***

**测试模式**

* 静默模式:用于分析CAN总线的活动,不会对总线造成影响
* 环回模式:用于自测试,同时发送的报文可以在CAN_TX引脚上检测到
* 环回静默模式:用于热自测试,自测的同时不会影响CAN总线
* ![Snipaste_2025-11-30_23-27-44](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-11-30_23-27-44.png)

***

**工作模式**

* 初始化模式:用于配置CAN外设,禁止报文的接收和发送

* 正常模式:配置CAN外设后进入正常模式,以便正常接收和发送报文

* 睡眠模式:低功耗,CAN外设时钟停止,可使用软件唤醒或者硬件自动唤醒

* ![gzms1](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/gzms1.png)

* AWUM(Automatic Wakeup Mode)：置1,自动唤醒,一旦检测到CAN总线活动,硬件就自动清零SLEEP,唤醒CAN外设;置0,手动唤醒,软件清零SLEEP,唤醒CAN外设

* SLAK(Sleep Ack)睡眠确认状态位:1代表已经进入睡眠状态

  INAK(Init Ack)初始化确认位:1代表已经进入初始化模式

  SLEEP:1代表请求进入睡眠

  INRQ:1代表请求进入初始化

  ACK:应答信号

  SYNC:总线空闲信号

***

**位时序**

* STM32CAN外设中将PTS段和PBS1段合并在一起,为BS1段

* ![sx](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/sx.png)

  SS:1Tq

  BS1:1~16Tq

  BS2:1~8Tq

  SJW:1~4Tq

  波特率 = APB1时钟频率 / 分频系数 / 一位的Tq数量
               = 36MHz / (BRP[9:0]+1) / (1 + (TS1[3:0]+1) + (TS2[2:0]+1))

***

**CAN外设中断**

* 发送中断:发送邮箱空时产生
* FIFO 0中断:收到一个报文/FIFO 0满/FIFO 0溢出时产生
* FIFO 1中断:收到一个报文/FIFO 1满/FIFO 1溢出时产生
* 状态改变错误中断:出错/唤醒/进入睡眠时产生

***

**时间触发通信**

* 对节点进行统一调度,使每个节点只在对应的时间段内发送报文,避免优先级仲裁

* TTCM(Time Triggered Communication Mode):置1,开启时间触发通信功能;置0,关闭时间触发通信功能

* CAN外设内置一个16位的计数器,用于记录时间戳,TTCM置1后,该计数器在每个CAN位的时间自增一次,溢出后归零

* 每个发送邮箱和接收FIFO都有一个TIME[15:0]寄存器,发送帧SOF时,硬件捕获计数器值到发送邮箱的TIME寄存器,接收帧SOF时,硬件捕获计数器值到接收FIFO的TIME寄存器(记录时间戳)

* 发送邮箱可配置TGT(Transmit Global Time)位,捕获计数器值的同时,也把此值写入到数据帧数据段的最后两个字节,为了使用此功能,DLC必须设置为8(发送数据的同时发送时间戳),且用户不得对第7,8位进行操作

  ![Snipaste_2025-12-01_00-36-10](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-12-01_00-36-10.png)

***

**错误处理和离线恢复**

* STM32CAN外设提供了额外的功能ABOM(Automatic Bus-off Management)自动离线管理,置1,开启离线自动恢复,进入离线状态后,就自动开启恢复过程;置0,关闭离线自动恢复,软件必须先请求进入然后再退出初始化模式,随后恢复过程才被开启

***

## 代码实现

**代码比较长我会把它传到github上,这里讲方法和注意事项**

***

* 方法:

  ```c
  void CAN_Filter_Config_ListMode(void)  //我们自己定义这个函数用来初始化过滤器
  CAN_FilterTypeDef filter;  //利用这个STM32CubeMX定义的结构体来赋值各个初始化的值
  HAL_CAN_ConfigFilter(&hcan, &filter); //利用这个HAL库函数来初始化过滤器
  
  void CAN_Send(uint32_t id, uint8_t* data, uint8_t len) //自己定义这个函数来发送数据
  CAN_TxHeaderTypeDef tx_header; //利用这个STM32CubeMX定义的结构体来赋值发送数据所需配置的值
  uint32_t mailbox;
  HAL_CAN_AddTxMessage(&hcan, &tx_header, data, &mailbox);//利用这个HAL库函数来发送数据
  
  uint8_t CAN_Receive(uint32_t *id, uint8_t *data, uint8_t *len) ////自己定义这个函数来接收数据
  CAN_RxHeaderTypeDef rx_header; //利用这个STM32CubeMX定义的结构体来存储接收到的值
  HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) //检测是否有数据
  HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rx_header, data) //读取数据
  //rx_header这个结构体里包含了非data的内容,ID,IDE,RTR等等
  *id = rx_header.StdId;
  *len = rx_header.DLC;
  ```

* 注意事项:
  1. mailbox不需要赋值,HAL_CAN_ConfigFilter()会在发送完成后对其进行赋值,发送过程会自动选择邮箱
  
  2. mailbox和FIFO在发送和读取后会硬件自动清除
  
  3. 每次使用STM32定义好的结构体的时候最好memset一下,防止某些情况一些参数没有被赋值影响正常工作

  4. 接收函数中记得用指针传回数据
  
  5. 寄存器位数可以一直使用32位模式,硬件会自动根据IDE判断是一个扩展格式地址还是两个标准格式地址
  
  6. **由于发送由硬件完成,发送不需要DMA的参与,而我所使用的芯片只有FIFO溢出DMA**
  
     **==事实上,由于CAN与DMA的设计理念不同(DMA适合连续数据，CAN是固定小帧;邮箱分散寄存器;8字节数据，CPU比DMA更快;CAN优先实时性，不是吞吐量;等等)在CAN中使用DMA并不是一个好的选择==**
  
* 一些补充:
  1. 我这里由于硬件厂家发错货了,所以其余的STM32和CAN收发器还没有,我这里设置了Loopback模式,也能来判断数据是否正常地发送接收了
  2. 发送遥控帧时DLC并不重要,遥控帧只是与数据帧区分了开来,具体接收到遥控帧该怎么回应需要我们来自己编写代码
  4. **==如果有多个主机发送ID相同的遥控帧,仲裁就会失效,尽量不要设计可能会出现这种情况的程序==**
  
* STM32CubeMX中断配置:

  ![Snipaste_2025-12-03_22-40-11](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/Snipaste_2025-12-03_22-40-11.png)

  发送中断和CAN接收FIFO0中断与USB的中断有共用现象

  下面两个是CAN接收FIFO1中断和CAN状态变化中断

* 回调函数(STM32CubeMX在NVIC中管理CAN的接收中断和状态变化中断):

  ```c
  //邮箱发送完成中断
  void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);
  void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan);
  void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan);
  //发送终止中断
  void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan);
  void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan);
  void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan);
  //FIFO中存在未读取的数据/FIFO溢出中断
  void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
  void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan);
  void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
  void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan);
  //睡眠中断
  void HAL_CAN_SleepCallback(CAN_HandleTypeDef *hcan);
  //因接收到数据而从睡眠中退出的状态变化中断
  void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *hcan);
  //错误回调
  void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);
  //启动中断的API
  HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *hcan, uint32_t ActiveITs)
  ```

  中断接收的实时性很强,适合多主机,总线上信息比较多的情况

* 现象图:

  ![IMG_1147](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/IMG_1147.JPG)

***

# EX:ADC模块

***在5.1中涉及到了一些ADC模块的内容,这里补充一下***

***

## 基础知识

* ADC(Analog-Digital Converter)模拟-数字转换器,ADC可以将引脚上连续变化的模拟电压转换为内存中存储的数字变量

* 12位逐次逼近型ADC,1us转换时间
  输入电压范围:0\~3.3V,转换结果范围:0\~4095
  18个输入通道,可测量16个外部和2个内部信号源
  规则组和注入组两个转换单元
  模拟看门狗自动监测输入电压范围

  支持双ADC模式

* STM32F103C8T6 ADC资源:ADC1,ADC2,10个外部输入通道

* ADC实现的基本原理(逼近型):,内置DAC(数模转换器),通过比较DAC的输出电压和外部输入电压,不断调整(二分法),最终二者一致,此时DAC的输入值即为ADC的输出值

* 单个ADC框图:

  ![ADC1](https://cdn.jsdelivr.net/gh/AAAxiaohuangyu/Pictures@main/ADC1.png)

* 规则通道组:一次性最多可以选16个通道,但是只有一个16位数据寄存器,所以后来的数据会覆盖之前的数据,想要读取每个数据要配合DMA使用
* 注入通道组:一次性最多可以选4个通道,有4个16位数据寄存器,**注入通道的优先级是要高于规则通道的,也就是说如果在规则通道组转换的时候注入通道组也有要转换,那么会先终止规则通道组的转换,等注入通道组转换完之后再继续**
* ADC触发信号:软件触发/硬件触发(定时器触发)
* 模拟看门狗:通过设置阈值高限/低限和看门的通道,使其在特定情况下自动触发中断
* 采样:ADC通过采样电压,并保存一段时间,防止在进行量化,编码的过程中电压不断变化,采样时间(即采样保持时间是可以配置的)
* ADC的转换结果是12位的而数据寄存器是16位的,所以数据存放存在左对齐和右对齐,一般选择右对齐

***

## 配置相关

* 非扫描模式/扫描模式:每一组中只有第一个通道序列可用/都可用
* 单次/连续转换:一次转换后就结束/连续转换
* STM32CubeMX中一般配置为连续模式,DMA配置为Circular

***

## 代码实现

```c
MX_ADC1_Init();
HAL_ADCEx_Calibration_Start(&hadc1); //ADC校准
HAL_ADC_Start(&hadc1); //启动ADC转换
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){} //回调函数
HAL_ADC_Start_DMA(ADC_HandleTypeDef* hadc, uint32_t* pData, uint32_t Length) 
//参数分别为ADC句柄,数据缓冲区,要读取的数据长度
```

**注:DMA一次会把所有配置的通道的数据依次存入缓冲区中,Length=通道数×采样数,可以对通道进行多次采样,每次采样的通道数据相邻,每一组采样数据相邻**

***

# Final:未完成的任务,可以精进的地方,以及碎碎念

**验收时间又往后延长了一个周,所以可以做一些没有完成的任务了**

***

* 未完成任务:

  - [ ] PID倒立摆调参(这个我在拆硬件的时候发现机械结构的连接也有点问题,非常松动,再加上倒立摆调参过程确实有些困难所以暂时搁置了)

    PS:历经了多次尝试仍然无果,只得放弃(**但是相比之前在重新装配了硬件之后内环PID控制勉强能够执行**)

* 可以精进的地方:
  - [x] 使用DMA在各个数据传输的地方代替CPU执行
  - [x] CAN中断接收学习
  - [ ] 使用休眠等手段降低功耗
  - [ ] FreeRTOS的深入学习,包括队列等以及底层架构
  - [x] ADC模块学习
  - [x] ESP8266的AP模式学习
  - [ ] 卡尔曼滤波算法的数学原理,以及更优的零偏消除办法
  - [ ] 四元数法求欧拉角的算法数学原理
  - [ ] 代码规范性,尤其是错误处理和各个API的封装规范以及参数定义规范等等
  
* 碎碎念:

  学起来很快乐,debug很心累

  以及.......哎呀我去,累死我了,终于学完了

  ps:致谢江协,deepseek以及CSDN

***

