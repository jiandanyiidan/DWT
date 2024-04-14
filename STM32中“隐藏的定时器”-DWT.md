# STM32中“隐藏的定时器”-DWT

## 前言

在Cortex-M里面有一个外设叫做DWT（Data Watchpoint and Trace），用于系统调试及跟踪。

DWT为Cortex中的一个“隐藏资源”，他的用途可以给下载器提供时间戳，并在固定的时间间隔将调试数据发到我们的link上面。

## 工作原理

它有一个32位的寄存器叫CYCCNT，它是一个向上的计数器，记录的是内核时钟运行的个数，内核时钟跳动一次，该计数器就加1，精度非常高，如果内核时钟是72M，那精度就是1/72M = 14ns，而程序的运行时间都是微秒级别的，所以14ns的精度是远远够的。

最长能记录的时间为：59.65s。计算方法为2的32次方/72000000。

当CYCCNT溢出之后，会清0重新开始向上计数。

## 使用方法
要实现延时的功能，总共涉及到三个寄存器：DEMCR 、DWT_CTRL、DWT_CYCCNT，分别用于开启DWT功能、开启CYCCNT及获得系统时钟计数值。

### DEMCR
想要使能DWT外设，需要由另外的内核调试寄存器DEMCR的位24控制，写1使能。
DEMCR的地址是0xE000 EDFC

![在这里插入图片描述](https://img-blog.csdnimg.cn/20181110192323371.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2ppZWppZW1jdQ==,size_16,color_FFFFFF,t_70)

![在这里插入图片描述](https://img-blog.csdnimg.cn/20181110201757887.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2ppZWppZW1jdQ==,size_16,color_FFFFFF,t_70)

### DWT_CYCCNT
使能DWT_CYCCNT寄存器之前，先清0。
让我们看看DWT_CYCCNT的基地址，从ARM-Cortex-M手册中可以看到其基地址是0xE000 1004，复位默认值是0，而且它的类型是可读可写的，我们往0xE000 1004这个地址写0就将DWT_CYCCNT清0了。

![img](https://img-blog.csdnimg.cn/20201022093700609.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2Jvb2tzeWhheQ==,size_16,color_FFFFFF,t_70)

### CYCCNTENA
它是DWT控制寄存器的第一位，写1使能，则启用CYCCNT计数器，否则CYCCNT计数器将不会工作。

![img](https://img-blog.csdnimg.cn/20201022094347219.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2Jvb2tzeWhheQ==,size_16,color_FFFFFF,t_70)

### 综上所述

使用DWT的CYCCNT有

1. 先使能DWT外设，这个由另外内核调试寄存器DEMCR的位24控制，写1使能
2. 使能CYCCNT寄存器之前，先清0。
3. 使能CYCCNT寄存器，这个由DWT的CYCCNTENA 控制，也就是DWT控制寄存器的位0控制，写1使能

## 代码实现部分

### 寄存器定义：

```c
//0xE000EDFC DEMCR RW Debug Exception and Monitor Control Register.  
//使能DWT模块的功能位
#define DEMCR           ( *(unsigned int *)0xE000EDFC )  
#define TRCENA          ( 0x01 << 24) // DEMCR的DWT使能位  
  
//0xE0001000 DWT_CTRL RW The Debug Watchpoint and Trace (DWT) unit  
//使能CYCCNT计数器开始计数
#define DWT_CTRL        ( *(unsigned int *)0xE0001000 )  
#define CYCCNTENA       ( 0x01 << 0 ) // DWT的SYCCNT使能位
 
//0xE0001004 DWT_CYCCNT RW Cycle Count register,   
//CYCCNT计数器的内部值(32位无符号)
#define DWT_CYCCNT      ( *(unsigned int *)0xE0001004) //显示或设置处理器的周期计数值  
```

### 初始化部分：

```c
//DWT init
void DWT_Init(uint32_t CPU_Freq_mHz)
{
    /* 使能DWT外设 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* DWT CYCCNT寄存器计数清0 */
    DWT->CYCCNT = (uint32_t)0u;

    /* 使能Cortex-M DWT CYCCNT寄存器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;
    CYCCNT_RountCount = 0;

    DWT_CNT_Update();
}
//get DWT count
uint32_t DWT_TS_GET(void)
{
  return((uint32_t)DWT_CYCCNT);
}
```

### 使用DWT延时函数

```c
void DWT_Delay_Ms(uint32_t time_ms)
{
  uint32_t old_counter,current_counter;
  uint32_t delay_ms;
  
  old_counter = DWT_TS_GET();
  current_counter = DWT_TS_GET();
  delay_ms = 0;
  while(delay_ms<time_ms)
  {
    current_counter = DWT_TS_GET();
    if(current_counter > old_counter)
      delay_ms = (current_counter - old_counter)/(SystemCoreClock/1000);
    else
      delay_ms = (current_counter + 0XFFFFFFFF - old_counter)/(SystemCoreClock/1000);
  }
}
```

### 实现测量代码运行时长

```c
{
  static uint32_t old_counter;
  uint32_t counter,couter_current;
  couter_current = DWT_TS_GET();
  if(couter_current > old_counter)
    counter = couter_current - old_counter;
  else
    counter = couter_current + 0XFFFFFFFF - old_counter;
  old_counter = couter_current;
  return (counter / (SystemCoreClock/1000));
}
```

### 其他功能实现

```c
/**
 * @brief 私有函数,用于检查DWT CYCCNT寄存器是否溢出,并更新CYCCNT_RountCount
 * @attention 此函数假设两次调用之间的时间间隔不超过一次溢出
 *
 * @todo 更好的方案是为dwt的时间更新单独设置一个任务?
 *       不过,使用dwt的初衷是定时不被中断/任务等因素影响,因此该实现仍然有其存在的意义
 *
 */
static void DWT_CNT_Update(void)
{
    static volatile uint8_t bit_locker = 0;
    if (!bit_locker)
    {
        bit_locker = 1;
        volatile uint32_t cnt_now = DWT->CYCCNT;
        if (cnt_now < CYCCNT_LAST)
            CYCCNT_RountCount++;

        CYCCNT_LAST = DWT->CYCCNT;
        bit_locker = 0;
    }
}


/**
 * @brief 获取两次调用之间的时间间隔,单位为秒/s
 *
 * @param cnt_last 上一次调用的时间戳
 * @return float 时间间隔,单位为秒/s
 */
float DWT_GetDeltaT(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

/**
 * @brief 获取两次调用之间的时间间隔,单位为秒/s,高精度
 *
 * @param cnt_last 上一次调用的时间戳
 * @return double 时间间隔,单位为秒/s
 */
double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}
/**
 * @brief DWT更新时间轴函数,会被三个timeline函数调用
 * @attention 如果长时间不调用timeline函数,则需要手动调用该函数更新时间轴,否则CYCCNT溢出后定时和时间轴不准确
 */
void DWT_SysTimeUpdate(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

    DWT_CNT_Update();

    CYCCNT64 = (uint64_t)CYCCNT_RountCount * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
    CNT_TEMP1 = CYCCNT64 / CPU_FREQ_Hz;
    CNT_TEMP2 = CYCCNT64 - CNT_TEMP1 * CPU_FREQ_Hz;
    SysTime.s = CNT_TEMP1;
    SysTime.ms = CNT_TEMP2 / CPU_FREQ_Hz_ms;
    CNT_TEMP3 = CNT_TEMP2 - SysTime.ms * CPU_FREQ_Hz_ms;
    SysTime.us = CNT_TEMP3 / CPU_FREQ_Hz_us;
}
/**
 * @brief 获取当前时间,单位为秒/s,即初始化后的时间
 *
 * @return float 时间轴
 */
float DWT_GetTimeline_s(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;

    return DWT_Timelinef32;
}
/**
 * @brief 获取当前时间,单位为毫秒/ms,即初始化后的时间
 *
 * @return float
 */
float DWT_GetTimeline_ms(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = SysTime.s * 1000 + SysTime.ms + SysTime.us * 0.001f;

    return DWT_Timelinef32;
}
/**
 * @brief 获取当前时间,单位为微秒/us,即初始化后的时间
 *
 * @return uint64_t
 */
uint64_t DWT_GetTimeline_us(void)
{
    DWT_SysTimeUpdate();

    uint64_t DWT_Timelinef32 = SysTime.s * 1000000 + SysTime.ms * 1000 + SysTime.us;

    return DWT_Timelinef32;
}
/**
 * @brief DWT延时函数,单位为秒/s
 * @attention 该函数不受中断是否开启的影响,可以在临界区和关闭中断时使用
 * @note 禁止在__disable_irq()和__enable_irq()之间使用HAL_Delay()函数,应使用本函数
 *
 * @param Delay 延时时间,单位为秒/s
 */
void DWT_Delay(float Delay)
{
    uint32_t tickstart = DWT->CYCCNT;
    float wait = Delay;

    while ((DWT->CYCCNT - tickstart) < wait * (float)CPU_FREQ_Hz)
        ;
}
```

## *优缺点如下：*

1、优点是：方便移植，经过测试在M3、M4、M7内核的MCU上都可以使用。

2、缺点是：和定时器一样，都有一个延时的最大时间，测量代码运行时间的最大值。