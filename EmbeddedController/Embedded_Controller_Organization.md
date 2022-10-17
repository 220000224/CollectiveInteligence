# Embedded Controller 정리

Written by:   조한세

Course:  임베디드컨트롤러

Program: C/C++

IDE/Compiler: Keil uVision 5

OS: WIn10

MCU:  STM32F411RE (Nucleo-64)

## **contents**

[TOC]



## **용어정리**

1.  Peripheral: I/O Pin Memory 구성

2. CPU: Processor Registers

- All memory address -> Memory map (Reference Manual)

3. Volatile: volatile가 선언된 변수는 최적화에서 제외된다. (자동으로 지워지지 않는 변수)



## **Bitwise**

- **check a_k:** val = a & (1 << 6)

- **check a_k, a_k+1:** val = (a >> 5) & (0x11)

- **set a_k:** a |= (1 << k)

- ** set a_k, a_k+1:**  a |= (0x11 << k)

- **clear a_k:** a &= ~(1 << k), e.q) PA &= ~(5 << 1): clear port 2, 4

- **Toggle a_k:** a ^= 1 << k (^= : XOR)





## **RCC**

### Internal Clock (HSI) for GPIO

1. **Enable HSI and choose as SYSCLK source**
   - Enable HSI: **(RCC->CR: HSION=1)** 
   - Wait until HSI is stable and ready: **(RCC->CR: HSIRDY? 1)** 
   - Choose the system clock switch : **(RCC->CFGR: SW = 00)** 
   - Check if the selected source is correct: **(RCC->CFGR: SWS ? 00)**

2. Configure HSI (optional) 
   - Calibration for RC oscillator: (RCC->CR: HSICAL, HSITRIM)
3. Configure APB/AHB Prescaler (optional) 
   - Change Prescaler: RCC->CFGR: HPRE, PPRE

4.  **Enable GPIOx clock(AHB1ENR )** 
   - Enable (RCC_AHB1ENR) for PORTx

```C
void RCC_HSI_enable()
{
// 1. Enable HSI and choose as SYSCLK source
    //RCC->CR |= ((uint32_t)RCC_CR_HSION);
    RCC->CR |= 0x00000001U;

    // wait until HSI is ready
    //while ( (RCC->CR & (uint32_t) RCC_CR_HSIRDY) == 0 ) {;}
    while ( (RCC->CR & 0x00000002U) == 0 ) {;}
    // Select HSI as system clock source 
    RCC->CFGR &= (uint32_t)(~RCC_CFGR_SW); 	// not essential
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_HSI; //00: HSI16 oscillator used as system clock 
    // Wait till HSI is used as system clock source
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != 0 );

    //EC_SYSTEM_CLK=16000000;
    EC_SYSCLK=16000000;
// 2. Configure HSI (optional)
// 3. Configure APB/AHB Prescaler (optional)
    
// 4. Enable GPIOx clock(AHB1ENR)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
}
```

### PLL for GPIO

1. **Enable either (HSI, or HSE) for PLL and Choose PLL for System Clock**
   - Enable HSE or HSI: **(RCC->CR : HSION=1)**
   - Wait until HSE or HSI is stable: **(RCC->CR : HSIRDY? 1)** 
   - Choose PLL for system clock switch : **(RCC->CFGR : SW = 10)**
   - Check if PLL selection is correct: **(RCC->CFGR : SWS ? 10)**

2. **Select the clock source for PLL**
   - Select the PLL source(HSI or HSE): **(RCC->PLLCFGR : PLLSRC= 0 or 1)**
3. (Optional)Configure PLL parameters
   - Select (M/N/P): **(RCC->PLLCFGR : PLLM, PLLN, …)**

4.  **Enable PLL**
   - Enable main PLL: **(RCC->CR : PLLON=1), (RCC->CR : PLLRDY?0)**
   - 
5. (Optional)Configure APB/AHB Prescaler 
   - Change Prescaler: (RCC->CFGR : HPRE, PPRE)
6. **Enable GPIOx clock(AHB1ENR )**
   - Enable (RCC_AHB1ENR) for PORTx

```C
void RCC_PLL_init() {	
    // To correctly read data from FLASH memory, the number of wait states (LATENCY)
    // must be correctly programmed according to the frequency of the CPU clock
    // (HCLK) and the supply voltage of the device.		
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |=  FLASH_ACR_LATENCY_2WS;

//  1.Enable either (HSI, or HSE) for PLL and Choose PLL for System Clock
    // Enable the Internal High Speed oscillator (HSI)
    RCC->CR |= RCC_CR_HSION;
    while((RCC->CR & RCC_CR_HSIRDY) == 0);

//  2. Select the clock source for PLL
    // Disable PLL for configuration
    RCC->CR    &= ~RCC_CR_PLLON;
    // Select clock source to PLL
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLSRC; 		// Set source for PLL: clear bits
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI; // Set source for PLL: 0 =HSI, 1 = HSE

//  3. (Optional)Configure PLL parameters
    // Make PLL as 84 MHz
    // f(VCO clock) = f(PLL clock input) * (PLLN / PLLM) = 16MHz * 84/8 = 168 MHz
    // f(PLL_R) = f(VCO clock) / PLLP = 168MHz/2 = 84MHz
    RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLLN) | 84U << 6; // (PLLN=84U)
    RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLLM) | 8U ;  // (PLLM=84U)
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;  // (PLLP=84U)
    // 00: PLLP = 2, 01: PLLP = 4, 10: PLLP = 6, 11: PLLP = 8	

    
//  4. Enable PLL 
    RCC->CR   |= RCC_CR_PLLON; 
    while((RCC->CR & RCC_CR_PLLRDY)>>25 != 0);

//  1.(이어서)
    // Select PLL as system clock
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    // Wait until System Clock has been selected
    while ((RCC->CFGR & RCC_CFGR_SWS) != 8UL);

    // The maximum frequency of the AHB and APB2 is 100MHz,
    // The maximum frequency of the APB1 is 50 MHz.
    RCC->CFGR &= ~RCC_CFGR_HPRE;  		// AHB prescaler = 1; SYSCLK not divided (84MHz)
    RCC->CFGR &= ~RCC_CFGR_PPRE1; 		// APB high-speed prescaler (APB1) = 2, HCLK divided by 2 (42MHz)
    RCC->CFGR |=  RCC_CFGR_PPRE1_2;
    RCC->CFGR &= ~RCC_CFGR_PPRE2; 		// APB high-speed prescaler (APB2) = 1, HCLK not divided	(84MHz)

    EC_SYSCLK=84000000;
}
```

**CR** (clock control register): Enable the clock of GPIO Port (RM p.102)

![image](https://user-images.githubusercontent.com/113971258/195986593-4c66eeb3-fa49-4b96-876e-bae28afe4915.png)

**CFGR** (clock configuration register): Configuration System clock (RM p.106)

![image](https://user-images.githubusercontent.com/113971258/195986614-b3ecc17b-b42a-430b-bbb8-bef148adb982.png)

**PLLCFGR** (PLL configuration register): Configuration PLL (RM p.104)

![image](https://user-images.githubusercontent.com/113971258/195986627-041e8115-105f-475b-9f49-0a3bd073dcb5.png)

**AHB1ENR** (peripheral clock enable register): peripheral (GPIO) clock (RM p.117)

![image](https://user-images.githubusercontent.com/113971258/195986634-f673b840-9cd3-457b-bb73-cda392a013bb.png)

**APB1ENR** (peripheral clock enable register): peripheral (TIM2 to 5) clock (RM p.118)

![image](https://user-images.githubusercontent.com/113971258/195986643-836dc758-431f-4760-bf06-0bbe0663da17.png)

**APB2ENR** (peripheral reset register): peripheral (TIM1, 9, 10, 11) clock (RM p.121)

![image](https://user-images.githubusercontent.com/113971258/195986652-a20f4851-d540-4b3f-af31-b5447abcf315.png)



## **GPIO digital I/O**

### Process of GPIOx register initiation 

#### **Ouput setting:**

0. Enable Peripheral Clock (**AHB1ENR**)

1. Configure as Digital Output (**MODER**)

2. Configure pull-up/down resistors (**PUPDR**)

3. For Output: Configure Output Type (**OTYPE**)

4. For Output: Configure Output Speed (**OSPEEDR**)

5. Output Data **(ODR)**

#### **Input setting:**

0. Enable Peripheral Clock (**AHB1ENR**)

1. Configure as Digital Output (**MODER**)

2. Configure pull-up/down resistors (**PUPDR**)

3. Input Data **(IDR)**



- **AHB1ENR**: 

```c
void RCC_GPIOx_enable()
{
	// RCC Peripheral Clock Enable Register 
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOxEN;
}
```



- **MODER**:

  - 00: input
  - 01: output
  - 10: AF
  - 11: Analog

  ```c
  void GPIO_mode(GPIO_TypeDef *Port, int pin, int mode){
     Port->MODER &= ~(3UL<<(2*pin));  //reset(clear)   
     Port->MODER |=  mode<<(2*pin);   //set 
  }
  ```

- **PUPDR**: 

  - 00: No pull-up, pull-down 
  - 01: Pull-up 
  - 10: Pull-down 
  - 11: Reserved

  ```c
  void GPIO_pupdr(GPIO_TypeDef *Port, int pin, int pupd){
  	Port->PUPDR  &= ~(3UL<<(2*pin));	//clear
  	Port->PUPDR  |=  pupd<<(2*pin);		//set
  }
  ```

- **OTYPE**:

  - 0: Output push-pull (reset state) 
  - 1: Output open-drain

  ```c
  void GPIO_otype(GPIO_TypeDef *Port, int pin, int type){
  	if(type == 0)	Port->OTYPER &= ~(1UL<<pin); 	// push-pull
  	else 			Port->OTYPER |=  (1UL<<pin); 	// open drain
  }
  ```

  

- **OSPEEDR**:

  - 00: Low speed 
  - 01: Medium speed 
  - 10: Fast speed 
  - 11: High speed

  ```c
  void GPIO_ospeed(GPIO_TypeDef *Port, int pin, int speed){
  	Port->OSPEEDR &= ~(3UL<<(2*pin));	// clear
  	Port->OSPEEDR |= speed<<(2*pin);	// set
  }
  ```

- **IDR**: 

```c
int GPIO_read(GPIO_TypeDef *Port, int pin){
	int input = 0;
	input = ((Port->IDR)>>pin)& 1;

	return input;
}
```

- **ODR**: 

```c
void GPIO_write(GPIO_TypeDef *Port, int pin, int Output){
	Port->ODR &= ~(1UL << pin);		// clear
	Port->ODR |= (Output << pin);	// set
}
```

##### 

| GPIO Schametic                                               |
| ------------------------------------------------------------ |
| ![image](https://user-images.githubusercontent.com/113971258/195987550-d94a9af8-23ef-42c5-ab31-314393979d88.png) |



## **Interrupt**

- **ISR (Interrupt Service Routine):** 함수명
- Vector table : RM p.201 참조
- **AHB1**: GPIO, RCC
- **APB1**: TIM2~5
- **APB2**: EXTI, TIM1,9,10,11, SYSCFG



### **SysTick**

- 24-bit down counter
- Time Period = (SysTick_LOAD + 1) × Clock Period
- **SYSCLK**: HSI(16MHz), HSE(8MHz), PLL

![image](https://user-images.githubusercontent.com/113971258/195988656-77559b38-43a7-4fb0-9ead-0f4a9d2561a3.png)

![image](https://user-images.githubusercontent.com/113971258/195988957-3e1fe411-637a-4b9f-afe8-0f60de99d1c8.png)

#### SysTick Configuration

1. Disable SysTick Timer 
   - **SysTick->CTRL ENABLE=0** (SM p.247)
2. Choose clock signal: System clock or ref. clock(STCLK) 
   - **SysTick->CTRL CLKSOURCE = 0 or 1**
     - 0: External clock
     - 1: Processor clock
3. Choose to use Tick Interrupt (timer goes 1->0) 
   - **SysTick->CTRL TICKINT = 0 or 1**
     - 0: Disable SysTick down counter
     - 1: Enable SysTick down counter
4. Write reload Counting value (24-bit) 
   - **SysTick->LOAD RELOAD = (value-1)** 
5. Start SysTick Timer 
   - **SysTick->CTRL ENABLE=1** 
6. (option) Read or Clear current counting value 
   - Read from SysTick->VAL 
   - Write clears value 
7. (option) 10msec calibration value 
   - SysTick->CALIB TENMS = 10ms clock cycle

#### NVIC Configuration

- NVIC SysTick Interrupt priority 
- NVIC SysTick Enable

ecSysTick.h

```c
void SysTick_init(uint32_t msec);
void SysTick_Handler(void);
void SysTick_counter(void);
void delay_ms(uint32_t msec);
void SysTick_reset(void);
uint32_t SysTick_val(void);
```

```c
void SysTick_init(uint32_t msec){	
	//  SysTick Control and Status Register
//  1. Disable SysTick IRQ and SysTick Counter
	SysTick->CTRL = 0;

//  2. Choose clock signal: System clock or ref. clock(STCLK) 
	// 1 = processor clock;  0 = external clock
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
	//SysTick_CTRL_CLKSOURCE_Msk = 1

   
//  4. Write reload Counting value (24-bit) 
	SysTick->LOAD = (MCU_CLK_PLL / 1000)*msec - 1;	// msec[ms], for HSI PLL = 84MHz.

//  6. (option) Read or Clear current counting value
	// Reset the SysTick counter value
	SysTick->VAL = 0;

//  3. Choose to use Tick Interrupt (timer goes 1->0) 
	// 1 = counting down to zero asserts the SysTick exception request
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; // SysTick_CTRL_TICKINT_Msk = 1<<1

//  5. Start SysTick Timer 
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // SysTick_CTRL_ENABLE_Msk = 1<<0
    
//  NVIC Configuration 
	NVIC_SetPriority(SysTick_IRQn, 16);		// Set Priority to 16
	NVIC_EnableIRQ(SysTick_IRQn);			// Enable interrupt in NVIC, SysTick_IRQn -> see vector table at 'STM32 Reference manual'
}
```

```c
void SysTick_Handler(void){
	SysTick_counter();	
}

void SysTick_counter(void){
	//down counter
	msTicks--;
}	


void delay_ms(uint32_t mesc){
  msTicks = mesc;
  while (msTicks != 0);
}



void SysTick_reset(void)
{
	// SysTick Current Value Register
	SysTick->VAL = 0;
}

uint32_t SysTick_val(void) {
	return SysTick->VAL;
}
```



### **EXTI**

#### EXTI Configuration

##### **(Digital Input Setting)**

1. Enable GPIO peripheral clock **(RCC->AHB1ENR )** 

2. Configure digital input pin(push-button) using GPIO registers.

##### **(EXTI Setting)**

3. Connect External Line to the GPIO Port

4. Enable SYSCFG peripheral clock. **(RCC->APB2ENR)** 
5. Connect the corresponding external line to GPIO **(SYSCFG->EXTICR)** 
6. Configure the trigger edge. **(EXTI->FTSR/RTSR)** 
7. Configure Interrupt mask (Enable/Disable EXTI) **(EXTI->IMR)** / Enable = unmask



##### **(NVIC Setting)** 

- Configure the priority of EXTI interrupt request. **(NVIC_SetPriority)** 
- Enable EXTI interrupt request. **(NVIC_EnableIRQ)** 

##### **(EXTI Use)** 

- Create user codes in handler **EXTI_IRQHandler()** 
- Clear pending bit after interrupt call

ecEXTI.h

```c
void EXTI_init(GPIO_TypeDef *Port, int pin, int trig, int priority);
void EXTI_enable(uint32_t pin);
void EXTI_disable(uint32_t pin);
uint32_t is_pending_EXTI(uint32_t pin);
void clear_pending_EXTI(uint32_t pin);
```

```c
void EXTI_init(GPIO_TypeDef* Port, int Pin, int trig_type, int priority) {

//  4. SYSCFG peripheral clock enable	
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

//  3. Connect External Line to the GPIO
	uint32_t EXTICR_port;
	if 		(Port == GPIOA) EXTICR_port = 0;
	else if (Port == GPIOB) EXTICR_port = 1;
	else if (Port == GPIOC) EXTICR_port = 2;
	else if (Port == GPIOD) EXTICR_port = 3;
	else 								   	EXTICR_port = 4;

//  5. Connect the corresponding external line to GPIO
	SYSCFG->EXTICR[Pin / 4] &= ~(0xFUL << (Pin % 4) * 4);			// clear 4 bits
	SYSCFG->EXTICR[Pin / 4] |= EXTICR_port << (Pin % 4) * 4;			// set 4 bits

//  6. Configure the trigger edge
	// Configure Trigger edge
	if (trig_type == FALL) EXTI->FTSR |= 1 << Pin;   // Falling trigger enable 
	else if (trig_type == RISE) EXTI->RTSR |= 1 << Pin;   // Rising trigger enable 
	else if (trig_type == BOTH) {			// Both falling/rising trigger enable
		EXTI->RTSR |= 1 << Pin;
		EXTI->FTSR |= 1 << Pin;
	}

//  7. Configure Interrupt mask (Enable/Disable EXTI)	
    EXTI->IMR |= 1 << Pin;     // not masked


	// NVIC(IRQ) Setting
	int EXTI_IRQn = 0;
	if (Pin < 5) 	EXTI_IRQn = Pin + 6; // EXTI0_IRQn = 6, EXTI1_IRQn = 7, EXTI2_IRQn = 8, EXTI3_IRQn = 9, EXTI4_IRQn = 10
	else if (Pin < 10) 	EXTI_IRQn = EXTI9_5_IRQn;
	else 			EXTI_IRQn = EXTI15_10_IRQn;
	NVIC_SetPriority(EXTI_IRQn, priority);	// EXTI priority
	NVIC_EnableIRQ(EXTI_IRQn); 	// EXTI IRQ enable
}
```

```c
void EXTI15_10_IRQHandler(void) {
	if (is_pending_EXTI(BUTTON_PIN)) {
		// EXTI interrupt Code
		clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
	}
}

uint32_t is_pending_EXTI(uint32_t pin){
	uint32_t EXTI_PRx = (1 << pin);     	// check  EXTI pending 	
	return ((EXTI->PR & EXTI_PRx) == EXTI_PRx);
}
void clear_pending_EXTI(uint32_t pin){
	EXTI->PR |= (1<< pin);     // clear EXTI pending 
}
```

​	

### **Timer Interrupt**

#### TIMx Setting and TIMx Interrupt (no output mode)

##### **System Clock setting** 

1. RCC setting (HSI/HSE/PLL) 

##### **Timer setting** 

1. Enable Timer Peripheral Clock: **(RCC → APB1ENR)** 
2. Set Counting Direction: **(TIMx → CR1 : DIR)** 
3. Set Timer Clock Pre-scaler value: **(TIMx → PSC : PSC[15:0])** 
4. Set Auto-reload value: **(TIMx → ARR : ARR)** 
5. Enable Update or Compare Interrupt: **(TIMx → DIER : UIE) or (: CCyE)** 
6. Enable counter: **(TIMx → CR1 : CEN)** 

##### **NVIC setting** 

1. Set interrupt Priority: **NVIC_SetPriority(TIMx_IRQn,2)** 
2. Enable TIMx Interrupt: **NVIC_EnableIRQ(TIMx_IRQn)**