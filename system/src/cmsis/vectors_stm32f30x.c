extern void __attribute__((noreturn))
bootloader_main(void);
void __attribute__((section(".after_vectors"),naked))
Reset_Handler(void);

void __attribute__((weak))
Default_Handler(void);

void __attribute__ ((section(".after_vectors"),naked))
Reset_Handler(void) {
    asm volatile (
        "ldr r0,=bootloader_main \n"
        "bx r0"
    );
}

void __attribute__((weak,alias("Default_Handler")))
NMI_Handler(void);
void __attribute__((weak,alias("Default_Handler")))
HardFault_Handler(void);

#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
void __attribute__((weak,alias("Default_Handler")))
MemManage_Handler(void);
void __attribute__((weak,alias("Default_Handler")))
BusFault_Handler(void);
void __attribute__((weak,alias("Default_Handler")))
UsageFault_Handler(void);
#endif

void __attribute__((weak,alias("Default_Handler")))
SVC_Handler(void);

#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
void __attribute__((weak,alias("Default_Handler")))
DebugMon_Handler(void);
#endif

void __attribute__((weak,alias("Default_Handler")))
PendSV_Handler(void);
void __attribute__((weak,alias("Default_Handler")))
SysTick_Handler(void);


extern unsigned int _estack;

typedef void (*const handler_t)(void);

__attribute__ ((section(".isr_vector"),used))
handler_t __isr_vectors[] = {
    /* Cortex-M4 core level */
    (handler_t)&_estack, /* Reset value of SP */
    Reset_Handler,       /* Address to jump to on reset */

    NMI_Handler,
    HardFault_Handler,
#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
#else
    0, 0, 0,
#endif
    0,
    0,
    0,
    0,
    SVC_Handler,
#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
    DebugMon_Handler,
#else
    0,
#endif
    0,
    PendSV_Handler,
    SysTick_Handler
};


void __attribute__ ((section(".after_vectors")))
Default_Handler(void) {
    while (1);
}
