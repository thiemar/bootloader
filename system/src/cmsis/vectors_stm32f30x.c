//
// This file is part of the µOS++ III distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include "cortexm/ExceptionHandlers.h"

// ----------------------------------------------------------------------------

void __attribute__((weak))
Default_Handler(void);

// ----------------------------------------------------------------------------

extern unsigned int _estack;

typedef void
(* const pHandler)(void);

// ----------------------------------------------------------------------------

// The vector table.
// This relies on the linker script to place at correct location in memory.

__attribute__ ((section(".isr_vector"),used))
pHandler __isr_vectors[] =
  {
  // Core Level - CM4
      (pHandler) &_estack, // The initial stack pointer
      Reset_Handler, // The reset handler

      NMI_Handler, // The NMI handler
      HardFault_Handler, // The hard fault handler
#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
      MemManage_Handler,                        // The MPU fault handler
      BusFault_Handler,                        // The bus fault handler
      UsageFault_Handler,                        // The usage fault handler
#else
      0, 0, 0,                                  // Reserved
#endif
      0,                                        // Reserved
      0,                                        // Reserved
      0,                                        // Reserved
      0,                                        // Reserved
      SVC_Handler,                              // SVCall handler
#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
      DebugMon_Handler,                         // Debug monitor handler
#else
      0,                                        // Reserved
#endif
      0, // Reserved
      PendSV_Handler, // The PendSV handler
      SysTick_Handler // The SysTick handler
    };

// ----------------------------------------------------------------------------

// Processor ends up here if an unexpected interrupt occurs or a specific
// handler is not present in the application code.

void __attribute__ ((section(".after_vectors")))
Default_Handler(void)
{
  while (1)
    {
    }
}

// ----------------------------------------------------------------------------
