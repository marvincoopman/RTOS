// RTOS ASM functions
// Marvin Coopman

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

#ifndef WAIT_H_
#define WAIT_H_

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

#include <stdint.h>

extern void setASP(void);
extern void setPSP(uint32_t *stack);
extern uint32_t *getPSP(void);
extern uint32_t *getMSP(void);
extern void pushPSPRegisterOffset(uint32_t offset, uint32_t data);
extern void removePriv(void);
extern void pushToPSPStack(void);
extern void popPSPStack(void);
extern void pushDummyPSPStack(uint32_t xPSR, uint32_t pfn);


#endif


