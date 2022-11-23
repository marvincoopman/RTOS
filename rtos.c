// RTOS Framework - Fall 2022
// J Losh

// Student Name:
// TO DO: Add your name(s) on this line.
//        Do not include your ID numbers in the file.

// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 6 Pushbuttons and 5 LEDs, UART
// LEDs on these pins:
// Blue:   PF2 (on-board)
// Red:    PE0 (lengthy and general)
// Orange: PA2 (idle)
// Yellow: PA3 (oneshot and general)
// Green:  PA4 (flash4hz)
// PBs on these pins
// PB0:    PD6 (set red, toggle yellow)
// PB1:    PD7 (clear red, post flash_request semaphore)
// PB2:    PC4 (restart flash4hz)
// PB3:    PC5 (stop flash4hz)
// PB4:    PC6 (lengthy priority increase, uncoop)
// PB5:    PC7 (errant)
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1
// Memory Protection Unit (MPU):
//   Region to allow peripheral access (RW) or a general all memory access (RW)
//   Region to allow flash access (XRW)
//   Regions to allow 32 1KiB SRAM access (RW or none)

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "gpio.h"
#include "uart0.h"
#include "wait.h"
#include "clock.h"
#include "rtosASM.h"

// TODO Step7 StartRTOS -> idle -> yield -> idle
// TODO Step8a __asm(" SVC #__)" yield code
// TODO continued yield -> svcIsr -> yield -> idle -> yield -> svcIsr ...
// HINT: SVC service call between 0 - 255 (SvcISR is inside the Kernel-space (MSP, Priv), the rest is Userspace (PSP, Unpriv)
// HINT: StartRTOS needs to setPSP(tcb[taskCount].sp) fn = tcb[].pFn \n *fn() \n setPSP() \n setTmpL
// HINT: check stackpointer is correct inside idle()
// TODO STEP8b SvcIsr ? pendsvIsr will only run if you are not in an interrupt , pendsvISR -> ?

// REQUIRED: correct these bitbanding references for the off-board LEDs
#define BLUE_LED   PORTF,2 // on-board blue LED
#define RED_LED    PORTE,0 // off-board red LED
#define GREEN_LED  PORTA,4 // off-board green LED
#define YELLOW_LED PORTA,3 // off-board yellow LED
#define ORANGE_LED PORTA,2 // off-board orange LED
#define PUSHBUTTON0              PORTD,6
#define PUSHBUTTON1              PORTD,7
#define PUSHBUTTON2              PORTC,5
#define PUSHBUTTON3              PORTC,4
#define PUSHBUTTON4              PORTC,6
#define PUSHBUTTON5              PORTC,7

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
typedef struct _semaphore
{
    uint16_t count;
    uint16_t queueSize;
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
} semaphore;

semaphore semaphores[MAX_SEMAPHORES];
#define keyPressed 1
#define keyReleased 2
#define flashReq 3
#define resource 4

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore

#define MAX_PRIORITIES 8
#define MAX_TASKS 12       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task //HINT: taskCurrent <-- sched \n fn = task[taskCurrent].pfn \n *fn(); setPSP and setTMPL
uint8_t taskCount = 0;     // total number of valid tasks
bool priority;
bool preemption;


// REQUIRED: add store and management for the memory used by the thread stacks
//           thread stacks must start on 1 kiB boundaries so mpu can work correctly

struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *spInit;                  // original top of stack
    void *sp;                      // current stack pointer
    int8_t priority;               // 0=highest to 15=lowest
    uint32_t ticks;                // ticks until sleep complete
    uint32_t srd;                  // MPU subregion disable bits (one per 1 KiB)
    char name[16];                 // name of task used in ps command
    void *semaphore;               // pointer to the semaphore that is blocking the thread
} tcb[MAX_TASKS];

uint32_t *heapTopPtr = 0x20008000;
uint32_t *heapBotPtr = 0x20002000;

#define OFFSET_R0  0
#define SVC_SLEEP  1
#define SVC_YIELD  2
#define SVC_SYSTICK  3
#define SVC_WAIT  4
#define SVC_POST  5
#define SVC_MALLOC  6
#define SVC_PMAP  7



//-----------------------------------------------------------------------------
// Memory Manager and MPU Funcitons
//-----------------------------------------------------------------------------

// TODO: add your malloc code here and update the SRD bits for the current thread
void * mallocFromHeap(uint32_t size_in_bytes)
{
    __asm(" SVC #6");
    return;
}

void * allocaFromHeap(uint32_t size_in_bytes)
{
    if(size_in_bytes == 0)
        return 0;

    if(heapTopPtr - heapBotPtr < size_in_bytes)
        return 0;

    void *temp = (void *)heapBotPtr;
    putxUart0((uint32_t)heapBotPtr);
    heapBotPtr += (((size_in_bytes - 1) / 1024) + 1) * (1024 / 4);
    return temp;
}

// REQUIRED: add your MPU functions here
void setupBackgroundRule(void)
{
    // Background rule
    NVIC_MPU_BASE_R = 0x10;
    NVIC_MPU_ATTR_R = 0x1306003F;
}

void allowFlashAccess(void)
{
//    configureMemoryRegion(FLASH_BASEADDR, SIZE_256KB, FLASH_REGION, RW_RW, FLASH_SCB, SR_ENABLE, EXECUTE_ENABLE);
    // Flash Access
    NVIC_MPU_BASE_R = 0x11;
    NVIC_MPU_ATTR_R =  0x03020023;
}


void setupSramAccess(void)
{
    // SRAM region 0
    NVIC_MPU_BASE_R = 0x20000013;
    NVIC_MPU_ATTR_R = 0x11060019;

    // SRAM region 1
    NVIC_MPU_BASE_R = 0x20002014;
    NVIC_MPU_ATTR_R = 0x11060019;

    // SRAM region 2
    NVIC_MPU_BASE_R = 0x20004015;
    NVIC_MPU_ATTR_R = 0x11060019;

    // SRAM region 3
    NVIC_MPU_BASE_R = 0x20006016;
    NVIC_MPU_ATTR_R = 0x11060019;
}

void setSramAccessWindow(uint32_t mask)
{
    // SRAM region 0
    NVIC_MPU_BASE_R = 0x20000013;
    NVIC_MPU_ATTR_R = 0x11060019 | ((mask << 8) & 0xFF00);

    // SRAM region 1
    NVIC_MPU_BASE_R = 0x20002014;
    NVIC_MPU_ATTR_R = 0x11060019| ((mask << 0) & 0xFF00);

    // SRAM region 2
    NVIC_MPU_BASE_R = 0x20004015;
    NVIC_MPU_ATTR_R = 0x11060019 | ((mask >> 8) & 0xFF00);

    // SRAM region 3
    NVIC_MPU_BASE_R = 0x20006016;
    NVIC_MPU_ATTR_R = 0x11060019| ((mask >> 16) & 0xFF00);
}

uint32_t getSramSRD(uint32_t baseAdd, uint32_t size_in_bytes)
{
    if(size_in_bytes == 0)
        return;
    if(baseAdd < 0x20000000 || baseAdd > 0x20007FFF)
        return;

    uint8_t numberOfSubregions = ((size_in_bytes - 1) / 1024);
    uint32_t currentSubregion = (baseAdd - 0x20000000) / 1024;
    uint32_t mask = ((2 * (1 << numberOfSubregions)) - 1) << currentSubregion;
    return mask;
}
// REQUIRED: initialize MPU here
void initMpu(void)
{
    // REQUIRED: call your MPU functions here
    setupBackgroundRule();
    allowFlashAccess();
    setupSramAccess();
}

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------
// HINT: ALLOCA
// REQUIRED: initialize systick for 1ms system timer (initClickISR)
void initRtos()
{
    uint8_t i;
    // no tasks running
    taskCount = 0;
    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pid = 0;
    }
}

// REQUIRED: Implement prioritization to 8 levels
int rtosScheduler()
{
    bool ok;
    static uint8_t task = 0xFF;
    static uint8_t priorityTaskCurrent[MAX_PRIORITIES] = {0};
    ok = false;
    if(!priority) {
        while (!ok) {
            task++;
            if (task >= MAX_TASKS)
                task = 0;
            ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
        }
    }
    else
    {
        // Saves task current as last used in priority buffer
        priorityTaskCurrent[tcb[taskCurrent].priority] = taskCurrent;
        int8_t maxPriority = 7;
        // Finds lowest priority for all available tasks
        for(task = 0; task < MAX_TASKS; task++)
        {
            if(tcb[task].priority < maxPriority && (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN))
                            maxPriority = tcb[task].priority;
        }

        task = priorityTaskCurrent[maxPriority];
        while(!ok)
        {
            task++;
            if (task >= MAX_TASKS)
                task = 0;
            ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN)
                    && tcb[task].priority == maxPriority;
        }

    }
    return task;
}

bool createThread(_fn fn, const char name[], uint8_t priority, uint32_t stackBytes)
{
    bool ok = false;
    uint8_t i = 0;
    uint8_t j = 0;
    bool found = false;
    // REQUIRED:
    // store the thread name

    // allocate stack space and store top of stack in sp and spInit
    // HINT: cast void * sp to uint8_t * to allow arithmetic
    // HINT: malloc returns low end of the stack while the function needs the top of the stack
    // add task if room in task list
    if (taskCount < MAX_TASKS)
    {
        // make sure fn not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS))
        {
            found = (tcb[i++].pid ==  fn);
        }
        if (!found)
        {
            // find first available tcb record
            i = 0;
            while (tcb[i].state != STATE_INVALID) {i++;}
            tcb[i].state = STATE_UNRUN;
            tcb[i].pid = fn;
            tcb[i].sp = allocaFromHeap(stackBytes);   // HINT: ACTIVE SP (0 if function is inactive)
            tcb[i].spInit = tcb[i].sp;                      // HINT: Top of the stack (backup copy of SP should be result of malloc)
            tcb[i].priority = priority;
            tcb[i].srd = getSramSRD(tcb[i].sp, stackBytes) | 0x1F; // Or'ed with 0x1F to stack in SRAM
            while(name[j] != 0) // Strcpy
            {
                tcb[i].name[j] = name[j];
                j++;
            }
            tcb[i].name[j] = '\0';
            // increment task count
            taskCount++;
            ok = true;
        }
    }
    // REQUIRED: allow tasks switches again
    return ok;
}

// REQUIRED: modify this function to restart a thread
void restartThread(_fn fn)
{
}

// REQUIRED: modify this function to stop a thread
// REQUIRED: remove any pending semaphore waiting
// NOTE: see notes in class for strategies on whether stack is freed or not
void stopThread(_fn fn)
{
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
}

bool createSemaphore(uint8_t semaphore, uint8_t count)
{
    bool ok = (semaphore < MAX_SEMAPHORES);
    {
        semaphores[semaphore].count = count;
    }
    return ok;
}

// REQUIRED: modify this function to start the operating system
// by calling scheduler, setting PSP, ASP bit, TMPL bit, and PC
void startRtos()
{
    taskCurrent = rtosScheduler();
//    tcb[taskCurrent].state =
//    uint32_t sp = 0x20002000;
    uint32_t sp = (uint32_t)tcb[taskCurrent].sp;
    setPSP(sp);
    setASP();
    _fn fn = tcb[taskCurrent].pid;
    tcb[taskCurrent].state = STATE_READY;

    // All tasks will be ran in unpriveleged mode
    setSramAccessWindow(tcb[taskCurrent].srd);
    removePriv();
    fn();

}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
void yield()
{
    // Use assembly
    __asm(" SVC #2");
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
void sleep(uint32_t tick)
{
    __asm(" SVC #1");
}

// REQUIRED: modify this function to wait a semaphore using pendsv
void wait(int8_t semaphore)
{
    __asm(" SVC #4");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(int8_t semaphore)
{
    __asm(" SVC #5");
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{
    uint8_t i;
    for(i = 0; i < taskCount; i++)
    {
        if(tcb[i].state == STATE_DELAYED)
        {
            if(--(tcb[i].ticks) == 0)
                tcb[i].state = STATE_READY;
        }
    }
    if(preemption)
    {
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
    }

    // Ping Pong


}


// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
    // Push regs -> psp
    pushToPSPStack();
    tcb[taskCurrent].sp = (void *)getPSP();
    taskCurrent = rtosScheduler();
    if(tcb[taskCurrent].state == STATE_READY)
    {

        // restore sp <- tcb[taskCurrent].sp
        // pop regs <- psp
        setSramAccessWindow(tcb[taskCurrent].srd);
        setPSP((uint32_t)tcb[taskCurrent].sp);
        popPSPStack();

    }
    else
    {
        // Step 10
        tcb[taskCurrent].state = STATE_READY;
        setSramAccessWindow(tcb[taskCurrent].srd);
        setPSP((uint32_t)tcb[taskCurrent].sp);
        pushDummyPSPStack(0x61000000, (uint32_t *)tcb[taskCurrent].pid);
        // Ensure PC and xPSR are correct
        // PC <- tcb[taskCurrent].sp;
        // xPSR <- research valid values upper 4 bits are ALU 24th bit is important
    }
}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{
    // Extracting SVC value
    uint32_t *psp = getPSP();
    uint16_t *pc = *(psp + 6);
    uint8_t value = (*(pc - 1) & 0xFF);
    switch(value) {
        case SVC_SLEEP:
            tcb[taskCurrent].state = STATE_DELAYED;
            tcb[taskCurrent].ticks = *psp;  // Retrieves R0
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
            break;
        case SVC_YIELD:
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV; // Triggers pendsv fault
            break;
        case SVC_SYSTICK:
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV; // Triggers pendsv fault
            break;
        case SVC_WAIT:
            if (semaphores[*psp].count > 0)
                semaphores[*psp].count--;
            else
            {
                semaphores[*psp].processQueue[semaphores[*psp].queueSize] = taskCurrent; // Add current task to Queue
                semaphores[*psp].queueSize++;
                tcb[taskCurrent].state = STATE_BLOCKED;
                tcb[taskCurrent].semaphore = semaphores[*psp];
                NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV; // Triggers pendsv fault
            }
            break;
        case SVC_POST:
            semaphores[*psp].count++;
            if(semaphores[*psp].queueSize > 0)
            {
                tcb[semaphores[*psp].processQueue[0]].state = STATE_READY;
                tcb[semaphores[*psp].processQueue[0]].semaphore = 0;
                semaphores[*psp].queueSize--;
                uint8_t i;
                for(i = 0; i < semaphores[*psp].queueSize; i++)
                {
                    semaphores[*psp].processQueue[i] = semaphores[*psp].processQueue[i + 1];
                }
                semaphores[*psp].count--;
            }
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV; // Triggers pendsv fault
            break;
        case SVC_MALLOC:
            if(*psp == 0)
                return;

            if(heapTopPtr - heapBotPtr < *psp)
                return;

            heapTopPtr -= (((*psp - 1) / 1024) + 1) * (1024 / 4);
            tcb[taskCurrent].srd |= getSramSRD(heapTopPtr, *psp);
            pushPSPRegisterOffset(OFFSET_R0, heapTopPtr);
            break;
        case SVC_PMAP:
            putsUart0("Name\t\t\tAddress\t\t\tSize\t\t\tStack/Heap\n");
            uint8_t i;
            for(i = 0; i < taskCount; i++)
            {
                putsUart0(tcb[i].name);
                putsUart0("\t\t\t");
                putxUart0((uint32_t *)tcb[i].spInit);
                putsUart0("\t\t\t");
                putiUart0(srdToSize(tcb[i].srd));
                putsUart0("\t\t\t");
                putsUart0("Stack");
                putcUart0('\n');
            }
            break;
    }
}

// REQUIRED: code this function
void mpuFaultIsr()
{
    putsUart0("MPU fault in process ");
    putiUart0((int32_t)tcb[taskCurrent].pid);
    putsUart0("\n");
    // Print PSP, MSP
    uint32_t *PSP = getPSP();
    putsUart0("R0:\t");
    putxUart0(*PSP);        // R0
    putsUart0("\nR1:\t");
    putxUart0(*(PSP+1));      // R1
    putsUart0("\nR2:\t");
    putxUart0(*(PSP+2));    // R2
    putsUart0("\nR3:\t");
    putxUart0(*(PSP+3));    // R3
    putsUart0("\nR12:\t");
    putxUart0(*(PSP+4));    // R12
    putsUart0("\nLR (Offending Instruction Address):\t");
    putxUart0(*(PSP+5));    // LR
    putsUart0("\nPC:\t");
    putxUart0(*(PSP+6));    // PC
    putsUart0("\nxPSR:\t");
    putxUart0(*(PSP+7));    // xPSR
    putsUart0("\n");

    uint32_t *MSP = (uint32_t*)getMSP();
    putsUart0("MSP: ");
    putxUart0(*MSP);
    putsUart0("\n");
    putsUart0("MFault flags:\t\t");
    putxUart0(NVIC_FAULT_STAT_R & 0xF); // Prints mfault flags bits 7-0(NVIC_FAULT_STAT_R[7:0])
    putsUart0("\n");

    // MMARV bit of MFAULTSTAT is set, when this field holds addr of mem managament fault
    if(NVIC_FAULT_STAT_R & NVIC_FAULT_STAT_MMARV)
    {
        putsUart0("Address in memory that caused mpu fault:\t");
        putxUart0(NVIC_MM_ADDR_R);
        putsUart0("\n");
    }
    // BFARV means a bus fault has occured
    if(NVIC_FAULT_STAT_R & NVIC_FAULT_STAT_BFARV)
    {
        putsUart0("bus fault addy:\t\t");
        putxUart0(NVIC_FAULT_ADDR_R);
        putsUart0("\n");
    }
    // Print process stack dump (xPSR, PC, LR, R0 - R3, R12);

    NVIC_SYS_HND_CTRL_R &= ~(NVIC_SYS_HND_CTRL_MEMP); // Clear MPU fault pending bit
    NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV; // Triggers pendsv fault
}

// REQUIRED: code this function
void hardFaultIsr()
{
    putsUart0("Hard fault in process ");
    putiUart0((int32_t)tcb[taskCurrent].pid);
    putsUart0("\n");
    // Print PSP, MSP, and hard fault flags
    putsUart0("PSP: ");
    putxUart0((uint32_t)getPSP());
    putsUart0("MSP: ");
    putxUart0((uint32_t)getMSP());
    putsUart0("\n");
    putsUart0("HFault flags: ");
    putxUart0(NVIC_HFAULT_STAT_R);
    while(1);
}

// REQUIRED: code this function
void busFaultIsr()
{
    putsUart0("Bus fault in process ");
    putiUart0((int32_t)tcb[taskCurrent].pid);
    putsUart0("\n");
    while(1);
}

// REQUIRED: code this function
void usageFaultIsr()
{
    uint32_t pid = 0;
    putsUart0("Usage fault in process ");
    putiUart0((int32_t)tcb[taskCurrent].pid);
    putsUart0("\n");
    while(1);
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
// REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
//           6 pushbuttons
void initHw()
{
    initSystemClockTo40Mhz();
    NVIC_SYS_HND_CTRL_R |= NVIC_SYS_HND_CTRL_USAGE | NVIC_SYS_HND_CTRL_MEM | NVIC_SYS_HND_CTRL_BUS;

    enablePort(PORTA);
    enablePort(PORTC);
    enablePort(PORTD);
    enablePort(PORTE);
    enablePort(PORTF);

    selectPinPushPullOutput(ORANGE_LED);
    selectPinPushPullOutput(RED_LED);
    selectPinPushPullOutput(GREEN_LED);
    selectPinPushPullOutput(YELLOW_LED);
    selectPinPushPullOutput(BLUE_LED);

    setPinCommitControl(PUSHBUTTON1);

    selectPinDigitalInput(PUSHBUTTON0);
    selectPinDigitalInput(PUSHBUTTON1);
    selectPinDigitalInput(PUSHBUTTON2);
    selectPinDigitalInput(PUSHBUTTON3);
    selectPinDigitalInput(PUSHBUTTON4);
    selectPinDigitalInput(PUSHBUTTON5);

    enablePinPullup(PUSHBUTTON0);
    enablePinPullup(PUSHBUTTON1);
    enablePinPullup(PUSHBUTTON2);
    enablePinPullup(PUSHBUTTON3);
    enablePinPullup(PUSHBUTTON4);
    enablePinPullup(PUSHBUTTON5);

    NVIC_ST_RELOAD_R =  39999; // 1ms
//    NVIC_ST_STCURRENT = 1;  // Clears current count
    NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE | NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_CLK_SRC;
}

// REQUIRED: add code to return a value from 0-63 indicating which of 6 PBs are pressed
uint8_t readPbs()
{
    uint8_t ret = 0x0;
    ret |= !getPinValue(PUSHBUTTON5);
    ret <<= 1;
    ret |= !getPinValue(PUSHBUTTON4);
    ret <<= 1;
    ret |= !getPinValue(PUSHBUTTON3);
    ret <<= 1;
    ret |= !getPinValue(PUSHBUTTON2);
    ret <<= 1;
    ret |= !getPinValue(PUSHBUTTON1);
    ret <<= 1;
    ret |= !getPinValue(PUSHBUTTON0);
    return ret;
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------

uint32_t srdToSize(uint32_t srd)
{
    uint32_t size = 0;
    // Finds number of 1's in srd mask
    srd &= ~0x1F;
    while(srd)
    {
        srd &= (srd - 1);
        size++;
    }
    return size * 1024;
}

// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
    while(true)
    {
        setPinValue(ORANGE_LED, 1);
        waitMicrosecond(1000);
        setPinValue(ORANGE_LED, 0);
        yield();
    }
}

void flash4Hz()
{
    while(true)
    {
        setPinValue(GREEN_LED, !getPinValue(GREEN_LED));
        sleep(125);
    }
}

void oneshot()
{
    while(true)
    {
        wait(flashReq);
        setPinValue(YELLOW_LED, 1);
        sleep(1000);
        setPinValue(YELLOW_LED, 0);
    }
}

void partOfLengthyFn()
{
    // represent some lengthy operation
    waitMicrosecond(990);
    // give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    uint8_t *p;

    // Example of allocating memory from stack
    // This will show up in the pmap command for this thread
    p = mallocFromHeap(1024);
    putxUart0(p);
    *p = 0;

    while(true)
    {
        wait(resource);
        for (i = 0; i < 5000; i++)
        {
            partOfLengthyFn();
        }
        setPinValue(RED_LED, !getPinValue(RED_LED));
        post(resource);
    }
}

void readKeys()
{
    uint8_t buttons;
    while(true)
    {
        wait(keyReleased);
        buttons = 0;
        while (buttons == 0)
        {
            buttons = readPbs();
            yield();
        }
        post(keyPressed);
        if ((buttons & 1) != 0)
        {
            setPinValue(YELLOW_LED, !getPinValue(YELLOW_LED));
            setPinValue(RED_LED, 1);
        }
        if ((buttons & 2) != 0)
        {
            post(flashReq);
            setPinValue(RED_LED, 0);
        }
        if ((buttons & 4) != 0)
        {
            restartThread(flash4Hz);
        }
        if ((buttons & 8) != 0)
        {
            stopThread(flash4Hz);
        }
        if ((buttons & 16) != 0)
        {
            setThreadPriority(lengthyFn, 4);
        }
        yield();
    }
}

void debounce()
{
    uint8_t count;
    while(true)
    {
        wait(keyPressed);
        count = 10;
        while (count != 0)
        {
            sleep(10);
            if (readPbs() == 0)
                count--;
            else
                count = 10;
        }
        post(keyReleased);
    }
}

void uncooperative()
{
    while(true)
    {
        while (readPbs() == 8)
        {
        }
        yield();
    }
}

void errant()
{
    uint32_t* p = (uint32_t*)0x20000000;
    while(true)
    {
        while (readPbs() == 32)
        {
            *p = 0;
        }
        yield();
    }
}

void important()
{
    while(true)
    {
        wait(resource);
        setPinValue(BLUE_LED, 1);
        sleep(1000);
        setPinValue(BLUE_LED, 0);
        post(resource);
    }
}

// REQUIRED: add processing for the shell commands through the UART here
void shell()
{
    while (true)
    {
        USER_DATA data;
        while(!kbhitUart0())
            yield();
        __asm(" SVC #7");
        yield();

//        getsUart0(&data);
//        parseFields(&data);
//        if(isCommand(&data, "reboot" , 0))
//        {
//            reboot();
//        }
//        else if(isCommand(&data, "ps" , 0))
//        {
//            ps();
//        }
//        else if(isCommand(&data, "ipcs" , 0))
//        {
//            ipcs();
//        }
//        else if(isCommand(&data, "kill" , 1))
//        {
//            if(!isFieldInteger(&data, 1))
//                continue;
//            kill(getFieldInteger(&data, 1));
//        }
//        else if(isCommand(&data, "pmap" , 1))
//        {
//            if(!isFieldInteger(&data, 1))
//                continue;
//            pmap(getFieldInteger(&data, 1));
//        }
//        else if(isCommand(&data, "preempt" , 1))
//        {
//            if(!isFieldString(&data, 1))
//                continue;
//            if(stringCompare(getFieldString(&data, 1), "on"))
//                preempt(ON);
//            else if(stringCompare(getFieldString(&data, 1), "off"))
//                preempt(OFF);
//
//
//        }
//        else if(isCommand(&data, "sched" , 1))
//        {
//            if(!isFieldString(&data, 1))
//                continue;
//            if(stringCompare(getFieldString(&data, 1), "prio"))
//                sched(PRIO);
//            else if(stringCompare(getFieldString(&data, 1), "rr"))
//                sched(RR);
//        }
//        else if(isCommand(&data, "pidof" , 1))
//        {
//            if(!isFieldString(&data, 1))
//                continue;
//            pidof(getFieldString(&data, 1));
//        }
//        else if(isCommand(&data, "run" , 1))
//        {
//            if(!isFieldString(&data, 1))
//                continue;
//            run(getFieldString(&data, 1));
//        }
    }

}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    bool ok;

    // Initialize hardware
    initHw();
    initUart0();
    initMpu();
    initRtos();

    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);

    // Power-up flash
    setPinValue(GREEN_LED, 1);
    waitMicrosecond(250000);
    setPinValue(GREEN_LED, 0);
    waitMicrosecond(250000);

    // Initialize semaphores
    createSemaphore(keyPressed, 1);
    createSemaphore(keyReleased, 0);
    createSemaphore(flashReq, 5);
    createSemaphore(resource, 1);

    // Add required idle process at lowest priority
    ok =  createThread(idle, "Idle", 7, 1024);

    // Add other processes
    ok &= createThread(lengthyFn, "LengthyFn", 6, 1024);
    ok &= createThread(flash4Hz, "Flash4Hz", 4, 1024);
    ok &= createThread(oneshot, "OneShot", 2, 1024);
    ok &= createThread(readKeys, "ReadKeys", 6, 1024);
    ok &= createThread(debounce, "Debounce", 6, 1024);
    ok &= createThread(important, "Important", 0, 1024);
    ok &= createThread(uncooperative, "Uncoop", 6, 1024);
    ok &= createThread(errant, "Errant", 6, 1024);
    ok &= createThread(shell, "Shell", 6, 2048);

    priority = true;
    preemption = false;
    // Start up RTOS
    if (ok)
        startRtos(); // never returns
    else
        setPinValue(RED_LED, 1);

    return 0;
}


