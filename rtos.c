// RTOS Framework - Fall 2022
// J Losh

// Student Name:
// Marvin Coopman
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

#define BLUE_LED   PORTF,2 // on-board blue LED
#define RED_LED    PORTE,0 // off-board red LED
#define GREEN_LED  PORTA,4 // off-board green LED
#define YELLOW_LED PORTA,3 // off-board yellow LED
#define ORANGE_LED PORTB,6 // off-board orange LED
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

void updateMemoryBlocks(uint8_t task, uint32_t srd, int8_t type);
void freeMemoryBlocks(uint32_t srd);

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
typedef struct _semaphore
{
    uint16_t count;
    uint16_t queueSize;
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
    char name[16]; // store task index here
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
bool priority = true;
bool preemption = false;


// REQUIRED: add store and management for the memory used by the thread stacks
//           thread stacks must start on 1 kiB boundaries so mpu can work correctly

struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *spInit;                  // original top of stack
    void *sp;                      // current stack pointer
    int8_t priorityInit;           // Original priority
    int8_t priority;               // 0=highest to 15=lowest
    uint32_t ticks;                // ticks until sleep complete
    uint32_t srd;                  // MPU subregion disable bits (one per 1 KiB)
    char name[16];                 // name of task used in ps command
    void *semaphore;               // pointer to the semaphore that is blocking the thread
    int8_t hasSemaphore;           // Pointer to the semaphore process is using
} tcb[MAX_TASKS];

struct _memoryBlocks
{
    int8_t ownership;
    uint8_t allocationType;
}memoryBlocks[32];

uint32_t *heapBotPtr = 0x20002000;
#define SRAMTOPADDR 0x20008000
#define SRAMBOTADDR 0x20000000
#define ALLOCATION_EMPTY 0
#define ALLOCATION_STACK 1
#define ALLOCATION_HEAP 2

#define OFFSET_R0  0
#define SVC_SLEEP  1
#define SVC_YIELD  2
#define SVC_SYSTICK  3
#define SVC_WAIT  4
#define SVC_POST  5
#define SVC_MALLOC  6
#define SVC_SETPRIORITY  7
#define SVC_PMAP  8
#define SVC_PIDOF  9
#define SVC_IPCS  10
#define SVC_PREEMPT  11
#define SVC_PRIORITY  12
#define SVC_REBOOT  13
#define SVC_PS  14
#define SVC_STOP  17
#define SVC_RESTART  18



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

//    if(heapTopPtr - heapBotPtr < size_in_bytes)
//        return 0;

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
    mask |= 0x1F;
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
    for(i = 0; i < 32; i++)
    {
        memoryBlocks[i].ownership = -1;
        memoryBlocks[i].allocationType = ALLOCATION_EMPTY;
    }
}

// REQUIRED: Implement prioritization to 8 levels
int rtosScheduler()
{
//    WTIMER0_
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
            ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN) && tcb[task].priority == maxPriority;
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
            tcb[i].priorityInit = priority;
            tcb[i].priority = priority;
            tcb[i].srd = getSramSRD(tcb[i].sp, stackBytes); // Or'ed with 0x1F to stack in SRAM
            // Update global memoryBlock array
            updateMemoryBlocks(i, tcb[i].srd, ALLOCATION_STACK);
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
bool restartThread(char *func)
{
    __asm(" SVC #18");
}

// REQUIRED: modify this function to stop a thread
// REQUIRED: remove any pending semaphore waiting
// NOTE: see notes in class for strategies on whether stack is freed or not
bool stopThread(_fn fn)
{
    __asm(" SVC #17");
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
    __asm(" SVC #7");
}

bool createSemaphore(uint8_t semaphore, uint8_t count, const char name[])
{
    uint8_t j = 0;
    bool ok = (semaphore < MAX_SEMAPHORES);
    {
        semaphores[semaphore].count = count;
        while(name[j] != 0) // Strcpy
        {
            semaphores[semaphore].name[j] = name[j];
            j++;
        }
        semaphores[semaphore].name[j] = 0;
    }
    return ok;
}

// REQUIRED: modify this function to start the operating system
// by calling scheduler, setting PSP, ASP bit, TMPL bit, and PC
void startRtos()
{
    taskCurrent = rtosScheduler();
    uint32_t sp = (uint32_t)tcb[taskCurrent].sp;
    setPSP(sp);
    setASP();
    _fn fn = tcb[taskCurrent].pid;
    tcb[taskCurrent].state = STATE_READY;
    setSramAccessWindow(tcb[taskCurrent].srd);
    // Systick configuration
    NVIC_ST_RELOAD_R =  39999; // 1ms
    NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE | NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_CLK_SRC;
    removePriv();
    fn();
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
void yield()
{
    __asm(" SVC #2");
}

// REQUIRED: modify this function to support 1ms system timer
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

bool getData(uint8_t type, USER_DATA *data)
{
    switch(type)
    {
        case SVC_PMAP:
            __asm(" SVC #8");
            break;
        case SVC_PIDOF:
            __asm(" SVC #9");
            break;
        case SVC_IPCS:
            __asm(" SVC #10");
            break;
        case SVC_PREEMPT:
            __asm(" SVC #11");
            break;
        case SVC_PRIORITY:
            __asm(" SVC #12");
            break;
        case SVC_REBOOT:
            __asm(" SVC #13");
            break;
        case SVC_PS:
            __asm(" SVC #14");
            break;
    }

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
    static uint8_t i, j;
    uint32_t *psp = getPSP();
    uint16_t *pc = *(psp + 6);
    uint8_t value = (*(pc - 1) & 0xFF); // Extracts SVC Number
    switch(value) {
        case SVC_SLEEP:
        {
            tcb[taskCurrent].state = STATE_DELAYED;
            tcb[taskCurrent].ticks = *psp;  // Retrieves R0
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
            break;
        }
        case SVC_YIELD:
        {
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
            break;
        }
        case SVC_SYSTICK:
        {
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
            break;
        }
        case SVC_WAIT:
        {
            if (semaphores[*psp].count > 0)
            {
                semaphores[*psp].count--;
                tcb[taskCurrent].hasSemaphore = *psp;
            }
            else
            {
                if(semaphores[*psp].queueSize >= MAX_QUEUE_SIZE)
                    NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV; // Triggers pendsv fault

                semaphores[*psp].processQueue[semaphores[*psp].queueSize] = taskCurrent; // Add current task to Queue
                semaphores[*psp].queueSize++;
                tcb[taskCurrent].state = STATE_BLOCKED;
                tcb[taskCurrent].semaphore = (void *)&semaphores[*psp];
                NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV; // Triggers pendsv fault
            }
            break;
        }
        case SVC_POST:
        {
            tcb[taskCurrent].hasSemaphore = -1;
            semaphores[*psp].count++;
            if(semaphores[*psp].queueSize > 0)
            {
                tcb[semaphores[*psp].processQueue[0]].state = STATE_READY;
                tcb[semaphores[*psp].processQueue[0]].hasSemaphore = *psp;
                tcb[semaphores[*psp].processQueue[0]].semaphore = 0;
                semaphores[*psp].queueSize--;
                uint8_t i;
                for(i = 0; i < semaphores[*psp].queueSize; i++)
                {
                    semaphores[*psp].processQueue[i] = semaphores[*psp].processQueue[i + 1];
                }
                semaphores[*psp].count--;
            }
//            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV; // Triggers pendsv fault
            break;
        }
        case SVC_MALLOC:
        {
            if(*psp == 0)
                break;

            i = 32;
            j = ((*psp - 1) / 1024) + 1;
            bool ok = false;
            while(!ok)
            {
                i--;
                if(i == 5)
                    return;
                if(memoryBlocks[i].allocationType == ALLOCATION_EMPTY)
                {
                    j--;
                    if(j == 0)
                    {
                        ok = true;
                    }
                }
                else
                {
                    j = ((*psp - 1) / 1024) + 1;
                }
            }
            uint32_t heapTopPtr = (SRAMBOTADDR + (i * 1024));
            putxUart0(heapTopPtr);
            tcb[taskCurrent].srd |= getSramSRD(heapTopPtr, *psp);
            updateMemoryBlocks(taskCurrent, getSramSRD(heapTopPtr, *psp), ALLOCATION_HEAP);
            pushPSPRegisterOffset(OFFSET_R0, heapTopPtr);
            break;
        }
        case SVC_SETPRIORITY:
        {
            for(i = 0; i < taskCount; i++)
            {
                if((uint32_t)tcb[i].pid == *psp)
                {
                    tcb[i].priority = *(psp + 1);
                    break;
                }
            }
            break;
        }
        case SVC_PMAP:
        {
            // Function\t\t\tAddress\t\t\tSize\t\t\tStack/Heap
            USER_DATA *data = (USER_DATA *) *(psp + 1);
            uint8_t i = 0;
            bool ok = false;
            uint32_t requestedPid = getFieldInteger(data, 1);

            while(!ok && i < taskCount)
            {
                if ((uint32_t *) tcb[i].pid == requestedPid)
                    break;
                i++;
            }
            if(i == taskCount)
            {
                pushPSPRegisterOffset(OFFSET_R0, 1); // Pid not found
                break;
            }
            if(data->savedIndex == 0)
                data->savedIndex = 5;

            uint8_t j = 0;
            uint8_t allocationType = ALLOCATION_EMPTY;
            while(data->savedIndex < 32)
            {
                if(memoryBlocks[data->savedIndex].ownership == i)
                {
                    allocationType = memoryBlocks[data->savedIndex].allocationType;
                    j++;
                }
                else if(j != 0 || (allocationType != memoryBlocks[data->savedIndex].allocationType && j != 0))
                {
                    data->savedIndex++;
                    data->value = j;
                    if(allocationType == ALLOCATION_STACK)
                    {
                        data->shellOutput[0] = 'S';
                        data->shellOutput[1] = 't';
                        data->shellOutput[2] = 'a';
                        data->shellOutput[3] = 'c';
                        data->shellOutput[4] = 'k';
                        data->shellOutput[5] = 0;
                    }
                    else
                    {
                        data->shellOutput[0] = 'H';
                        data->shellOutput[1] = 'e';
                        data->shellOutput[2] = 'a';
                        data->shellOutput[3] = 'p';
                        data->shellOutput[4] = 0;
                    }
                    break;
                }
                data->savedIndex++;
            }
            if(j != 0 && data->savedIndex == 32)
            {
                data->value = j;
                if(allocationType == ALLOCATION_STACK)
                {
                    data->shellOutput[0] = 'S';
                    data->shellOutput[1] = 't';
                    data->shellOutput[2] = 'a';
                    data->shellOutput[3] = 'c';
                    data->shellOutput[4] = 'k';
                    data->shellOutput[5] = 0;
                }
                else
                {
                    data->shellOutput[0] = 'H';
                    data->shellOutput[1] = 'e';
                    data->shellOutput[2] = 'a';
                    data->shellOutput[3] = 'p';
                    data->shellOutput[4] = 0;
                }
                ok = true;
                data->savedIndex++;
            }
            ok = data->savedIndex >= 32;
            pushPSPRegisterOffset(OFFSET_R0, ok); // Hasnt traversed the entire memoryBlock
            break;
        }
        case SVC_PIDOF:
        {
            bool ok = false;
            USER_DATA *data = (USER_DATA *) *(psp + 1);
            uint8_t i = 0;
            while(!ok && i < taskCount)
            {
                if((ok = stringCompare(getFieldString(data, 1), tcb[i].name)))
                {
                    if(tcb[i].state == STATE_INVALID)
                        break;
                    data->value= (uint32_t)tcb[i].pid;
                    ok = true;
                    break;
                }
                i++;
            }
            pushPSPRegisterOffset(OFFSET_R0, ok); // Invalid name
            break;
        }
        case SVC_IPCS:
        {
            USER_DATA *data = (USER_DATA *) *(psp + 1);
            if(data->savedIndex== 0)
                data->savedIndex= 1;
            bool ok = false;
            uint8_t i = 0;
            uint8_t j = 0;

            // String copy "Name\t\t\tCount\t\t\tNext process waiting
            while(semaphores[data->savedIndex].name[i] != 0)
            {
                data->shellOutput[i] = semaphores[data->savedIndex].name[i];
                i++;
            }
            data->shellOutput[i++] = '\t';
            data->shellOutput[i++] = '\t';
            data->shellOutput[i++] = '\t';
            data->shellOutput[i++] = semaphores[data->savedIndex].count + '0';
            data->shellOutput[i++] = '\t';
            data->shellOutput[i++] = '\t';
            data->shellOutput[i++] = '\t';

            while(tcb[semaphores[data->savedIndex].processQueue[0]].name[j] != 0)
            {
                data->shellOutput[i++] = tcb[semaphores[data->savedIndex].processQueue[0]].name[j];
                j++;
            }

            data->shellOutput[i] = 0; // Null Character
            data->savedIndex++;
            if(data->savedIndex== MAX_SEMAPHORES)
                pushPSPRegisterOffset(OFFSET_R0, 1); // Done sending data
            else
                pushPSPRegisterOffset(OFFSET_R0, 0); // Not done sending data
            break;
        }
        case SVC_PREEMPT:
        {
            USER_DATA *data = (USER_DATA *) *(psp + 1);
            preemption = data->value ;
            break;
        }
        case SVC_PRIORITY:
        {
            USER_DATA *data = (USER_DATA *) *(psp + 1);
            priority = data->value ;
            break;
        }
        case SVC_REBOOT:
        {
            NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
        }
        case SVC_PS:
        {
            bool ok = false;
            USER_DATA *data = (USER_DATA *) *(psp + 1);
            uint8_t i = 0;
            while(!ok && i < taskCount)
            {
                if((ok = stringCompare(getFieldString(data, 1), tcb[i].name)))
                {
                    if(tcb[i].state == STATE_INVALID)
                        break;
                    data->value= (uint32_t)tcb[i].pid;
                    ok = true;
                    break;
                }
                i++;
            }
            pushPSPRegisterOffset(OFFSET_R0, ok); // Invalid name
            break;
        }
        case SVC_STOP:
        {
            bool ok = false;
            for(i = 0; i < taskCount; i++)
            {
                if ((uint32_t *) tcb[i].pid == *psp)
                {
                    if(tcb[i].state == STATE_BLOCKED)
                    {
                        // Remove process from process queue
                        for(j = 0; j < ((semaphore *)(tcb[i].semaphore))->queueSize; j++)
                        {
                            if (((semaphore *) (tcb[i].semaphore))->processQueue[j] == i)
                                ((semaphore *) (tcb[i].semaphore))->processQueue[j] = 0;

                            if(((semaphore *) (tcb[i].semaphore))->processQueue[j] == 0)
                            {
                                ((semaphore *) (tcb[i].semaphore))->processQueue[j] = ((semaphore *) (tcb[i].semaphore))->processQueue[j + 1];
                                ((semaphore *) (tcb[i].semaphore))->processQueue[j + 1] = 0;
                            }
                        }
                        tcb[i].semaphore = 0;
                        ((semaphore *)(tcb[i].semaphore))->queueSize--;
                    }
                    else if(tcb[i].state == STATE_DELAYED)
                        tcb[i].ticks = 0;
                    else if(tcb[i].hasSemaphore != -1)
                    {
                        // Post semaphore
                        semaphores[tcb[i].hasSemaphore].count++;
                        if(semaphores[tcb[i].hasSemaphore].queueSize > 0)
                        {
                            tcb[semaphores[tcb[i].hasSemaphore].processQueue[0]].state = STATE_READY;
                            tcb[semaphores[tcb[i].hasSemaphore].processQueue[0]].hasSemaphore = tcb[i].hasSemaphore;
                            tcb[semaphores[tcb[i].hasSemaphore].processQueue[0]].semaphore = 0;
                            semaphores[tcb[i].hasSemaphore].queueSize--;
                            uint8_t j;
                            for(j = 0; j < semaphores[tcb[i].hasSemaphore].queueSize; j++)
                            {
                                semaphores[tcb[i].hasSemaphore].processQueue[j] = semaphores[tcb[i].hasSemaphore].processQueue[j + 1];
                            }
                            semaphores[tcb[i].hasSemaphore].count--;
                        }
                        tcb[i].hasSemaphore = -1;
                    }
                    tcb[i].state = STATE_INVALID;
                    freeMemoryBlocks(tcb[i].srd);
                    ok = true;
                    NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV; // Context switch
                }
            }
            pushPSPRegisterOffset(OFFSET_R0, ok); // Restart successful
            break;
        }
        case SVC_RESTART:
        {
            char *func = *psp;
            bool ok = false;
            for(i = 0; i < taskCount; i++)
            {
                if(stringCompare(func, tcb[i].name))
                {
                    tcb[i].priority = tcb[i].priorityInit;
                    tcb[i].sp = tcb[i].spInit;
                    tcb[i].state = STATE_UNRUN;
                    ok = true;
                    NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV; // Context swtich
                }
            }
            pushPSPRegisterOffset(OFFSET_R0, ok); // Restart successful
            break;
        }
    }
}

void mpuFaultIsr()
{
    putsUart0("MPU fault in process ");
    putiUart0((int32_t)tcb[taskCurrent].pid);
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

    NVIC_SYS_HND_CTRL_R &= ~(NVIC_SYS_HND_CTRL_MEMP); // Clear MPU fault pending bit
    NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV; // Context swtich
}

void hardFaultIsr()
{
    putsUart0("Hard fault in process ");
    putiUart0((int32_t)tcb[taskCurrent].pid);
    putsUart0("\n");
    putsUart0("HFault flags: ");
    putxUart0(NVIC_HFAULT_STAT_R);
    while(1);
}

void busFaultIsr()
{
    putsUart0("Bus fault in process ");
    putiUart0((int32_t)tcb[taskCurrent].pid);
    putsUart0("\n");
    while(1);
}

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
    enablePort(PORTB);
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


//     Widetimer configuration

    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R0;
    WTIMER0_CTL_R &= ~(TIMER_CTL_TAEN | TIMER_CTL_TBEN);
    WTIMER0_CFG_R = TIMER_CFG_16_BIT;
    WTIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACMR | TIMER_TAMR_TACDIR;
    WTIMER0_TBMR_R = TIMER_TBMR_TBMR_PERIOD | TIMER_TBMR_TBCMR | TIMER_TBMR_TBCDIR;
    WTIMER0_CTL_R |= TIMER_CTL_TAEN | TIMER_CTL_TBEN;
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

void updateMemoryBlocks(uint8_t task, uint32_t srd, int8_t type)
{
    uint8_t i;
    for(i = 0; i < 32; i++)
    {
        if(srd & 1)
        {
            memoryBlocks[i].ownership = task;
            memoryBlocks[i].allocationType = type;
        }
        srd >>= 1;
    }
}

void freeMemoryBlocks(uint32_t srd)
{
    uint8_t i;
    for(i = 0; i < 32; i++)
    {
        if(srd & 1 && memoryBlocks[i].allocationType == ALLOCATION_HEAP)
        {
            memoryBlocks[i].ownership = -1;
            memoryBlocks[i].allocationType = ALLOCATION_EMPTY;
        }
        srd >>= 1;
    }
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
    USER_DATA data;
    bool ok = false;
    data.value = 0;
    data.savedIndex= 0;
    while (true)
    {
        getsUart0(&data);
        parseFields(&data);
        if(isCommand(&data, "reboot" , 0))
        {
            getData(SVC_REBOOT, &data);
        }
        else if(isCommand(&data, "ps" , 1))
        {
            if(!isFieldString(&data, 1))
                continue;

            if(!getData(SVC_PS, &data))
            {
                putsUart0("Invalid function name\n");
                continue;
            }
            putsUart0("Name\t\t\tPID\t\t\tCPU%%\n");
            putsUart0(getFieldString(&data, 1));
            putsUart0("\t\t\t");
            putiUart0(data.value);
            putsUart0("\t\t\t");
            putsUart0("Place Holder\n");

        }
        else if(isCommand(&data, "ipcs" , 0))
        {
            putsUart0("Name\t\t\tCount\t\t\tFirst in Queue\n");
            while(!ok)
            {
                ok = getData(SVC_IPCS, &data);
                putsUart0(data.shellOutput);
                putcUart0('\n');
            }
        }
        else if(isCommand(&data, "kill" , 1))
        {
            if(!isFieldInteger(&data, 1))
                continue;
            if(stopThread(getFieldInteger(&data, 1)))
            {
                putsUart0("Stopped ");
                putsUart0(getFieldString(&data, 1));
                putsUart0("\n");

            }
            else
                putsUart0("Invalid PID\n");
        }
        else if(isCommand(&data, "pmap" , 1))
        {
            if(!isFieldInteger(&data, 1))
                continue;

            putsUart0("Name\t\t\tAddress\t\t\tSize\t\t\tStack/Heap\n");
            while(!ok)
            {
                data.value = 0;
                ok = getData(SVC_PMAP, &data);
                if(data.value == 0)
                    continue;
                putsUart0("Name\t\t\t");
                putxUart0(SRAMBOTADDR + (((data.savedIndex - data.value) - 1) * 1024));
                putsUart0("\t\t\t");
                putiUart0(data.value * 1024);
                putsUart0("\t\t\t");
                putsUart0(data.shellOutput);
                putcUart0('\n');
            }
        }
        else if(isCommand(&data, "preempt" , 1))
        {
            if(!isFieldString(&data, 1))
                continue;
            if(stringCompare(getFieldString(&data, 1), "on"))
            {
                data.value = 1;
                getData(SVC_PREEMPT, &data);
                putsUart0("Preemption on");
            }
            else if(stringCompare(getFieldString(&data, 1), "off"))
            {
                data.value = 0;
                getData(SVC_PREEMPT, &data);
                putsUart0("Preemption off");
            }
        }
        else if(isCommand(&data, "sched" , 1))
        {
            if(!isFieldString(&data, 1))
                continue;
            if(stringCompare(getFieldString(&data, 1), "prio"))
            {
                data.value = 1;
                getData(SVC_PRIORITY, &data);
                putsUart0("Priority scheduling enabled\n");
            }
            else if(stringCompare(getFieldString(&data, 1), "rr"))
            {
                data.value = 0;
                getData(SVC_PRIORITY, &data);
                putsUart0("Priority scheduling disabled\n");
            }
        }
        else if(isCommand(&data, "pidof" , 1))
        {
            if(!isFieldString(&data, 1))
                continue;

            if(!getData(SVC_PIDOF, &data))
            {
                putsUart0("Invalid function name\n");
                continue;
            }
            putsUart0("Name\t\t\tPID\n");
            putsUart0(getFieldString(&data, 1));
            putsUart0("\t\t\t");
            putiUart0(data.value);
            putcUart0('\n');
        }
        else if(isCommand(&data, "run" , 1))
        {
            if(!isFieldString(&data, 1))
                continue;
            if(restartThread(getFieldString(&data, 1)))
            {
                putsUart0("Restarted ");
                putsUart0(getFieldString(&data, 1));
                putcUart0('\n');
            }
            else
                putsUart0("Invalid function name\n");
        }
        data.value = 0;
        data.savedIndex= 0;
        ok = false;
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
    createSemaphore(keyPressed, 1, "keypressed");
    createSemaphore(keyReleased, 0, "keyreleased");
    createSemaphore(flashReq, 5, "flashreq");
    createSemaphore(resource, 1, "resource");

    // Add required idle process at lowest priority
    ok =  createThread(idle, "idle", 7, 1024);

    // Add other processes
    ok &= createThread(lengthyFn, "lengthyfn", 6, 1024);
    ok &= createThread(flash4Hz, "flash4hz", 4, 1024);
    ok &= createThread(oneshot, "oneshot", 2, 1024);
    ok &= createThread(readKeys, "readkeys", 6, 1024);
    ok &= createThread(debounce, "debounce", 6, 1024);
    ok &= createThread(important, "important", 0, 1024);
    ok &= createThread(uncooperative, "uncoop", 6, 1024);
    ok &= createThread(errant, "errant", 6, 1024);
    ok &= createThread(shell, "shell", 6, 2048);

    // Start up RTOS
    if (ok)
        startRtos(); // never returns
    else
        setPinValue(RED_LED, 1);

    return 0;
}
