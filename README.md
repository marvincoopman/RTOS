# **RTOS**
An implementation of a Real-time Operating System on an ARM-Cortex based Microcontroller. This RTOS supports preemption, semaphores, yielding, sleeping, priority scheduling, and a shell interface.

# **Commands**
>## reboot
>Reboots the RTOS

>## ps
>Prints out the each process' status including its name, PID, priority, state, and CPU %

>## ipcs
>Prints out the inter-process communication status including each semaphore's name, count, and the first process in queue for the resource



>## pmap \<PID>
>prints out a process's name, address, size, and type of memory allocation

>## preempt \<ON|OFF>
>Turns ON/OFF preemption

>## sched \<PRIO|RR>
>Sets the scheduler to a round-robin or priority-based

>## pidof \<PROCESS_NAME>
>Prints out the name and pid of a process

>## kill \<PID>
>Kills a process

>## run \<PROCESS_NAME>
>Restarts a killed process

>## help
>Prints out commands with descriptions
