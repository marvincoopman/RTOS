// UART0 Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#ifndef UART0_H_
#define UART0_H_

#include <stdint.h>
#include <stdbool.h>

#define MAX_CHARS 80
#define MAX_FIELDS 5

typedef struct _USER_DATA
{
    char buffer[MAX_CHARS+1];
    char shellOutput[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS]; //n = number a = alpha f = float
    uint32_t value;
    uint32_t time;
    uint8_t savedIndex;
} USER_DATA;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initUart0(void);
void setUart0BaudRate(uint32_t baudRate, uint32_t fcyc);

void putcUart0(char c);
void putsUart0(char* str);
void putiUart0(uint32_t num);
void putxUart0(uint32_t num);
void putpUart0(uint32_t num);
char getcUart0(void);
void getsUart0(USER_DATA *data);
void parseFields(USER_DATA *data);
bool kbhitUart0(void);

bool isAlpha(char alpha);
bool isDigit(char digit);
bool isMathOperator(char operator);

int32_t stringToInt(char* stringInt);
float stringToFloat(char* numbers);

bool isFieldString(USER_DATA* data, uint8_t fieldNumber);
bool isFieldInteger(USER_DATA* data, uint8_t fieldNumber);
bool isFieldFloat(USER_DATA* data, uint8_t fieldNumber);
char* getFieldString(USER_DATA* data, uint8_t fieldNumber);
int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber);
float getFieldFloat(USER_DATA* data, uint8_t fieldNumber);

bool isCommand(USER_DATA* data, const char command[], uint8_t minArguments);
bool stringCompare(const char* command, const char* input);

#endif

