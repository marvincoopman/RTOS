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

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "gpio.h"

// Pins
#define UART_TX PORTA,1
#define UART_RX PORTA,0

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize UART0
void initUart0(void)
{
    // Enable clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
    _delay_cycles(3);
    enablePort(PORTA);

    // Configure UART0 pins
    selectPinPushPullOutput(UART_TX);
    selectPinDigitalInput(UART_RX);
    setPinAuxFunction(UART_TX, GPIO_PCTL_PA1_U0TX);
    setPinAuxFunction(UART_RX, GPIO_PCTL_PA0_U0RX);

    // Configure UART0 with default baud rate
    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (usually 40 MHz)
}

// Set baud rate as function of instruction cycle frequency
void setUart0BaudRate(uint32_t baudRate, uint32_t fcyc)
{
    uint32_t divisorTimes128 = (fcyc * 8) / baudRate;   // calculate divisor (r) in units of 1/128,
    // where r = fcyc / 16 * baudRate
    divisorTimes128 += 1;                               // add 1/128 to allow rounding
    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART0_IBRD_R = divisorTimes128 >> 7;                // set integer value to floor(r)
    UART0_FBRD_R = ((divisorTimes128) >> 1) & 63;       // set fractional value to round(fract(r)*64)
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
    // turn-on UART0
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i = 0;
    while (str[i] != '\0')
        putcUart0(str[i++]);
}

void putiUart0(uint32_t num)
{
    if(num == 0)
    {
        putcUart0('0');
        return;
    }

    uint32_t factor = 1;
    uint32_t temp = num;
    char digit = '0';
    while(temp > 9)
    {
        temp /= 10;
        factor *= 10;
    }

    while(factor > 0)
    {

        digit = (num / factor) + '0';
        putcUart0(digit);
        num %= factor;
        factor /= 10;
    }
}

void putxUart0(uint32_t num)
{
    putcUart0('0');
    putcUart0('x');
    uint8_t i;
    uint8_t temp;
    for(i = 8; i != 0; i--)
    {
        temp = (num >> ((i-1) * 4)) & 0xF;
        switch(temp)
        {
            case 0xA:
                putcUart0('A');
                break;
            case 0xB:
                putcUart0('B');
                break;
            case 0xC:
                putcUart0('C');
                break;
            case 0xD:
                putcUart0('D');
                break;
            case 0xE:
                putcUart0('E');
                break;
            case 0xF:
                putcUart0('F');
                break;
            default:
                putcUart0(temp + '0');
                break;
        }
    }
}

void putpUart0(uint32_t num)
{
    if(num == 0)
    {
        putcUart0('0');
        return;
    }

    uint32_t factor = 1;
    uint32_t temp = num;
    char digit = '0';
    while(temp > 9)
    {
        temp /= 10;
        factor *= 10;
    }
    if(factor < 10)
    {
        putcUart0('.');
        if(factor == 1)
            putcUart0('0');
    }
    while(factor > 0)
    {
        if(factor == 10)
            putcUart0('.');
        digit = (num / factor) + '0';
        putcUart0(digit);
        num %= factor;
        factor /= 10;
    }
    putcUart0('%');
}
// Blocking function that returns with serial data once the buffer is not empty
char getcUart0(void)
{
//    while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    while(UART0_FR_R & UART_FR_RXFE)                       // if Uart0 rx fifo is empty --> yield
        __asm(" SVC #2");
    return UART0_DR_R & 0xFF;                        // get character from fifo
}


void getsUart0(USER_DATA *data)
{
    uint8_t count = 0;
    while(true)
    {
        data->buffer[count] = getcUart0();
        //Backspace pressed or unprintable character
        if(data->buffer[count] == 13 || count == MAX_CHARS)
        {
            data->buffer[count] = '\0';
            return;
        }
        else if(data->buffer[count] < 32 || data->buffer[count] == 127)
        {
            //ensures first character is not delimiter
            if(count == 0)
                continue;
            if(count > 0)
                count -= 2;
        }
        //Enter key pressed or max characters

        count++;
    }
}

void parseFields(USER_DATA *data)
{
    uint8_t index = 0;
    bool wasPreviousDelimiter = true;
    data->fieldCount = 0;
    while(data->buffer[index] != '\0')
    {
        if(wasPreviousDelimiter)
        {
            if(isAlpha(data->buffer[index]))
            {
                data->fieldType[data->fieldCount] = 'a';
                data->fieldPosition[data->fieldCount] = index;
                data->fieldCount++;
                wasPreviousDelimiter = false;
            }
            else if(isDigit(data->buffer[index]))
            {
                data->fieldType[data->fieldCount] = 'n';
                data->fieldPosition[data->fieldCount] = index;
                data->fieldCount++;
                wasPreviousDelimiter = false;
            }
        }
        else if(data->buffer[index] == '.')
        {
            if(isDigit(data->buffer[index - 1]))
                data->fieldType[data->fieldCount - 1] = 'f';
        }

            //delimiter
        else if(!isAlpha(data->buffer[index]) && !isDigit(data->buffer[index]))
        {

            //delimiters turn to NULL characters
            data->buffer[index] = '\0';
            wasPreviousDelimiter = true;
        }
        index++;
    }
}
// Returns the status of the receive buffer
bool kbhitUart0(void)
{
    return !(UART0_FR_R & UART_FR_RXFE);
}

//includes upper and lowercase
bool isAlpha(char alpha)
{
    if((alpha >= 'a' && alpha <= 'z') || (alpha >= 'A' && alpha <= 'Z'))
        return true;

    return false;
}

//includes '.' for floats and '-' for negatives
bool isDigit(char digit)
{
    if((digit >= '0' && digit <= '9') || digit == '.' || digit == '-')
        return true;

    return false;
}

//includes addition, subtraction, multiplication, and division
bool isMathOperator(char operator)
{
    if(operator == '+' || operator == '-' || operator == '/' || operator == '*')
        return true;

    return false;
}

int32_t stringToInt(char* numbers)
{
    int32_t sum = 0;
    int8_t offset = 0;
    while(numbers[offset] != '\0')
    {
        if(isDigit(numbers[offset]))
        {
            if(isMathOperator(numbers[offset]))
            {
                switch(numbers[offset])
                {
                    case '+':
                        sum += stringToInt(&numbers[++offset]);
                        break;
                    case '-':
                        sum -= stringToInt(&numbers[++offset]);
                        break;
                    case '/':
                        sum /= stringToInt(&numbers[++offset]);
                        break;
                    case '*':
                        sum *= stringToInt(&numbers[++offset]);
                        break;
                }
                break;
            }
            //converts ascii numeric to real number
            sum *= 10;
            sum += numbers[offset] - '0';

        }
        offset++;
    }
    return sum;
}

float stringToFloat(char* numbers)
{
    float integerSum = 0;
    int8_t offset = 0;
    float fractionalSum = 0;
    float fractionalFactor = 0.1;
    while(numbers[offset] != '\0' && numbers[offset] != '.')
    {
        if(isDigit(numbers[offset]))
        {
            if(isMathOperator(numbers[offset]))
            {
                switch(numbers[offset])
                {
                    case '+':
                        integerSum += stringToFloat(&numbers[++offset]);
                        break;
                    case '-':
                        integerSum -= stringToFloat(&numbers[++offset]);
                        break;
                    case '/':
                        fractionalSum += integerSum;
                        integerSum /= stringToFloat(&numbers[++offset]);
                        break;
                    case '*':
                        fractionalSum += integerSum;
                        integerSum *= stringToFloat(&numbers[++offset]);
                        break;
                }
                break;
            }
            //converts ascii numeric to real number
            integerSum *= 10;
            integerSum += (float)(numbers[offset] - '0');
        }
        offset++;
    }
    if(numbers[offset] == '.')
    {
        offset++;
        while(numbers[offset] != '\0')
        {
            if(isDigit(numbers[offset]))
            {
                if(isMathOperator(numbers[offset]))
                {
                    switch(numbers[offset])
                    {
                        case '+':
                            fractionalSum += stringToFloat(&numbers[++offset]);
                            break;
                        case '-':
                            fractionalSum -= stringToFloat(&numbers[++offset]);
                            break;
                        case '/':
                            fractionalSum += integerSum;
                            integerSum = 0;
                            fractionalSum /= stringToFloat(&numbers[++offset]);
                            break;
                        case '*':
                            fractionalSum += integerSum;
                            integerSum = 0;
                            fractionalSum *= stringToFloat(&numbers[++offset]);
                            break;
                    }
                    break;
                }
                fractionalSum += (float)(numbers[offset] - '0') * fractionalFactor;
                fractionalFactor *= 0.1;
            }
            offset++;
        }
    }
    return integerSum + fractionalSum;
}

bool isFieldString(USER_DATA* data, uint8_t fieldNumber)
{
    if(fieldNumber > data->fieldCount)
        return false;

    if(data->fieldType[fieldNumber] != 'a')
        return false;

    return true;
}
bool isFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
    if(fieldNumber > data->fieldCount)
        return false;

    if(data->fieldType[fieldNumber] != 'n')
        return false;

    return true;
}
bool isFieldFloat(USER_DATA* data, uint8_t fieldNumber)
{
    if(fieldNumber > data->fieldCount)
        return false;

    if(data->fieldType[fieldNumber] != 'f')
        return false;

    return true;
}

char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{
    return &data->buffer[data->fieldPosition[fieldNumber]];
}

int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
    return stringToInt(&data->buffer[data->fieldPosition[fieldNumber]]);
}

float getFieldFloat(USER_DATA* data, uint8_t fieldNumber)
{
    return stringToFloat(&data->buffer[data->fieldPosition[fieldNumber]]);
}

//command is always lowercase
bool isCommand(USER_DATA* data, const char command[], uint8_t minArguments)
{
    if(data->fieldCount - 1< minArguments)
        return false;
    uint8_t index = 0;
    while(true)
    {
        //data->buffer[data->fieldPosition[0] + index] accounts for initial offset of first command
        //in case a delimiter is the first character
        if(command[index] != data->buffer[index] && command[index] != data->buffer[index] + 32)
            return false;

        if(command[index] == '\0' && data->buffer[index] == '\0')
            return true;

        index++;
    }

}
//command is always lowercase
bool stringCompare(const char input[], const char command[])
{
    uint8_t index = 0;
    while(true)
    {
        if(command[index] != input[index] && command[index] != input[index] + 32)
            return false;

        if(command[index] == '\0' && input[index] == '\0')
            return true;

        index++;
    }
}
