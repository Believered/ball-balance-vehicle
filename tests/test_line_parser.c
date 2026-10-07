#include "line_parser.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static int feed(UartLineParser *parser, const char *input)
{
    int frames = 0;
    while (*input) {
        if (UartLineParser_Push(parser, (uint8_t)*input++)) {
            frames++;
            UartLineParser_Reset(parser);
        }
    }
    return frames;
}

int main(void)
{
    UartLineParser parser;
    UartLineParser_Reset(&parser);
    assert(feed(&parser, "1,2,3\r\n4,5,6\r\n") == 2);
    assert(feed(&parser, "broken\nvalid\r\n") == 1);
    for (size_t i = 0; i < 400U; i++)
        assert(UartLineParser_Push(&parser, '9') == 0U);
    assert(UartLineParser_Push(&parser, '\r') == 0U);
    assert(UartLineParser_Push(&parser, '\n') == 0U);
    assert(feed(&parser, "recovered\r\n") == 1);
    for (size_t i = 0; i < UART_LINE_CAPACITY - 2U; i++)
        assert(UartLineParser_Push(&parser, 'x') == 0U);
    assert(UartLineParser_Push(&parser, '\r') == 0U);
    assert(UartLineParser_Push(&parser, '\n') == 1U);
    assert(strlen(parser.data) == UART_LINE_CAPACITY - 2U);
    UartLineParser_Reset(&parser);
    assert(UartLineParser_Push(&parser, 0U) == 0U);
    assert(feed(&parser, "ignored\r\n") == 0);
    assert(feed(&parser, "next\r\n") == 1);
    int roll = 10, pitch = 20, yaw = 30;
    assert(UartLineParser_ParseAttitude("-2147483648,+2147483647,0", &roll, &pitch, &yaw));
    assert(roll == INT_MIN && pitch == INT_MAX && yaw == 0);
    const char *invalid[] = {"2147483648,0,0", "-2147483649,0,0", "1,2,3x", "1,,3", "1,2", "1,2,3,4"};
    for (size_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); i++)
        assert(!UartLineParser_ParseAttitude(invalid[i], &roll, &pitch, &yaw));
    puts("PASS: CRLF, exact capacity, overflow discard, NUL and recovery");
    return 0;
}
