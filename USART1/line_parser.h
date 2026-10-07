#ifndef LINE_PARSER_H
#define LINE_PARSER_H

#include <stddef.h>
#include <stdint.h>
#include <limits.h>

/* A complete CRLF line, including a trailing NUL, fits this buffer.
 * Overlong input is discarded until LF, so a truncated suffix is never
 * interpreted as a new command. Caller consumes a completed line immediately.
 */
#define UART_LINE_CAPACITY 128U
typedef struct {
    char data[UART_LINE_CAPACITY];
    size_t length;
    uint8_t dropping;
} UartLineParser;

static inline void UartLineParser_Reset(UartLineParser *parser)
{
    parser->length = 0U;
    parser->dropping = 0U;
    parser->data[0] = '\0';
}

static inline uint8_t UartLineParser_Push(UartLineParser *parser, uint8_t byte)
{
    if (parser->dropping != 0U) {
        if (byte == '\n') UartLineParser_Reset(parser);
        return 0U;
    }
    if (byte == '\n') {
        if ((parser->length != 0U) &&
            (parser->data[parser->length - 1U] == '\r')) {
            parser->data[--parser->length] = '\0';
            return 1U;
        }
        UartLineParser_Reset(parser);
        return 0U;
    }
    if ((byte == 0U) || (parser->length >= UART_LINE_CAPACITY - 1U)) {
        parser->dropping = 1U;
        return 0U;
    }
    parser->data[parser->length++] = (char)byte;
    parser->data[parser->length] = '\0';
    return 0U;
}

static inline uint8_t UartLineParser_Int(const char **cursor, int *value)
{
    const char *text = *cursor;
    uint32_t magnitude = 0U;
    uint8_t negative = 0U;
    uint32_t limit;
    if ((*text == '-') || (*text == '+')) {
        negative = (uint8_t)(*text == '-');
        text++;
    }
    if ((*text < '0') || (*text > '9')) return 0U;
    limit = (uint32_t)INT_MAX + (uint32_t)negative;
    while ((*text >= '0') && (*text <= '9')) {
        uint32_t digit = (uint32_t)(*text - '0');
        if (magnitude > (limit - digit) / 10U) return 0U;
        magnitude = magnitude * 10U + digit;
        text++;
    }
    *value = negative ? -(int)(magnitude - (magnitude != 0U)) -
                           (int)(magnitude != 0U) : (int)magnitude;
    *cursor = text;
    return 1U;
}

static inline uint8_t UartLineParser_ParseAttitude(const char *text,
                                                 int *roll, int *pitch, int *yaw)
{
    int parsed_roll, parsed_pitch, parsed_yaw;
    if (!UartLineParser_Int(&text, &parsed_roll) || *text++ != ',' ||
        !UartLineParser_Int(&text, &parsed_pitch) || *text++ != ',' ||
        !UartLineParser_Int(&text, &parsed_yaw) || *text != '\0') return 0U;
    *roll = parsed_roll;
    *pitch = parsed_pitch;
    *yaw = parsed_yaw;
    return 1U;
}

#endif
