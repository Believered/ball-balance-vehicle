#!/usr/bin/env bash
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
parser_binary=$(mktemp /tmp/ball-parser-XXXXXX)
pid_binary=$(mktemp /tmp/ball-pid-XXXXXX)
trap 'rm -f "$parser_binary" "$pid_binary"' EXIT
flags=(-std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined)
gcc "${flags[@]}" -I"$root/USART1" "$root/tests/test_line_parser.c" -o "$parser_binary"
"$parser_binary"
gcc "${flags[@]}" -I"$root/tests/stubs" -I"$root/MOTER" -I"$root/PID" \
    "$root/PID/pid.c" "$root/tests/test_pid.c" -lm -o "$pid_binary"
"$pid_binary"
