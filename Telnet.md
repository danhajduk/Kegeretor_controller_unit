
# ANSI Escape Codes for Colors and Text Formatting

## Text Colors:
- **Black:** `\033[30m`
- **Red:** `\033[31m`
- **Green:** `\033[32m`
- **Yellow:** `\033[33m`
- **Blue:** `\033[34m`
- **Magenta:** `\033[35m`
- **Cyan:** `\033[36m`
- **White:** `\033[37m`

## Background Colors:
- **Black Background:** `\033[40m`
- **Red Background:** `\033[41m`
- **Green Background:** `\033[42m`
- **Yellow Background:** `\033[43m`
- **Blue Background:** `\033[44m`
- **Magenta Background:** `\033[45m`
- **Cyan Background:** `\033[46m`
- **White Background:** `\033[47m`

## Text Styles:
- **Bold On:** `\033[1m`
- **Bold Off:** `\033[22m`
- **Underline On:** `\033[4m`
- **Underline Off:** `\033[24m`
- **Blink On:** `\033[5m`
- **Blink Off:** `\033[25m`
- **Reverse (swap text and background colors):** `\033[7m`
- **Reverse Off:** `\033[27m`

## Reset Formatting:
- **Reset all styles and colors:** `\033[0m`

## Cursor Manipulation:
- **Cursor Up (n lines):** `\033[{n}A`
- **Cursor Down (n lines):** `\033[{n}B`
- **Cursor Forward (n columns):** `\033[{n}C`
- **Cursor Backward (n columns):** `\033[{n}D`
- **Save Cursor Position:** `\033[s`
- **Restore Cursor Position:** `\033[u`
- **Move Cursor to Home Position:** `\033[H`
- **Move Cursor to (row, col):** `\033[{row};{col}H`

## Screen Manipulation:
- **Clear Screen:** `\033[2J`
- **Clear Screen (from cursor down):** `\033[J`
- **Clear Line:** `\033[K`
- **Clear Line (from cursor right):** `\033[0K`
- **Clear Line (from cursor left):** `\033[1K`
- **Clear Entire Line:** `\033[2K`

## Advanced Colors (8-bit or 256 colors):
- **Foreground 256 Color:** `\033[38;5;{color_code}m`
- **Background 256 Color:** `\033[48;5;{color_code}m`

256-color codes range from 0 to 255, where:
- 0–15: Standard colors (as defined above, but more vivid)
- 16–231: 6×6×6 RGB cube (216 colors)
- 232–255: Grayscale from black to white (24 shades)

### Example:
- **Bright Red (Foreground):** `\033[38;5;9m`
- **Bright Green (Background):** `\033[48;5;10m`

## Example Usages:
```cpp
Serial.println("\033[31mThis is red text.\033[0m");
Serial.println("\033[1;34mThis is bold blue text.\033[0m");
Serial.println("\033[7;33;44mInverted yellow on blue text.\033[0m");
```

## Notes:
- ANSI escape codes work on most terminal emulators, including Telnet and SSH, but not all environments (e.g., some microcontroller serial monitors) may support all features.
- **Resetting** styles (`\033[0m`) is essential to prevent unintended formatting from carrying over to subsequent text.
