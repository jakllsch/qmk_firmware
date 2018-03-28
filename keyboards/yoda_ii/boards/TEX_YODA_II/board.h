/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef BOARD_H
#define BOARD_H

/*
 * Setup for TEX Yoda II board.
 */

/*
 * Board identifier.
 */
#define BOARD_TEX_YODA_II
#define BOARD_NAME                  "TEX Yoda II"

#define HT32F1654

#define LINE_TPPWR                  PAL_LINE(IOPORTA, 10)
#define LINE_BLPWM                  PAL_LINE(IOPORTA, 14)
#define LINE_BLEN                   PAL_LINE(IOPORTA, 15)
#define LINE_PB0                    PAL_LINE(IOPORTB, 0)
#define LINE_LED65                  PAL_LINE(IOPORTB, 1)
#define LINE_NORCE                  PAL_LINE(IOPORTB, 10)
#define LINE_TPREQ                  PAL_LINE(IOPORTC, 11)

#define LINE_COL0                   PAL_LINE(IOPORTC, 4)
#define LINE_COL1                   PAL_LINE(IOPORTB, 6)
#define LINE_COL2                   PAL_LINE(IOPORTD, 0)
#define LINE_COL3                   PAL_LINE(IOPORTB, 11)
#define LINE_COL4                   PAL_LINE(IOPORTA, 11)
#define LINE_COL5                   PAL_LINE(IOPORTA, 0)
#define LINE_COL6                   PAL_LINE(IOPORTA, 1)
#define LINE_COL7                   PAL_LINE(IOPORTA, 2)
#define LINE_COL8                   PAL_LINE(IOPORTA, 3)
#define LINE_COL9                   PAL_LINE(IOPORTC, 3)

#define LINE_ROW0                   PAL_LINE(IOPORTC, 12)
#define LINE_ROW1                   PAL_LINE(IOPORTA, 5)
#define LINE_ROW2                   PAL_LINE(IOPORTA, 6)
#define LINE_ROW3                   PAL_LINE(IOPORTA, 7)
#define LINE_ROW4                   PAL_LINE(IOPORTA, 4)
#define LINE_ROW5                   PAL_LINE(IOPORTC, 13)
#define LINE_ROW6                   PAL_LINE(IOPORTC, 14)
#define LINE_ROW7                   PAL_LINE(IOPORTC, 15)


#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* BOARD_H */
