/* Copyright (c) 2021-2023 Advanced Micro Devices, Inc.

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE. */

#ifndef ROCR_RDEBUG_H
#define ROCR_RDEBUG_H 1

#include <link.h> /* for struct r_debug */

/* ROCR r_debug::r_version history:

1: Initial debug protocol
2: New trap handler ABI. The reason for halting a wave is recorded in
   ttmp11[8:7].
3: New trap handler ABI. A wave halted at S_ENDPGM rewinds its PC by 8 bytes,
   and sets ttmp11[9]=1.
4: New trap handler ABI. Save the trap id in ttmp11[17:10].
5: New trap handler ABI. Save the PC in ttmp11[22:7] ttmp6[31:0], and park the
   wave if stopped.
6: New trap handler ABI. ttmp6[25:0] contains dispatch index modulo queue size
7: New trap handler ABI. Send interrupts as a bitmask, coalescing concurrent
   exceptions.
*/

#define ROCR_RDEBUG_VERSION 7

#endif /* ROCR_RDEBUG_H */
