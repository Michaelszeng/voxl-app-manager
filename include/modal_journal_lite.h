/*******************************************************************************
 * Copyright 2022 ModalAI Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * 4. The Software is used solely in conjunction with devices provided by
 *    ModalAI Inc.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/**
 * This is a "Lite" verison of libmodal_journal. 
 * 
 * https://gitlab.com/voxl-public/voxl-sdk/core-libs/libmodal-journal/-/blob/master/src/modal_journal.c?ref_type=heads
 * 
 * This is used instead of libmodal_journal because libmodal_journal is not
 * compatible with applications that use multi-processing. (Most likely the
 * writing to and reading from files is causing file descriptor problems)
 * 
 * This "Lite" version maintains the debug levels but doesn't save the logs to
 * a file.
 */


#ifndef MODAL_JOURNAL_LITE_H
#define MODAL_JOURNAL_LITE_H

#include <errno.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// Different debug levels
typedef enum M_JournalLevel
{
    VERBOSE = 0,        ///< Log everything
    DEBUG,              ///< Enable >= debug logs
    WARNING,            ///< Enable >= warning logs (default)
    ERROR,              ///< Enable >= error logs
    PRINT               ///< Enable only print logs
} M_JournalLevel;

// ---------------------------------------------------------------------------------
// Function to set the current debug level
// ---------------------------------------------------------------------------------
void M_JournalSetLevel(M_JournalLevel level);

// ---------------------------------------------------------------------------------
// Function to print the debug messages conditionally depending on the debug level
// ---------------------------------------------------------------------------------
void M_JournalPrint(M_JournalLevel level, const char * format, ...);

#define M_VERBOSE(x...) M_JournalPrint(VERBOSE, x)
#define   M_DEBUG(x...) M_JournalPrint(DEBUG,   x)
#define    M_WARN(x...) M_JournalPrint(WARNING, x)
#define   M_ERROR(x...) M_JournalPrint(ERROR,   x)
#define   M_PRINT(x...) M_JournalPrint(PRINT,   x)

// ---------------------------------------------------------------------------------
// perror wrapper
// ---------------------------------------------------------------------------------
#define M_perror_lvl(lvl, str) M_JournalPrint(lvl,   "%s: %s\n", str, strerror(errno))
#define M_perror(str)          M_JournalPrint(ERROR, "%s: %s\n", str, strerror(errno))

#ifdef __cplusplus
}
#endif

#endif
