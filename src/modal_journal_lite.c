/*******************************************************************************
 * Copyright 2020 ModalAI Inc.
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


#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <sys/stat.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <voxl_cutils.h>
#include "modal_journal_lite.h"


static M_JournalLevel userDebugLevel    = WARNING;
static M_JournalLevel fsDebugLevel      = WARNING;
#define currentDebugLevel (userDebugLevel > fsDebugLevel ? fsDebugLevel : userDebugLevel)
static pthread_mutex_t print_mutex = PTHREAD_MUTEX_INITIALIZER;

static void print_level(M_JournalLevel level, FILE *stream);




void M_JournalSetLevel(M_JournalLevel level)
{
    userDebugLevel = level;
}


void M_JournalPrint(M_JournalLevel level, const char * format, ...)
{
    // Only print for the current debug level
    if (level < currentDebugLevel) return;
    pthread_mutex_lock(&print_mutex);
    FILE* stream;
    switch (level) {
        case ERROR:
        case WARNING:
            stream = stderr;
            break;
        default:
            stream = stdout;
            break;
    }
    va_list args;
    va_start(args, format);
    print_level(level, stream);
    vfprintf(stream, format, args);
    fflush(stream);
    va_end(args);
    pthread_mutex_unlock(&print_mutex);
}


static void print_level(M_JournalLevel level, FILE *stream)
{
    switch (level) {
        case VERBOSE:
            fprintf(stream, FONT_BOLD COLOR_GRN     "VERBOSE: " RESET_FONT);
            break;
        case DEBUG:
            fprintf(stream, FONT_BOLD COLOR_LIT_BLU "DEBUG:   " RESET_FONT);
            break;
        case WARNING:
            fprintf(stream, FONT_BOLD COLOR_YLW     "WARNING: " RESET_FONT);
            break;
        case ERROR:
            fprintf(stream, FONT_BOLD COLOR_RED     "ERROR:   " RESET_FONT);
            break;
        default:
            //Do Nothing
            break;
    }
}