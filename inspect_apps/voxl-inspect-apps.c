/*******************************************************************************
 * Copyright 2023 ModalAI Inc.
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <getopt.h>
#include <string.h>
#include <modal_pipe.h>
#include <voxl_cutils.h>

#include "voxl_app_manager.h"


#define CLIENT_NAME         "voxl-inspect-apps"
#define SYS_STATUS_OUT_PATH	(MODAL_PIPE_DEFAULT_BASE_DIR "app_mgr_available_apps/")


static char en_newline = 0;


static void _print_usage()
{
    printf("\nCommand line arguments are as follows:\n\n");
    printf("-h, --help         : Print this help message\n");
    printf("-n, --new_line     : prints data on new lines instead of overwriting\n");
    printf("\nFor example: voxl-inspect-apps\n");
}


static int _parse_opts(int         argc,            ///< Number of arguments
		              char* const argv[])          ///< arguments
{
    static struct option LongOptions[] =
    {
        {"help",          no_argument,        0, 'h'},
        {"new_line",      no_argument,        0, 'n'},
    };

    int optionIndex      = 0;
    int status           = 0;
    int option;

    while ((status == 0) && (option = getopt_long (argc, argv, "hi:n", &LongOptions[0], &optionIndex)) != -1)
    {
        switch(option)
        {
            case 'h':
                status = -1;
                break;
            case 'n':
            	en_newline = 1;
            	break;
            case '?':  // Unknown argument
            default:
                printf("Invalid argument passed!\n");
                status = -1;
                break;
        }
    }

    if(status != 0) return status;

    return status;
}


static void _disconnect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context)
{
    fprintf(stderr, "\r" CLEAR_LINE FONT_BLINK "server disconnected" RESET_FONT);
    return;
}


static void _helper_cb(__attribute__((unused))int ch, char* data, int bytes, __attribute__((unused)) void* context)
{
    // in the case that the pipe reading process falls behind pipe writing, n_packets may be more than 1
    int n_packets;
    // length of this array is equal to n_packets
    available_apps_t* available_apps_arr = pipe_validate_available_apps_t(data, bytes, &n_packets);
    if (n_packets <= 0) {
        return;
    }

    for (int i=0; i<n_packets; i++)  {
        available_apps_t* available_apps = &available_apps_arr[i];
        
        // prints header
        if(!en_newline) printf(CLEAR_TERMINAL);
        printf(FONT_BOLD);

        printf("Number of Available Apps: %d\n", available_apps->num_apps);
        if (available_apps->active_app[0] < 0) {
            printf("No currently active app.\n");
        }
        else {
            printf("Active App ID: %d\n", available_apps->active_app[1]);
        }
        printf(RESET_FONT);
        printf("====================================\n");
        for (int j=0; j < available_apps->num_apps; j++) {
            printf("%d: %s\n", available_apps->apps[j].id, available_apps->apps[j].name);
        }

        if(en_newline) printf("\n");
        
        fflush(stdout);
    }

    return;
}


int main(int argc, char* const argv[]) {
    if (_parse_opts(argc, argv)) {
        _print_usage();
        return -1;
    }

    enable_signal_handler();
    main_running = 1;
    //disable terminal wrapping
    printf(DISABLE_WRAP);

    // set up all our MPA callbacks
    pipe_client_set_simple_helper_cb(0, _helper_cb, NULL);
    pipe_client_set_disconnect_cb(0, _disconnect_cb, NULL);

    printf("waiting for server at %s\r", SYS_STATUS_OUT_PATH);
    fflush(stdout);
    int ret = pipe_client_open(0, SYS_STATUS_OUT_PATH, CLIENT_NAME, \
                EN_PIPE_CLIENT_SIMPLE_HELPER, \
                AVAILABLE_APPS_RECOMMENDED_READ_BUF_SIZE);

    // check for errors trying to connect to the server pipe
    if (ret<0) {
        pipe_print_error(ret);
        printf(ENABLE_WRAP);
        return -1;
    }

    // keep going until the signal handler sets the running flag to 0
    while(main_running) usleep(500000);

    // all done, signal pipe read threads to stop
    printf("\nclosing and exiting\n" RESET_FONT ENABLE_WRAP);
    pipe_client_close_all();
}

