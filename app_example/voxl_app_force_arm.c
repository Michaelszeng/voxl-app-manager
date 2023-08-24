// /*******************************************************************************
//  * Copyright 2023 ModalAI Inc.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions are met:
//  *
//  * 1. Redistributions of source code must retain the above copyright notice,
//  *    this list of conditions and the following disclaimer.
//  *
//  * 2. Redistributions in binary form must reproduce the above copyright notice,
//  *    this list of conditions and the following disclaimer in the documentation
//  *    and/or other materials provided with the distribution.
//  *
//  * 3. Neither the name of the copyright holder nor the names of its contributors
//  *    may be used to endorse or promote products derived from this software
//  *    without specific prior written permission.
//  *
//  * 4. The Software is used solely in conjunction with devices provided by
//  *    ModalAI Inc.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  * POSSIBILITY OF SUCH DAMAGE.
//  ******************************************************************************/

// #include <stdio.h>
// #include <unistd.h>
// #include <pthread.h>
// #include <errno.h>
// #include <unistd.h>

// #include "voxl_app_template.h"
// #include "primitives.h"
// #include "queue.h"
// #include "app_runner.h"


// /**
//  * Flag that indicates whether this app is actively running or not.
//  * 
//  * Best practice to wrap all actions in this app in an if-ACTIVE_FLAG-statement
//  * so that this app doesn't continue trying to push to queue or expend resources
//  * when it doesn't have control of the drone anyway.
//  * 
//  * Note: this flag is automatically updated in the onUpdate() function.
//  */
// uint8_t ACTIVE_FLAG = 0;



// int onInit() {
//     printf("INITIALIZING...\n");
//     return 0;
// }

// int onStart(update_t update) {
//     printf("STARTING app voxl_app_force_arm.c.\n");

//     ACTIVE_FLAG = 1;

//     force_arm();

//     return 0;
// }

// void onUpdate(update_t update) {
    
// }

// void onException(uint64_t primitive_id, primitive_t primitive_type, event_t event_type, void* event_data) {
//     printf("EXCEPTION type %d occured with primitve ID %lu of type %d.\n", event_type, primitive_id, primitive_type);
// }

// void onStop() {
//     printf("STOPPING app voxl_app_force_arm.c\n");

//     ACTIVE_FLAG = 0;
// }

// void onDeinit() {
//     printf("DEINITIALIZING...\n");
// }
