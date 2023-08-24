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
#include <stdlib.h> // for system()
#include <unistd.h>		// for access()
#include <modal_json.h>
// #include <voxl_common_config.h>

#include "config_file.h"
#include "voxl_app_manager.h"



#define FILE_HEADER "\
/**\n\
 * VOXL App Manager Configuration File\n\
 * \n\
 * version: don't touch this, used to keep track of changing config file formats\n\
 * \n\
 * app_1_name:\n\
 *         Provide the name of the app that you would like to assign to position 1\n\
 *         (when the switch is pulled toward your body) of SWC on your RC\n\
 *         transmitter (i.e. 'libvoxl_app_template.so').\n\
 * \n\
 * app_2_name:\n\
 *         Provide the name of the app that you would like to assign to position 2\n\
 *         (the middle position) of SWC on your RC transmitter (i.e. \n\
 *         'libvoxl_app_template.so').\n\
 * \n\
 * app_3_name:\n\
 *         Provide the name of the app that you would like to assign to position 3\n\
 *         (when the switch is pulled away from your body) of SWC on your RC\n\
 *         transmitter (i.e. 'libvoxl_app_template.so').\n\
 */\n"


// config file version detection
int config_file_version = 0;

char app1_name[MAX_APP_NAME_LENGTH];
char app2_name[MAX_APP_NAME_LENGTH];
char app3_name[MAX_APP_NAME_LENGTH];


int config_file_print(void)
{
	printf("=================================================================");
	printf("\n");
	printf("Parameters as loaded from config file:\n");
	printf("config_file_version:        %d\n", config_file_version);
	printf("\n");
	printf("app_1_name:                 %s\n", app1_name);
    printf("app_2_name:                 %s\n", app2_name);
    printf("app_3_name:                 %s\n", app3_name);
	printf("=================================================================");
	printf("\n");
	return 0;
}

int config_file_load(void)
{
    int ret;

    // check if the file exists and make a new one if not
	ret = json_make_empty_file_with_header_if_missing(VOXL_APP_MANAGER_CONF_FILE,\
																		FILE_HEADER);
    if(ret < 0) return -1;
	else if(ret>0) fprintf(stderr, "Created new json file: %s\n", VOXL_APP_MANAGER_CONF_FILE);

    // read the data in
	cJSON* parent = json_read_file(VOXL_APP_MANAGER_CONF_FILE);
	if(parent==NULL) return -1;
	cJSON* item;

    // check config file version
	json_fetch_int_with_default(    parent, "config_file_version", &config_file_version, 1);

    // app names assigned to each position of the switch
    json_fetch_string_with_default( parent, "app_1_name", app1_name, MAX_APP_NAME_LENGTH-1, "NONE");
    json_fetch_string_with_default( parent, "app_2_name", app2_name, MAX_APP_NAME_LENGTH-1, "NONE");
    json_fetch_string_with_default( parent, "app_3_name", app3_name, MAX_APP_NAME_LENGTH-1, "NONE");

    // write modified data to disk if neccessary
	if(json_get_modified_flag()){
		printf("The JSON config file data was modified during parsing, saving the changes to disk\n");
		json_write_to_file_with_header(VOXL_APP_MANAGER_CONF_FILE, parent, FILE_HEADER);
	}

    cJSON_Delete(parent);

    // TODO: validate config file data if necessary
}