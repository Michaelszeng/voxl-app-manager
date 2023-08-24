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


#ifndef PIPE_CHANNELS_H
#define PIPE_CHANNELS_H

/**
 * Consolidate some of the information about our pipe IO here
 */

#define PIPE_SERVER_NAME	"voxl-app-manager"  // client name used when connecting to servers
#define PIPE_CLIENT_NAME "voxl-app-manager"

// server channels
#define AVAILABLE_APPS_PIPE_NAME	"app_mgr_available_apps"
#define AVAILABLE_APPS_PIPE_LOCATION (MODAL_PIPE_DEFAULT_BASE_DIR AVAILABLE_APPS_PIPE_NAME "/")
#define AVAILABLE_APPS_CH 0

// client channels
#define MAVLINK_ONBOARD_CH		4	// voxl-mavlink-server
#define MAVLINK_ONBOARD_NAME	"mavlink_onboard"
#define MAVLINK_TO_GCS_CH		5	// voxl-mavlink-server
#define MAVLINK_TO_GCS_NAME		"mavlink_to_gcs"

#endif // end PIPE_CHANNELS_H
