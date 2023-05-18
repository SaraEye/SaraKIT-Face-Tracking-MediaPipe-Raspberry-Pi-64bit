/*
Copyright (c) 2018, Raspberry Pi (Trading) Ltd.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * \file RaspiCommonSettings.c
 *
 * Description
 *
 * Handles general settings applicable to all the camera applications
 */

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <memory.h>
#include <unistd.h>
#include <errno.h>
#include <sysexits.h>

#include "interface/vcos/vcos.h"
#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/mmal_parameters_camera.h"

#include "RaspiCommonSettings.h"
#include "RaspiHelpers.h"


void raspicommonsettings_set_defaults(RASPICOMMONSETTINGS_PARAMETERS *state)
{
   strncpy(state->camera_name, "(Unknown)", MMAL_PARAMETER_CAMERA_INFO_MAX_STR_LEN);
   // We dont set width and height since these will be specific to the app being built.
   state->width = 0;
   state->height = 0;
   state->filename = NULL;
   state->verbose = 0;
   state->cameraNum = 0;
   state->sensor_mode = 0;
   state->gps = 0;
};

/**
 * Dump parameters as human readable to stderr
 *
 * @param state Pointer to parameter block
 *
 */
void raspicommonsettings_dump_parameters(RASPICOMMONSETTINGS_PARAMETERS *state)
{
   fprintf(stderr, "Camera Name %s\n", state->camera_name);
   fprintf(stderr, "Width %d, Height %d, filename %s\n", state->width,
           state->height, state->filename);
   fprintf(stderr, "Using camera %d, sensor mode %d\n\n", state->cameraNum, state->sensor_mode);
   fprintf(stderr, "GPS output %s\n\n", state->gps ? "Enabled" : "Disabled");
};

