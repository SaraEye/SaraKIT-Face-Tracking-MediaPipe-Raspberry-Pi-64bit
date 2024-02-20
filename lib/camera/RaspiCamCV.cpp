#include "RaspiCamCV.hpp"

#include <memory.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

extern cv::VideoCapture capcv0, capcv1;
extern RaspiCamCvCapture *caprp0, *caprp1;

// new
#include "bcm_host.h"
#include "interface/vcos/vcos.h"
#include "time.h"

extern "C" {
#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_parameters_camera.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
}

#include <semaphore.h>

#include "../../struct.hpp"
#include "RaspiCamControl.h"

/// Camera number to use - we only have one camera, indexed from 0.

// Standard port setting for the camera component
#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

// Video format information
#define VIDEO_FRAME_RATE_NUM 30
#define VIDEO_FRAME_RATE_DEN 1

/// Video render needs at least 2 buffers.
#define VIDEO_OUTPUT_BUFFERS_NUM 3

// Max bitrate we allow for recording
const int MAX_BITRATE = 30000000;  // 30Mbits/s

int mmal_status_to_int(MMAL_STATUS_T status);

/** Structure containing all state information for the current run
 */
typedef struct _RASPIVID_STATE {
    int finished;
    uint32_t width;   /// Requested width of image
    uint32_t height;  /// requested height of image
    int bitrate;      /// Requested bitrate
    int framerate;    /// Requested frame rate (fps)
    int monochrome;   /// Capture in gray only (2x faster)
    int buffered;
    // int immutableInput;     /// Flag to specify whether encoder works in place or creates a new buffer. Result is preview can display either
    /// the camera output or the encoder output (with compression artifacts)
    int camnum;
    int newframe;
    int hflip;
    int vflip;
    RASPICAM_CAMERA_PARAMETERS camera_parameters;  /// Camera setup parameters

    MMAL_COMPONENT_T *camera_component;   /// Pointer to the camera component
    MMAL_COMPONENT_T *encoder_component;  /// Pointer to the encoder component

    MMAL_POOL_T *video_pool;  /// Pointer to the pool of buffers used by encoder output port

    cv::Mat dstImageBuf, dstImage, dstImageHalf, dstImageGray, dstImageGrayHalf, dstImageGrayHalfEdge;

    VCOS_SEMAPHORE_T capture_sem0, capture_sem1;
    VCOS_SEMAPHORE_T capture_done_sem0, capture_done_sem1;

} RASPIVID_STATE;

static void default_status(RASPIVID_STATE *state) {
    if (!state) {
        vcos_assert(0);
        return;
    }

    // Default everything to zero
    memset(state, 0, sizeof(RASPIVID_STATE));

    // Now set anything non-zero
    state->finished = 0;
    state->width = 640;         // use a multiple of 320 (640, 1280)
    state->height = 480;        // use a multiple of 240 (480, 960)
    state->bitrate = 17000000;  // This is a decent default bitrate for 1080p
    state->framerate = VIDEO_FRAME_RATE_NUM;
    // state->immutableInput 	= 1;
    state->monochrome = 0;  // Gray (1) much faster than color (0)
    state->buffered = 0;
    state->newframe = 0;
    state->hflip = 0;
    state->vflip = 0;

    // Set up the camera_parameters to default
    raspicamcontrol_set_defaults(&state->camera_parameters);
}

/**
 *  buffer header callback function for video
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void video_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
    MMAL_BUFFER_HEADER_T *new_buffer;
    RASPIVID_STATE *state = (RASPIVID_STATE *)port->userdata;

    if (state) {
        if (state->finished) {
			if (state->camnum==0) {			
				vcos_semaphore_post(&state->capture_done_sem0);
			}
			else {
				vcos_semaphore_post(&state->capture_done_sem1);
			}
            return;
        }
		if (buffer->length)
		{
            mmal_buffer_header_mem_lock(buffer);

            //
            // *** PR : OPEN CV Stuff here !
            //
            if (state->monochrome==0) {//cam0
                //rgb
                state->dstImageBuf.data = buffer->data;
                cvtColor(state->dstImageBuf, state->dstImage, cv::COLOR_YUV2BGR_I420, 3);
                //gray
                state->dstImageGray.data = buffer->data;
//                cv::resize(state->dstImageGray, state->dstImageGrayHalf, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
//                cv::Canny(state->dstImageGrayHalf, state->dstImageGrayHalfEdge, threshold_slider, threshold_slider * 3, 3);
            } else
            {   //cam1
                state->dstImageGray.data = buffer->data;
                cv::resize(state->dstImageGray, state->dstImageGrayHalf, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
//                memcpy(state->dstImageLastGray.data, buffer->data, state->width * state->height);
            }

			if (state->camnum==0) {
				if (state->buffered==0) {
					vcos_semaphore_post(&state->capture_done_sem0);
					vcos_semaphore_wait(&state->capture_sem0);
				}
			} else {
				if (state->buffered==0) {
					vcos_semaphore_post(&state->capture_done_sem1);
					vcos_semaphore_wait(&state->capture_sem1);
				}
			}
            state->newframe++;
			mmal_buffer_header_mem_unlock(buffer);
        } else {
            //	printf("buffer null cam%d\n",state->camnum);
            vcos_log_error("buffer null");
        }
    } else {
        //	printf("eceived a encoder buffer callback with no state cam%d\n",state->camnum);
        vcos_log_error("Received a encoder buffer callback with no state");
    }

    // release buffer back to the pool
    mmal_buffer_header_release(buffer);

    // and send one back to the port (if still open)
    if (port->is_enabled) {
        MMAL_STATUS_T status;

        new_buffer = mmal_queue_get(state->video_pool->queue);

        if (new_buffer) status = mmal_port_send_buffer(port, new_buffer);

        if (!new_buffer || status != MMAL_SUCCESS) vcos_log_error("Unable to return a buffer to the encoder port");
    }
}

/**
 * Create the camera component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return 0 if failed, pointer to component if successful
 *
 */
static MMAL_COMPONENT_T *create_camera_component(RASPIVID_STATE *state) {
    MMAL_COMPONENT_T *camera = 0;
    MMAL_ES_FORMAT_T *format;
    MMAL_PORT_T *preview_port = NULL, *video_port = NULL, *still_port = NULL;
    MMAL_STATUS_T status;

    /* Create the component */
    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);

    MMAL_PARAMETER_INT32_T camera_num = {{MMAL_PARAMETER_CAMERA_NUM, sizeof(camera_num)}, state->camnum};

    if (status != MMAL_SUCCESS) {
        vcos_log_error("Failed to create camera component");
        goto error;
    }

    status = mmal_port_parameter_set(camera->control, &camera_num.hdr);

    if (!camera->output_num) {
        vcos_log_error("Camera doesn't have output ports");
        goto error;
    }

    video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
    still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];

    //  set up the camera configuration
    {
        MMAL_PARAMETER_CAMERA_CONFIG_T cam_config = {{MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config)},
                                                     .max_stills_w = state->width,
                                                     .max_stills_h = state->height,
                                                     .stills_yuv422 = 0,
                                                     .one_shot_stills = 0,
                                                     .max_preview_video_w = state->width,
                                                     .max_preview_video_h = state->height,
                                                     .num_preview_video_frames = 1,
                                                     .stills_capture_circular_buffer_height = 0,
                                                     .fast_preview_resume = 1,
                                                     .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC};
        mmal_port_parameter_set(camera->control, &cam_config.hdr);
    }
    // Set the encode format on the video  port

    format = video_port->format;
    //	if (state->monochrome)
    {
        format->encoding = MMAL_ENCODING_I420;
        format->encoding_variant = MMAL_ENCODING_I420;
    }
    //	else
    //	{
    //		format->encoding =
    //			mmal_util_rgb_order_fixed(video_port) ? MMAL_ENCODING_BGR24 : MMAL_ENCODING_RGB24;
    //		format->encoding_variant = 0;
    //	}

    format->es->video.width = state->width;
    format->es->video.height = state->height;
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = state->width;
    format->es->video.crop.height = state->height;
    format->es->video.frame_rate.num = state->framerate;
    format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;

    status = mmal_port_format_commit(video_port);
    if (status) {
        vcos_log_error("camera video format couldn't be set");
        goto error;
    }

    // PR : plug the callback to the video port
    status = mmal_port_enable(video_port, video_buffer_callback);
    if (status) {
        vcos_log_error("camera video callback2 error");
        goto error;
    }

    // Ensure there are enough buffers to avoid dropping frames
    if (video_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM) video_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

    // Set the encode format on the still  port
    format = still_port->format;
    format->encoding = MMAL_ENCODING_OPAQUE;
    format->encoding_variant = MMAL_ENCODING_I420;
    format->es->video.width = state->width;
    format->es->video.height = state->height;
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = state->width;
    format->es->video.crop.height = state->height;
    format->es->video.frame_rate.num = 1;
    format->es->video.frame_rate.den = 1;

    status = mmal_port_format_commit(still_port);
    if (status) {
        vcos_log_error("camera still format couldn't be set");
        goto error;
    }

    // PR : create pool of message on video port
    MMAL_POOL_T *pool;
    video_port->buffer_size = video_port->buffer_size_recommended;
    video_port->buffer_num = video_port->buffer_num_recommended;
    pool = mmal_port_pool_create(video_port, video_port->buffer_num, video_port->buffer_size);
    if (!pool) {
        vcos_log_error("Failed to create buffer header pool for video output port");
    }
    state->video_pool = pool;

    /* Ensure there are enough buffers to avoid dropping frames */
    if (still_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM) still_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

    /* Enable component */
    status = mmal_component_enable(camera);

    if (status) {
        vcos_log_error("camera component couldn't be enabled");
        goto error;
    }

    raspicamcontrol_set_all_parameters(camera, &state->camera_parameters);

    state->camera_component = camera;
    if (state->vflip==1 || state->hflip==1)
        raspicamcontrol_set_flips(camera,state->hflip,state->vflip);
    return camera;

error:

    if (camera) mmal_component_destroy(camera);

    return 0;
}

/**
 * Destroy the camera component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_camera_component(RASPIVID_STATE *state) {
    if (state->camera_component) {
        mmal_component_destroy(state->camera_component);
        state->camera_component = NULL;
    }
}

/**
 * Destroy the encoder component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_encoder_component(RASPIVID_STATE *state) {
    // Get rid of any port buffers first
    if (state->video_pool) {
        mmal_port_pool_destroy(state->encoder_component->output[0], state->video_pool);
    }
}

/**
 * Connect two specific ports together
 *
 * @param output_port Pointer the output port
 * @param input_port Pointer the input port
 * @param Pointer to a mmal connection pointer, reassigned if function successful
 * @return Returns a MMAL_STATUS_T giving result of operation
 *
 */
static MMAL_STATUS_T connect_ports(MMAL_PORT_T *output_port, MMAL_PORT_T *input_port, MMAL_CONNECTION_T **connection) {
    MMAL_STATUS_T status;

    status = mmal_connection_create(connection, output_port, input_port, MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);

    if (status == MMAL_SUCCESS) {
        status = mmal_connection_enable(*connection);
        if (status != MMAL_SUCCESS) mmal_connection_destroy(*connection);
    }

    return status;
}

/**
 * Checks if specified port is valid and enabled, then disables it
 *
 * @param port  Pointer the port
 *
 */
static void check_disable_port(MMAL_PORT_T *port) {
    if (port && port->is_enabled) mmal_port_disable(port);
}

RaspiCamCvCapture *raspiCamCvCreateCameraCapture2(int index, RASPIVID_CONFIG *config) {
    RaspiCamCvCapture *capture = (RaspiCamCvCapture *)malloc(sizeof(RaspiCamCvCapture));
    // Our main data storage vessel..
    RASPIVID_STATE *state = (RASPIVID_STATE *)malloc(sizeof(RASPIVID_STATE));
    capture->pState = state;

    // dupa MMAL_STATUS_T status = -1;
    MMAL_STATUS_T status;
    MMAL_PORT_T *camera_video_port = NULL;
    MMAL_PORT_T *camera_still_port = NULL;

    bcm_host_init();

    default_status(state);

    if (config != NULL) {
        if (config->width != 0) state->width = config->width;
        if (config->height != 0) state->height = config->height;
        if (config->bitrate != 0) state->bitrate = config->bitrate;
        if (config->framerate != 0) state->framerate = config->framerate;
        if (config->monochrome != 0) state->monochrome = config->monochrome;
        if (config->buffered != 0) state->buffered = config->buffered;
        if (config->vflip != 0) state->vflip = config->vflip;
        if (config->hflip != 0) state->hflip = config->hflip;        
    }

    state->camnum = index;

    int w = state->width;
    int h = state->height;

    state->dstImageBuf = cv::Mat(h * 3 / 2, w, CV_8UC1);

    state->dstImage = cv::Mat(h * 3 / 2, w, CV_8UC1);
    state->dstImageHalf = cv::Mat(h/2, w/2, CV_8UC1);
    state->dstImageGray = cv::Mat(h, w, CV_8UC1);
    state->dstImageGrayHalf = cv::Mat(h/2, w/2, CV_8UC1);
    state->dstImageGrayHalfEdge = cv::Mat(h/2, w/2, CV_8UC1);

	if (state->camnum==0) {
		vcos_semaphore_create(&state->capture_sem0, "Capture-Sem0", 0);
		vcos_semaphore_create(&state->capture_done_sem0, "Capture-Done-Sem0", 0);
	} else {
		vcos_semaphore_create(&state->capture_sem1, "Capture-Sem1", 0);
		vcos_semaphore_create(&state->capture_done_sem1, "Capture-Done-Sem1", 0);
	}

	// create camera
    if (!create_camera_component(state)) {
        vcos_log_error("%s: Failed to create camera component", __func__);
        raspiCamCvReleaseCapture(&capture);
        return NULL;
    }

    camera_video_port = state->camera_component->output[MMAL_CAMERA_VIDEO_PORT];
    camera_still_port = state->camera_component->output[MMAL_CAMERA_CAPTURE_PORT];

    // assign data to use for callback
    camera_video_port->userdata = (struct MMAL_PORT_USERDATA_T *)state;

    // start capture
    if (mmal_port_parameter_set_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS) {
        vcos_log_error("%s: Failed to start capture", __func__);
        raspiCamCvReleaseCapture(&capture);
        return NULL;
    }

    // Send all the buffers to the video port

    int num = mmal_queue_length(state->video_pool->queue);
    int q;
    for (q = 0; q < num; q++) {
        MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state->video_pool->queue);

        if (!buffer) vcos_log_error("Unable to get a required buffer %d from pool queue", q);

        if (mmal_port_send_buffer(camera_video_port, buffer) != MMAL_SUCCESS)
            vcos_log_error("Unable to send a buffer to encoder output port (%d)", q);
    }

    // mmal_status_to_int(status);

    // Disable all our ports that are not handled by connections
    // check_disable_port(camera_still_port);

	if (config->buffered != 0) {
		return capture;
	}

	//if (status != 0)
	//	raspicamcontrol_check_configuration(128);
	if (state->camnum==0) {
		vcos_semaphore_wait(&state->capture_done_sem0);
	} else {
		vcos_semaphore_wait(&state->capture_done_sem1);
	}
	return capture;
}

RaspiCamCvCapture *raspiCamCvCreateCameraCapture(int index) {
    return raspiCamCvCreateCameraCapture2(index, NULL);
}

void raspiCamCvReleaseCapture(RaspiCamCvCapture **capture) {
	RASPIVID_STATE * state = (*capture)->pState;

	// Unblock the callback.
	state->finished = 1;
	if (state->camnum==0) {
		vcos_semaphore_post(&state->capture_sem0);
		vcos_semaphore_wait(&state->capture_done_sem0);

		vcos_semaphore_delete(&state->capture_sem0);
		vcos_semaphore_delete(&state->capture_done_sem0);
	} else {
		vcos_semaphore_post(&state->capture_sem1);
		vcos_semaphore_wait(&state->capture_done_sem1);

		vcos_semaphore_delete(&state->capture_sem1);
		vcos_semaphore_delete(&state->capture_done_sem1);
	}

    if (state->camera_component) mmal_component_disable(state->camera_component);

    destroy_camera_component(state);

    state->dstImageBuf.release();

    state->dstImage.release();
    state->dstImageHalf.release();
    state->dstImageGray.release();    
    state->dstImageGrayHalf.release();
    state->dstImageGrayHalfEdge.release();

    free(state);
    free(*capture);
    *capture = 0;
}

cv::Mat raspiCamCvQueryFrame(RaspiCamCvCapture *capture) {
	RASPIVID_STATE * state = capture->pState;
	
	if (state->camnum==0) {
		if (state->buffered==0) {
			vcos_semaphore_post(&state->capture_sem0);
			vcos_semaphore_wait(&state->capture_done_sem0);
		}
	} else {
		if (state->buffered==0) {
			vcos_semaphore_post(&state->capture_sem1);
			vcos_semaphore_wait(&state->capture_done_sem1);
		}
	}

	return state->dstImage;
}

int init_camera(int camnum, int width, int height, bool gray, bool camcv, bool buffered, int hflip, int vflip) {
    camOpenCV=camcv;
    if (camnum == -1) {
        capcv0.open("video.mp4");
        return 0;
    }
    cv::Mat frameBig(960, 1280, CV_8UC3);
    if (camcv) {
        if (camnum == 0) {
            capcv0.open(0, cv::CAP_ANY);
            if (!capcv0.isOpened()) {
                printf("ERROR: Unable to open the camera");
                return -1;
            }
            capcv0.set(cv::CAP_PROP_FRAME_WIDTH, width );
            capcv0.set(cv::CAP_PROP_FRAME_HEIGHT, height );
            capcv0.set(cv::CAP_PROP_BUFFERSIZE, 1);
            capcv0.set(cv::CAP_PROP_MODE, 4);  // 7
            capcv0.set(cv::CAP_PROP_FPS, 42);  // 90
            // cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('H', '2', '6', '4'));
        } else {
            capcv1.open(1, cv::CAP_ANY);
            if (!capcv1.isOpened()) {
                printf("ERROR: Unable to open the camera");
                return -1;
            }
            capcv1.set(cv::CAP_PROP_FRAME_WIDTH, width);
            capcv1.set(cv::CAP_PROP_FRAME_HEIGHT, height);
            capcv1.set(cv::CAP_PROP_BUFFERSIZE, 1);
            capcv1.set(cv::CAP_PROP_MODE, 6);
            capcv1.set(cv::CAP_PROP_FPS, 42);
            // cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('H', '2', '6', '4'));
        }

    } else {
        if (camnum == 0) {
            RASPIVID_CONFIG *config = (RASPIVID_CONFIG *)malloc(sizeof(RASPIVID_CONFIG));
            config->width = width;
            config->height = height;
            config->bitrate = 0;  // zero: leave as default
            config->framerate = 42;
            config->monochrome = gray;
            config->buffered = buffered;
            config->vflip = vflip;
            config->hflip = hflip;
            caprp0 = (RaspiCamCvCapture *)raspiCamCvCreateCameraCapture2(0, config);
            free(config);
        } else {
            RASPIVID_CONFIG *config = (RASPIVID_CONFIG *)malloc(sizeof(RASPIVID_CONFIG));
            config->width = width;
            config->height = height;
            config->bitrate = 0;  // zero: leave as default
            config->framerate = 42;
            config->monochrome = gray;
            config->buffered = buffered;
            config->vflip = vflip;
            config->hflip = hflip;
            caprp1 = (RaspiCamCvCapture *)raspiCamCvCreateCameraCapture2(1, config);
            free(config);
        }
    }
    return 0;
}

int GetFrameFile(cv::Mat &frame0, cv::Mat &frame1) {
    cv::Mat fr;
    capcv0.read(fr);
    frame0 = fr(cv::Rect(0, 0, 640, 480));
    frame1 = fr(cv::Rect(640, 0, 640, 480));
    return 0;
}

int GetFrameCam0(cv::Mat &frame0, cv::Mat &frame0Gray, cv::Mat &frame0GrayHalf, cv::Mat &frame0GrayHalfEdge) {
    // RaspiVid
    frame0 = caprp0->pState->dstImage;
    frame0Gray = caprp0->pState->dstImageGray;
    frame0GrayHalf = caprp0->pState->dstImageGrayHalf;
    frame0GrayHalfEdge = caprp0->pState->dstImageGrayHalfEdge;
    int fr=caprp0->pState->newframe;
    caprp0->pState->newframe=0;
    return fr;
}

int GetFrameCam1(cv::Mat &frame1, cv::Mat &frame1Gray, cv::Mat &frame1GrayHalf) {
    // RaspiVid
    frame1 = caprp1->pState->dstImage;
    frame1Gray = caprp1->pState->dstImageGray;
    frame1GrayHalf = caprp1->pState->dstImageGrayHalf;
    frame1GrayHalfEdge = caprp1->pState->dstImageGrayHalfEdge;
    int fr=caprp1->pState->newframe;
    caprp1->pState->newframe=0;
    return fr;
}

int GetFrame()
{
    int res=0;
    if (viewStatus == ViewerStatus::Pause)
    {
        ispause = !ispause;
    }

    if (ispause)
    {
        if (camFile)
        {
            capcv0.set(cv::CAP_PROP_POS_FRAMES, cv::getTrackbarPos("Position", window_name));
            GetFrameFile(frame0, frame1);
            memcpy(imgcopy0.data, frame0.data, camwidth * camheight * 3);
            memcpy(imgcopy1.data, frame1.data, camwidth * camheight * 3);
        }
        else
        {
            frame0 = imgcopy0.clone();
            frame1 = imgcopy1.clone();
        }
        res=1;
    }
    else
    {
        if (camFile)
        {
            GetFrameFile(frame0, frame1);
            cv::setTrackbarPos("Position", window_name, int(capcv0.get(cv::CAP_PROP_POS_FRAMES)));
            res=1;
        }
        else
        {
            if (camOpenCV) {
                if (capcv0.isOpened()) {
                    capcv0.read(frame0);
                    //todo 
                    //cv::flip(frame,frame, 0);
                    res=1;
                }
                if (capcv1.isOpened()) {
                    capcv1.read(frame1);
                    res=res|2;
                }
                return res;
            } else {
                if (caprp0 != nullptr) 
                    res=GetFrameCam0(frame0,frame0Gray,frame0GrayHalf,frame0GrayHalfEdge);
                
                if (caprp1 != nullptr)
                    res=res|(GetFrameCam1(frame1,frame1Gray,frame1GrayHalf)<<1);
            }
        }
    }
    return res;
}
