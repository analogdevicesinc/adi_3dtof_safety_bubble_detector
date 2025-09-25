/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#ifndef MODULE_PROFILE_H
#define MODULE_PROFILE_H

#ifdef ENABLE_FUNCTION_PROFILING
extern volatile int * pprof_buff;
void dumpFunctionParams(int ID);
void initProfile(void);
void closeProfile(void);
void flushProfile(void);

#define PROFILE_FUNCTION_START(ID) dumpFunctionParams(ID);
#define PROFILE_FUNCTION_END(ID) PROFILE_FUNCTION_START(ID)

#define FLUSH_FUNCTION_PROFILE() flushProfile()
#define INIT_FUNCTION_PROFILE() initProfile()
#define CLOSE_FUNCTION_PROFILE() closeProfile()

#else /* !ENABLE_FUNCTION_PROFILING */
#define PROFILE_FUNCTION_START(ID)
#define PROFILE_FUNCTION_END(ID)

#define FLUSH_FUNCTION_PROFILE()
#define INIT_FUNCTION_PROFILE()
#define CLOSE_FUNCTION_PROFILE()
#endif /* !ENABLE_FUNCTION_PROFILING */

#ifdef PARSE_FUNCTION
#define PROFILE_ID_START()   \
  void profile_id_init(void) \
  {
#define PROFILE_ID_END() }
#define PROFILE_ID(id_name, id_num) profiles[id_num].func_name = #id_name;
#else
#define PROFILE_ID_START()
#define PROFILE_ID_END()
#define PROFILE_ID(id_name, id_num) static const int id_name = id_num;
#endif /* PARSE_FUNCTION */

/* ID list */
PROFILE_ID_START()

PROFILE_ID(FLOOR_DETECTOR_0, 0)
PROFILE_ID(FLOOR_DETECTOR_1_PREPROCESS, 1)
PROFILE_ID(FLOOR_DETECTOR_2_GET_NEXT_FRAME, 2)
PROFILE_ID(FLOOR_DETECTOR_3_COMPUTE_POINT_CLOUD, 3)
PROFILE_ID(FLOOR_DETECTOR_4_COMPRESS_DEPTH_IR, 4)
PROFILE_ID(FLOOR_DETECTOR_5_ROTATE_POINT_CLOUD, 5)
PROFILE_ID(FLOOR_DETECTOR_6_RUN, 6)
PROFILE_ID(FLOOR_DETECTOR_7_RANSAC_RUN, 7)
PROFILE_ID(FLOOR_DETECTOR_8_FALLBACK_RUN, 8)
PROFILE_ID(FLOOR_DETECTOR_9_POSTPROCESS, 9)

PROFILE_ID_END()

#endif /* MODULE_PROFILE_H */
