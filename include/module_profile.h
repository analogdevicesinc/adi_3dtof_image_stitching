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

PROFILE_ID(ImageStitch_RUN, 0)
PROFILE_ID(ImageStitch_PREPROCESS, 1)
PROFILE_ID(ImageStitch_CREATE_COMBINED_CLOUD, 2)
PROFILE_ID(ImageStitch_PCLALIGNMENT, 3)
PROFILE_ID(ImageStitch_ACCUMULATE_COMBINED_CLOUD, 4)
PROFILE_ID(ImageStitch_TRANSFORM_COMBINED_CLOUD, 5)
PROFILE_ID(ImageStitch_DISPLAY_POSTPROCESS, 6)
PROFILE_ID(ImageStitch_CYLINDRICALPROJECT_IR, 7)
PROFILE_ID(ImageStitch_CYLINDRICALPROJECT_DEPTH, 8)
PROFILE_ID(CylindricalProjection_Initialize, 9)
PROFILE_ID(CylindricalProjection_LoadCloud, 10)
PROFILE_ID(CylindricalProjection_MakeImage, 11)
PROFILE_ID(CylindricalProjection_ShowImage, 12)
PROFILE_ID(GPUProjectPoints, 13)
PROFILE_ID(processOutput_Thread, 14)
PROFILE_ID(StitchFramesCoreGPU_stitchFrames, 15)
PROFILE_ID(StitchFramesCoreGPU_preprocess, 16)
PROFILE_ID(StitchFramesCoreGPU_copyIn, 17)
PROFILE_ID(StitchFramesCoreGPU_postProcessStage1, 18)
PROFILE_ID(StitchFramesCoreGPU_postProcessStage2, 19)
PROFILE_ID(StitchFramesCoreGPU_copyOut, 20)
PROFILE_ID(Message_Callback_processing, 21)
PROFILE_ID(StitchFramesCoreCPU_stitchFrames, 22)
PROFILE_ID(StitchFramesCoreCPU_preprocess, 23)
PROFILE_ID(StitchFramesCoreCPU_copyIn, 24)
PROFILE_ID(StitchFramesCoreCPU_postProcessStage1, 25)
PROFILE_ID(StitchFramesCoreCPU_postProcessStage2, 26)
PROFILE_ID(StitchFramesCoreCPU_copyOut, 27)

PROFILE_ID_END()

#endif /* MODULE_PROFILE_H */
