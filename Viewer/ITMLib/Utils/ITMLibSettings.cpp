// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMLibSettings.h"

#include <stdio.h>

using namespace ITMLib::Objects;

/// Size of a voxel, usually given in meters.
//float voxelSize;
//
///** @{ */
///** \brief
//    Fallback parameters: consider only parts of the
//    scene from @p viewFrustum_min in front of the camera
//    to a distance of @p viewFrustum_max. Usually the
//    actual depth range should be determined
//    automatically by a ITMLib::Engine::ITMVisualisationEngine.
//*/
//float viewFrustum_min, viewFrustum_max;
//
///** @} */
///** \brief
//    Encodes the width of the band of the truncated
//    signed distance transform that is actually stored
//    in the volume. This is again usually specified in
//    meters. The resulting width in voxels is @ref mu
//    divided by @ref voxelSize.
//*/
//float mu;
//
///** \brief
//    Up to @ref maxW observations per voxel are averaged.
//    Beyond that a sliding average is computed.
//*/
//int maxW;
//
///** Stop integration once maxW has been reached. */
//bool stopIntegratingAtMaxW;


ITMLibSettings::ITMLibSettings(void)
	//: sceneParams(0.02f, 100, 0.005f, 0.2f, 3.0f, false)
    : sceneParams(0.008f, 100, 0.00125f, 0.06f, 2.0f, true)

{
    // 0.02f,         100,          0.005f,                  0.2f,                  3.0f,                  false
    // float mu, int maxW, float voxelSize, float viewFrustum_min, float viewFrustum_max, bool stopIntegratingAtMaxW
    
	/// depth threashold for the ICP tracker
	//depthTrackerICPThreshold = 0.1f * 0.1f;
    depthTrackerICPThreshold = 0.1f * 0.07f;

	/// For ITMDepthTracker: ICP iteration termination threshold
	//depthTrackerTerminationThreshold = 1e-3f;
    depthTrackerTerminationThreshold = 0.0015;
    
    // 0.001

	/// skips every other point when using the colour tracker
	skipPoints = true;

#ifndef COMPILE_WITHOUT_CUDA
	deviceType = DEVICE_CUDA;
#else
#ifdef COMPILE_WITH_METAL
	deviceType = DEVICE_METAL;
#else
	deviceType = DEVICE_CPU;
#endif
#endif

	//deviceType = DEVICE_CPU;

	/// enables or disables swapping. HERE BE DRAGONS: It should work, but requires more testing
	useSwapping = false;

	/// enables or disables approximate raycast
	useApproximateRaycast = false;

	/// enable or disable bilateral depth filtering;
	useBilateralFilter = true;

	//trackerType = TRACKER_COLOR;
	trackerType = TRACKER_ICP;
	//trackerType = TRACKER_REN;
	//trackerType = TRACKER_IMU;
	//trackerType = TRACKER_WICP;

	/// model the sensor noise as  the weight for weighted ICP
	modelSensorNoise = false;
	if (trackerType == TRACKER_WICP) modelSensorNoise = true;
	

	// builds the tracking regime. level 0 is full resolution
	if (trackerType == TRACKER_IMU)
	{
		noHierarchyLevels = 2;
		trackingRegime = new TrackerIterationType[noHierarchyLevels];

		trackingRegime[0] = TRACKER_ITERATION_BOTH;
		trackingRegime[1] = TRACKER_ITERATION_TRANSLATION;
	    //trackingRegime[2] = TRACKER_ITERATION_TRANSLATION;
	}
	else
	{
		noHierarchyLevels = 5;
		trackingRegime = new TrackerIterationType[noHierarchyLevels];

		trackingRegime[0] = TRACKER_ITERATION_BOTH;
		trackingRegime[1] = TRACKER_ITERATION_BOTH;
		trackingRegime[2] = TRACKER_ITERATION_ROTATION;
		trackingRegime[3] = TRACKER_ITERATION_ROTATION;
		trackingRegime[4] = TRACKER_ITERATION_ROTATION;
	}

	if (trackerType == TRACKER_REN) noICPRunTillLevel = 1;
	else noICPRunTillLevel = 0;

	if ((trackerType == TRACKER_COLOR) && (!ITMVoxel::hasColorInformation)) {
		printf("Error: Color tracker requires a voxel type with color information!\n");
	}
}

ITMLibSettings::~ITMLibSettings()
{
	delete[] trackingRegime;
}
