#pragma once
#include "VRHelper.h"
#include "VRController.h"
#include "KinectSettings.h"
#include <iostream>

class PlayspaceMovementAdjuster {
public:
    PlayspaceMovementAdjuster( vrinputemulator::VRInputEmulator* vrinputEmulator )
		: lastLeftPosition(0,0,0),
        universeOrigin(0,0,0),
        lastRightPosition(0,0,0)
    {
		inputEmulator = vrinputEmulator;
		SetUniverseOrigin(sf::Vector3f(0, 0, 0), *inputEmulator, virtualDeviceIndexes);
    }
    ~PlayspaceMovementAdjuster() {

    }

    void update(VRcontroller& leftController, VRcontroller& rightController, std::vector<uint32_t> latestVirtualDeviceIndexes) {
        virtualDeviceIndexes = latestVirtualDeviceIndexes;
        playspaceMovementUpdate(leftController, rightController);
    }
    
    void resetPlayspaceAdjustments() {
		SetUniverseOrigin(sf::Vector3f(0, 0, 0), *inputEmulator, virtualDeviceIndexes);
    }

private:
    sf::Vector3f lastLeftPosition;
    sf::Vector3f lastRightPosition;
	sf::Vector3f universeOrigin;
	vrinputemulator::VRInputEmulator* inputEmulator;
    std::vector<uint32_t> virtualDeviceIndexes;

    bool poseIsUsable(vr::TrackedDevicePose_t pose) {
        return pose.bPoseIsValid && pose.bDeviceIsConnected;
    }
    void playspaceMovementUpdate(VRcontroller& leftController, VRcontroller& rightController) {

        vr::TrackedDevicePose_t leftPose = leftController.GetPose();
        sf::Vector3f positionDelta = sf::Vector3f(0, 0, 0);
		if (poseIsUsable(leftPose)) {
			vr::HmdMatrix34_t* leftMat = &(leftPose.mDeviceToAbsoluteTracking);

			vrinputemulator::DeviceOffsets info;
			inputEmulator->getDeviceOffsets(leftController.GetID(), info);
			sf::Vector3f leftPos = sf::Vector3f(leftMat->m[0][3], leftMat->m[1][3], leftMat->m[2][3]) + universeOrigin;
			if (KinectSettings::leftHandPlayspaceMovementButton && leftController.GetPress((vr::EVRButtonId)(KinectSettings::leftHandPlayspaceMovementButton - 1))) {
				positionDelta = leftPos - lastLeftPosition;
			}
			lastLeftPosition = leftPos;
		}

        vr::TrackedDevicePose_t rightPose = rightController.GetPose();
        if (poseIsUsable(rightPose)) {
			vr::HmdMatrix34_t* rightMat = &(rightPose.mDeviceToAbsoluteTracking);
			sf::Vector3f rightPos = sf::Vector3f(rightMat->m[0][3], rightMat->m[1][3], rightMat->m[2][3]) + universeOrigin;
			if (KinectSettings::rightHandPlayspaceMovementButton && rightController.GetPress((vr::EVRButtonId)(KinectSettings::rightHandPlayspaceMovementButton - 1))) {
				positionDelta = rightPos - lastRightPosition;
			}
            lastRightPosition = rightPos;
        }

		if (sqrt(positionDelta.x*positionDelta.x + positionDelta.y*positionDelta.y + positionDelta.z*positionDelta.z) != 0) {
			universeOrigin += positionDelta;
			SetUniverseOrigin(universeOrigin, *inputEmulator, virtualDeviceIndexes);
		}
    }
};