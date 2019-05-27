/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoCamera.WhiteBalance;
import edu.wpi.first.cameraserver.CameraServer;
import org.opencv.core.*;


public class Camera {

    // Declare cameras
    private UsbCamera cargoCamera;
    private UsbCamera hatchCamera;

    private CvSource outputStream;

    private CvSink cargoFrameGrabber;
    private CvSink hatchFrameGrabber;

    private static final int STANDARD_IMG_WIDTH = 160;
    private static final int STANDARD_IMG_HEIGHT = 120;

    private static final Camera instance = new Camera();
    public static Camera getInstance() {
        return instance;
    }

    public Camera() {
        CameraServer cameraServer = CameraServer.getInstance();

        //Initialize each camera with a channel and name, pushes non-processed images
        cargoCamera = cameraServer.startAutomaticCapture("Cargo Camera", 1);
        hatchCamera = cameraServer.startAutomaticCapture("Hatch Camera", 0);

        //Configure resoltuion, FPS, exposure, brightness and white-balance
        configureCamera(cargoCamera, false);
        configureCamera(hatchCamera, false);

        //Initialize frame grabber used to grab individual frames from video stream to be processed later
        cargoFrameGrabber = cameraServer.getVideo(cargoCamera);
        hatchFrameGrabber = cameraServer.getVideo(hatchCamera);

        //Push processed or unprocessed frames
        outputStream = cameraServer.putVideo("Processed Video", STANDARD_IMG_WIDTH, STANDARD_IMG_HEIGHT);

    }

    public void configureCamera(UsbCamera camera, boolean targetingCamera) {
        camera.setResolution(STANDARD_IMG_WIDTH, STANDARD_IMG_HEIGHT);
        camera.setFPS(15);
        if (targetingCamera) {
            camera.setExposureManual(5);
        } else {
            camera.setExposureAuto();
        }

        camera.setBrightness(40);
        camera.setWhiteBalanceManual(WhiteBalance.kFixedIndoor);
    }

    public void outputFrame(Mat currentFrame) {
        if (!currentFrame.empty()) {
            outputStream.putFrame(currentFrame);
        }
    }

}
