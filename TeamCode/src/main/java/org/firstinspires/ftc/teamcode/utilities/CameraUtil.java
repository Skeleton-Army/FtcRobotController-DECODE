package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.config.BlackWhiteCamera;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class CameraUtil {

    public static void configureWebcam(OpenCvPipeline selectedPipeline, final HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(selectedPipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(BlackWhiteCamera.WIDTH, BlackWhiteCamera.HEIGHT, OpenCvCameraRotation.UPRIGHT, OpenCvWebcam.StreamFormat.MJPEG);
                //setupCamera(webcam.getExposureControl(), webcam.getFocusControl());
            }

            @Override
            public void onError(int errorCode)
            {
                //telemetry.addData("Webcam", "Error: " + errorCode);
            }
        });
    }
}
