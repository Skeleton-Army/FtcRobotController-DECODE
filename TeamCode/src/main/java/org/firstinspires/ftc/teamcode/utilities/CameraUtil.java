package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.skeletonarmy.marrow.OpModeManager;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.config.BlackWhiteCamera;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

public class CameraUtil {

    static OpenCvWebcam webcam;
    public static void configureWebcam(OpenCvPipeline selectedPipeline, final HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(selectedPipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(BlackWhiteCamera.WIDTH, BlackWhiteCamera.HEIGHT, OpenCvCameraRotation.UPRIGHT, OpenCvWebcam.StreamFormat.MJPEG);
                //setupCamera(webcam.getExposureControl(), webcam.getFocusControl())
                /*webcam.getExposureControl().setMode(ExposureControl.Mode.Manual);
                webcam.getExposureControl().setExposure(2, TimeUnit.MILLISECONDS);

                webcam.getGainControl().setGain(250);*/
            }

            @Override
            public void onError(int errorCode)
            {
                //telemetry.addData("Webcam", "Error: " + errorCode);
            }
        });
    }

    public static double getLatencyCamera() {
        return webcam.getTotalFrameTimeMs();
    }

    public static double getFPSCamera() {
        return webcam.getFps();
    }
    public static void printStats() {
        OpModeManager.getTelemetry().addData("Camera FPS", webcam.getFps());
        OpModeManager.getTelemetry().addData("Camera Latency/ms", webcam.getTotalFrameTimeMs());
    }
}
