package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.vision.BallVisionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class VisionSubsystem extends SubsystemBase {

    private OpenCvCamera camera;
    private final BallVisionPipeline ballPipeline;
    private final AprilTagPipeline aprilTagPipeline;

    private boolean cameraInitialized = false;

    public VisionSubsystem(HardwareMap hardwareMap) {
        ballPipeline = new BallVisionPipeline();
        aprilTagPipeline = new AprilTagPipeline();

        try {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            WebcamName webcamName = hardwareMap.get(WebcamName.class, RobotConstants.WEBCAM_NAME);

            camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
            camera.setPipeline(ballPipeline);

            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                    // Stream live video feed with OpenCV overlays directly to FTC Dashboard
                    FtcDashboard.getInstance().startCameraStream(camera, 30);
                    cameraInitialized = true;
                }

                @Override
                public void onError(int errorCode) {
                    cameraInitialized = false;
                }
            });

            aprilTagPipeline.initAprilTag(hardwareMap);
        } catch (Exception e) {
            cameraInitialized = false;
        }
    }

    public BallVisionPipeline.BallTarget getTargetBall() {
        return ballPipeline.getLatestTarget();
    }

    public AprilTagPipeline.AprilTagTarget getTargetAprilTag(int targetId) {
        return aprilTagPipeline.getAprilTagTarget(targetId);
    }

    public boolean isCameraInitialized() {
        return cameraInitialized;
    }
}
