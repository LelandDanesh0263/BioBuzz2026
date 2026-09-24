package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

public class AprilTagPipeline {

    public static class AprilTagTarget {
        public int id = -1;
        public double rangeInches = 0.0;
        public double bearingDegrees = 0.0;
        public double yawDegrees = 0.0;
        public boolean detected = false;
    }

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    public void initAprilTag(HardwareMap hardwareMap) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .build();

        try {
            WebcamName webcamName = hardwareMap.get(WebcamName.class, RobotConstants.WEBCAM_NAME);
            visionPortal = new VisionPortal.Builder()
                    .setCamera(webcamName)
                    .addProcessor(aprilTagProcessor)
                    .build();
        } catch (Exception e) {
            visionPortal = null;
        }
    }

    public AprilTagTarget getAprilTagTarget(int targetId) {
        AprilTagTarget target = new AprilTagTarget();
        if (aprilTagProcessor == null) return target;

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        if (currentDetections == null) return target;

        for (AprilTagDetection detection : currentDetections) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;
                if (targetId == -1 || singleDet.id == targetId) {
                    target.id = singleDet.id;
                    if (singleDet.ftcPose != null) {
                        target.rangeInches = singleDet.ftcPose.range;
                        target.bearingDegrees = singleDet.ftcPose.bearing;
                        target.yawDegrees = singleDet.ftcPose.yaw;
                    }
                    target.detected = true;
                    break;
                }
            }
        }
        return target;
    }

    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
