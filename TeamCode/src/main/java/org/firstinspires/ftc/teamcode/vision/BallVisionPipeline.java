package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BallVisionPipeline extends OpenCvPipeline {

    public enum BallType {
        NONE,
        NECTAR, // Small ball
        POLLEN  // Bigger ball
    }

    public static class BallTarget {
        public BallType type = BallType.NONE;
        public double centerX = 0;
        public double centerY = 0;
        public double radius = 0;
        public double offsetX = 0; // Relative to frame center (320)
        public double offsetY = 0; // Relative to frame center (240)
        public boolean detected = false;
    }

    private final Mat hsvMat = new Mat();
    private final Mat maskMat = new Mat();

    private volatile BallTarget latestTarget = new BallTarget();

    @Override
    public Mat processFrame(Mat input) {
        // Convert to HSV
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Threshold yellow/ball color
        Scalar lowerHsv = new Scalar(
                RobotConstants.HSV_YELLOW_LOWER_H,
                RobotConstants.HSV_YELLOW_LOWER_S,
                RobotConstants.HSV_YELLOW_LOWER_V
        );
        Scalar upperHsv = new Scalar(
                RobotConstants.HSV_YELLOW_UPPER_H,
                RobotConstants.HSV_YELLOW_UPPER_S,
                RobotConstants.HSV_YELLOW_UPPER_V
        );

        Core.inRange(hsvMat, lowerHsv, upperHsv, maskMat);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(maskMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        BallTarget bestTarget = new BallTarget();
        double maxRadius = 0;

        Point center = new Point();
        float[] radius = new float[1];

        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            Imgproc.minEnclosingCircle(contour2f, center, radius);

            double currentRadius = radius[0];

            if (currentRadius > maxRadius) {
                if (currentRadius >= RobotConstants.NECTAR_MIN_RADIUS_PX && currentRadius <= RobotConstants.NECTAR_MAX_RADIUS_PX) {
                    maxRadius = currentRadius;
                    bestTarget.type = BallType.NECTAR;
                    bestTarget.centerX = center.x;
                    bestTarget.centerY = center.y;
                    bestTarget.radius = currentRadius;
                    bestTarget.offsetX = center.x - (input.cols() / 2.0);
                    bestTarget.offsetY = center.y - (input.rows() / 2.0);
                    bestTarget.detected = true;
                } else if (currentRadius >= RobotConstants.POLLEN_MIN_RADIUS_PX && currentRadius <= RobotConstants.POLLEN_MAX_RADIUS_PX) {
                    maxRadius = currentRadius;
                    bestTarget.type = BallType.POLLEN;
                    bestTarget.centerX = center.x;
                    bestTarget.centerY = center.y;
                    bestTarget.radius = currentRadius;
                    bestTarget.offsetX = center.x - (input.cols() / 2.0);
                    bestTarget.offsetY = center.y - (input.rows() / 2.0);
                    bestTarget.detected = true;
                }
            }
            contour2f.release();
            contour.release();
        }

        hierarchy.release();

        if (bestTarget.detected) {
            Imgproc.circle(input, new Point(bestTarget.centerX, bestTarget.centerY), (int) bestTarget.radius, new Scalar(0, 255, 0), 3);
            Imgproc.circle(input, new Point(bestTarget.centerX, bestTarget.centerY), 5, new Scalar(255, 0, 0), -1);

            String label = bestTarget.type == BallType.NECTAR ? "NECTAR" : "POLLEN";
            Imgproc.putText(input, label + " (R=" + (int) bestTarget.radius + ")",
                    new Point(bestTarget.centerX - 30, bestTarget.centerY - (bestTarget.radius + 10)),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255, 255, 255), 2);
        }

        // Draw center crosshair
        Imgproc.line(input, new Point(input.cols() / 2.0 - 15, input.rows() / 2.0),
                new Point(input.cols() / 2.0 + 15, input.rows() / 2.0), new Scalar(255, 255, 255), 1);
        Imgproc.line(input, new Point(input.cols() / 2.0, input.rows() / 2.0 - 15),
                new Point(input.cols() / 2.0, input.rows() / 2.0 + 15), new Scalar(255, 255, 255), 1);

        latestTarget = bestTarget;
        return input;
    }

    public BallTarget getLatestTarget() {
        return latestTarget;
    }
}
