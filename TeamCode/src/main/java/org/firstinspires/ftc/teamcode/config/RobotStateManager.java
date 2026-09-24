package org.firstinspires.ftc.teamcode.config;

import android.content.Context;
import android.content.SharedPreferences;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotStateManager {

    private final SharedPreferences prefs;

    public RobotStateManager(HardwareMap hardwareMap) {
        this.prefs = hardwareMap.appContext.getSharedPreferences(RobotConstants.PREFS_NAME, Context.MODE_PRIVATE);
    }

    public void saveRobotPose(Pose2d pose) {
        SharedPreferences.Editor editor = prefs.edit();
        editor.putFloat(RobotConstants.KEY_POSE_X, (float) pose.position.x);
        editor.putFloat(RobotConstants.KEY_POSE_Y, (float) pose.position.y);
        editor.putFloat(RobotConstants.KEY_POSE_HEADING, (float) pose.heading.toDouble());
        editor.apply();
    }

    public Pose2d loadRobotPose(Pose2d defaultPose) {
        double x = prefs.getFloat(RobotConstants.KEY_POSE_X, (float) defaultPose.position.x);
        double y = prefs.getFloat(RobotConstants.KEY_POSE_Y, (float) defaultPose.position.y);
        double heading = prefs.getFloat(RobotConstants.KEY_POSE_HEADING, (float) defaultPose.heading.toDouble());
        return new Pose2d(x, y, heading);
    }

    public void saveSlideOffset(int encoderOffset) {
        prefs.edit().putInt(RobotConstants.KEY_SLIDE_OFFSET, encoderOffset).apply();
    }

    public int loadSlideOffset() {
        return prefs.getInt(RobotConstants.KEY_SLIDE_OFFSET, 0);
    }

    public void saveFlowersOffset(int encoderOffset) {
        prefs.edit().putInt(RobotConstants.KEY_FLOWERS_OFFSET, encoderOffset).apply();
    }

    public int loadFlowersOffset() {
        return prefs.getInt(RobotConstants.KEY_FLOWERS_OFFSET, 0);
    }

    public void saveTunedPidGains() {
        SharedPreferences.Editor editor = prefs.edit();
        editor.putFloat(RobotConstants.KEY_OUTTAKE_KP, (float) RobotConstants.OUTTAKE_KP);
        editor.putFloat(RobotConstants.KEY_OUTTAKE_KI, (float) RobotConstants.OUTTAKE_KI);
        editor.putFloat(RobotConstants.KEY_OUTTAKE_KD, (float) RobotConstants.OUTTAKE_KD);
        editor.putFloat(RobotConstants.KEY_OUTTAKE_KG, (float) RobotConstants.OUTTAKE_KG);

        editor.putFloat(RobotConstants.KEY_FLOWERS_KP, (float) RobotConstants.FLOWERS_KP);
        editor.putFloat(RobotConstants.KEY_FLOWERS_KI, (float) RobotConstants.FLOWERS_KI);
        editor.putFloat(RobotConstants.KEY_FLOWERS_KD, (float) RobotConstants.FLOWERS_KD);
        editor.putFloat(RobotConstants.KEY_FLOWERS_KG, (float) RobotConstants.FLOWERS_KG);

        editor.apply();
    }

    public void loadTunedPidGains() {
        if (prefs.contains(RobotConstants.KEY_OUTTAKE_KP)) {
            RobotConstants.OUTTAKE_KP = prefs.getFloat(RobotConstants.KEY_OUTTAKE_KP, (float) RobotConstants.OUTTAKE_KP);
            RobotConstants.OUTTAKE_KI = prefs.getFloat(RobotConstants.KEY_OUTTAKE_KI, (float) RobotConstants.OUTTAKE_KI);
            RobotConstants.OUTTAKE_KD = prefs.getFloat(RobotConstants.KEY_OUTTAKE_KD, (float) RobotConstants.OUTTAKE_KD);
            RobotConstants.OUTTAKE_KG = prefs.getFloat(RobotConstants.KEY_OUTTAKE_KG, (float) RobotConstants.OUTTAKE_KG);
        }

        if (prefs.contains(RobotConstants.KEY_FLOWERS_KP)) {
            RobotConstants.FLOWERS_KP = prefs.getFloat(RobotConstants.KEY_FLOWERS_KP, (float) RobotConstants.FLOWERS_KP);
            RobotConstants.FLOWERS_KI = prefs.getFloat(RobotConstants.KEY_FLOWERS_KI, (float) RobotConstants.FLOWERS_KI);
            RobotConstants.FLOWERS_KD = prefs.getFloat(RobotConstants.KEY_FLOWERS_KD, (float) RobotConstants.FLOWERS_KD);
            RobotConstants.FLOWERS_KG = prefs.getFloat(RobotConstants.KEY_FLOWERS_KG, (float) RobotConstants.FLOWERS_KG);
        }
    }

    public void checkAndHandleDashboardSave() {
        if (RobotConstants.SAVE_PID_VALUES_TO_ROBOT) {
            saveTunedPidGains();
            RobotConstants.SAVE_PID_VALUES_TO_ROBOT = false;
        }
    }

    public void clearState() {
        prefs.edit().clear().apply();
    }
}
