package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {

    // --- Dashboard Live Save Toggle ---
    public static boolean SAVE_PID_VALUES_TO_ROBOT = false;

    // --- Hardware Device Names ---
    public static String MOTOR_FRONT_LEFT = "frontLeft";
    public static String MOTOR_FRONT_RIGHT = "frontRight";
    public static String MOTOR_BACK_LEFT = "backLeft";
    public static String MOTOR_BACK_RIGHT = "backRight";

    public static String MOTOR_OUTTAKE_SLIDE = "slide_motor";
    public static String MOTOR_INTAKE = "intake_motor";
    public static String MOTOR_TRANSFER = "transfer_motor";
    public static String MOTOR_FLOWERS = "flowers_motor";

    public static String SERVO_INTAKE_PITCH = "intake_pitch";
    public static String SERVO_INTAKE_CLAW = "intake_claw";
    public static String SERVO_TRANSFER_GATE = "transfer_gate";
    public static String SERVO_OUTTAKE_CLAW = "outtake_claw";
    public static String SERVO_FLOWERS_PUSHER = "flowers_pusher";

    public static String SENSOR_COLOR_INTAKE = "intake_color_sensor";
    public static String WEBCAM_NAME = "Webcam 1";

    // --- Outtake Slide PIDF & Feedforward Constants ---
    public static double OUTTAKE_KP = 0.005;
    public static double OUTTAKE_KI = 0.00005;
    public static double OUTTAKE_KD = 0.00015;
    public static double OUTTAKE_KF = 0.0;
    public static double OUTTAKE_KG = 0.08; // Gravity compensation power
    public static double OUTTAKE_KS = 0.02; // Static friction feedforward

    public static double OUTTAKE_MAX_VELOCITY_INCHES_PER_SEC = 25.0;
    public static double OUTTAKE_MAX_ACCEL_INCHES_PER_SEC2 = 50.0;
    public static double OUTTAKE_TICKS_PER_INCH = 145.0; // goBILDA 312 RPM spool standard
    public static double OUTTAKE_MAX_HEIGHT_INCHES = 30.0;

    // --- Outtake Preset Heights (Inches) ---
    public static double OUTTAKE_STOW_HEIGHT = 0.0;
    public static double OUTTAKE_INTAKE_HEIGHT = 1.0;
    public static double OUTTAKE_TRANSFER_HEIGHT = 2.5;
    public static double OUTTAKE_LOW_TARGET_HEIGHT = 14.0;
    public static double OUTTAKE_HIGH_TARGET_HEIGHT = 26.0;

    // --- Flowers Mechanism PIDF Constants ---
    public static double FLOWERS_KP = 0.006;
    public static double FLOWERS_KI = 0.00001;
    public static double FLOWERS_KD = 0.0002;
    public static double FLOWERS_KF = 0.0;
    public static double FLOWERS_KG = 0.05;

    public static double FLOWERS_STOW_POSITION = 0.0; // Motor ticks or degrees
    public static double FLOWERS_SCORE_POSITION = 450.0;

    // --- Drive Subsystem Heading PID Constants ---
    public static double HEADING_KP = 1.8;
    public static double HEADING_KI = 0.0;
    public static double HEADING_KD = 0.12;

    // --- Servo Position Setpoints ---
    public static double INTAKE_PITCH_GROUND = 0.85;
    public static double INTAKE_PITCH_STOW = 0.20;
    public static double INTAKE_CLAW_OPEN = 0.60;
    public static double INTAKE_CLAW_CLOSED = 0.20;

    public static double TRANSFER_GATE_OPEN = 0.70;
    public static double TRANSFER_GATE_CLOSED = 0.15;

    public static double OUTTAKE_CLAW_OPEN = 0.55;
    public static double OUTTAKE_CLAW_CLOSED = 0.15;

    public static double FLOWERS_PUSHER_RETRACT = 0.10;
    public static double FLOWERS_PUSHER_EXTEND = 0.85;

    // --- Game Element (Ball) Detection Thresholds ---
    public static double NECTAR_MIN_RADIUS_PX = 15.0;
    public static double NECTAR_MAX_RADIUS_PX = 45.0;
    public static double POLLEN_MIN_RADIUS_PX = 46.0;
    public static double POLLEN_MAX_RADIUS_PX = 100.0;

    // HSV Color Thresholds
    public static double HSV_YELLOW_LOWER_H = 15;
    public static double HSV_YELLOW_UPPER_H = 35;
    public static double HSV_YELLOW_LOWER_S = 100;
    public static double HSV_YELLOW_UPPER_S = 255;
    public static double HSV_YELLOW_LOWER_V = 100;
    public static double HSV_YELLOW_UPPER_V = 255;

    // --- RoadRunner Kinematics Constants ---
    public static double TRACK_WIDTH_INCHES = 14.5;
    public static double WHEEL_RADIUS_INCHES = 1.88976; // 96mm Mecanum wheels
    public static double GEAR_RATIO = 1.0;
    public static double MAX_WHEEL_VEL = 50.0;
    public static double MAX_WHEEL_ACCEL = 40.0;
    public static double MAX_ANG_VEL = Math.PI;
    public static double MAX_ANG_ACCEL = Math.PI;

    // --- SharedPreferences Storage Keys ---
    public static final String PREFS_NAME = "BIOBUZZ_RobotState";
    public static final String KEY_POSE_X = "pose_x";
    public static final String KEY_POSE_Y = "pose_y";
    public static final String KEY_POSE_HEADING = "pose_heading";
    public static final String KEY_SLIDE_OFFSET = "slide_offset";
    public static final String KEY_FLOWERS_OFFSET = "flowers_offset";

    public static final String KEY_OUTTAKE_KP = "outtake_kp";
    public static final String KEY_OUTTAKE_KI = "outtake_ki";
    public static final String KEY_OUTTAKE_KD = "outtake_kd";
    public static final String KEY_OUTTAKE_KG = "outtake_kg";

    public static final String KEY_FLOWERS_KP = "flowers_kp";
    public static final String KEY_FLOWERS_KI = "flowers_ki";
    public static final String KEY_FLOWERS_KD = "flowers_kd";
    public static final String KEY_FLOWERS_KG = "flowers_kg";
}
