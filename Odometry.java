package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Odometry {
    private final Robot robot;
    private final ElapsedTime timer;

    private double leftEncoderPosition;
    private double rightEncoderPosition;
    private double horizontalEncoderPosition;

    private double previousLeftEncoderPosition = 0;
    private double previousRightEncoderPosition = 0;
    private double previousHorizontalEncoderPosition = 0;

    private double x = 0;
    private double y = 0;
    private double heading = 0;

    private static final double WHEEL_DIAMETER = 0.05; // meters
    private static final double WHEEL_DISTANCE = 0.265; // meters
    private static final double TICKS_PER_ROTATION = 8192;

    public Odometry(Robot robot) {
        this.robot = robot;
        this.timer = new ElapsedTime();
    }

    public void start() {
        timer.reset();
        resetEncoders();
    }

    public void update() {
        double currentTime = timer.seconds();

        double leftEncoderDelta = (leftEncoderPosition - previousLeftEncoderPosition) * WHEEL_DIAMETER * Math.PI / TICKS_PER_ROTATION;
        double rightEncoderDelta = (rightEncoderPosition - previousRightEncoderPosition) * WHEEL_DIAMETER * Math.PI / TICKS_PER_ROTATION;
        double horizontalEncoderDelta = (horizontal
