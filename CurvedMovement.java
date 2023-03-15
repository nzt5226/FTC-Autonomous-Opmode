package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CurvedMovement {
    private Robot robot;
    private Telemetry telemetry;
    private double[] pose = {0, 0, 0}; // x, y, theta in inches and radians

    private double targetHeading;
    private double targetDistance;
    private double currentPower;

    // Constants for PID control
    private static final double kP = 
