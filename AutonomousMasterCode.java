package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Autonomous Master Code", group="Linear Opmode")
public class AutonomousMasterCode extends LinearOpMode {

    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private RevRoboticsCoreHexMotor intake, shooter;
    private Servo claw;
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle;

    private Odometry odometry;
    private SensorFusion sensorFusion;
    private CurvedMovement curvedMovement;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_INCH = 307.699557;

    static final double DRIVE_SPEED = 0.5;
    static final double TURN_SPEED = 0.3;

    static final double HEADING_THRESHOLD = 1;
    static final double P_TURN_COEFF = 0.1;
    static final double P_DRIVE_COEFF = 0.15;

    // Kalman filter variables
    private static final double Q_ANGLE = 0.01;
    private static final double Q_GYRO = 0.005;
    private static final double R_ANGLE = 0.5;
    private KalmanFilter kalmanFilter = new KalmanFilter(Q_ANGLE, Q_GYRO, R_ANGLE);

    @Override
    public void runOpMode() {

        leftFront = hardwareMap.dcMotor.get("left_front_motor");
        leftBack = hardwareMap.dcMotor.get("left_back_motor");
        rightFront = hardwareMap.dcMotor.get("right_front_motor");
        rightBack = hardwareMap.dcMotor.get("right_back_motor");
        intake = (RevRoboticsCoreHexMotor) hardwareMap.dcMotor.get("intake");
        shooter = (RevRoboticsCoreHexMotor) hardwareMap.dcMotor.get("shooter");
        claw = hardwareMap.servo.get("claw");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        // Initialize odometry and sensor fusion
        odometry = new Odometry(leftFront, rightFront, leftBack, COUNTS_PER_INCH);
        sensorFusion = new SensorFusion(imu, kalmanFilter);
        curvedMovement = new CurvedMovement(odometry, sensorFusion, leftFront, rightFront, leftBack, rightBack);

        // Set up imu parameters
        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU
