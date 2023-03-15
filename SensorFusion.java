import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SensorFusion {
    private BNO055IMU imu;
    private DistanceSensor leftDistanceSensor, rightDistanceSensor;
    private Encoder leftEncoder, rightEncoder;
    private Telemetry telemetry;
    private double[] distances;
    private int[] leftEncoderCounts, rightEncoderCounts;

    // Kalman filter variables
    private double[] x_hat = {0, 0, 0}; // x, y, theta
    private double[][] P = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}; // covariance matrix
    private double[][] Q = {{0.01, 0, 0}, {0, 0.01, 0}, {0, 0, 0.01}}; // process noise covariance matrix
    private double[][] R = {{0.5, 0, 0}, {0, 0.5, 0}, {0, 0, 0.5}}; // measurement noise covariance matrix

    public SensorFusion(HardwareMap hardwareMap, Telemetry telemetry) {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");
        leftEncoder = hardwareMap.get(Encoder.class, "leftEncoder");
        rightEncoder = hardwareMap.get(Encoder.class, "rightEncoder");
        this.telemetry = telemetry;

        distances = new double[2];
        leftEncoderCounts = new int[2];
        rightEncoderCounts = new int[2];

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(imuParameters);
    }

    // update all sensor data
    public void update() {
        updateIMU();
        updateDistanceSensors();
        updateEncoders();
        updateKalman();
    }

    // update IMU data
    private void updateIMU() {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        x_hat[2] = orientation.firstAngle;
    }

    // update distance sensor data
    private void updateDistanceSensors() {
        distances[0] = leftDistanceSensor.getDistance(DistanceUnit.INCH);
        distances[1] = rightDistanceSensor.getDistance(DistanceUnit.INCH);
    }

    // update encoder data
    private void updateEncoders() {
        int leftEncoderCount = leftEncoder.getCurrentPosition();
        int rightEncoderCount = rightEncoder.getCurrentPosition();
        leftEncoderCounts[1] = leftEncoderCount - leftEncoderCounts[0];
        rightEncoderCounts[1] = rightEncoderCount - rightEncoderCounts[0];
        leftEncoderCounts[0] = leftEncoderCount;
        rightEncoderCounts
