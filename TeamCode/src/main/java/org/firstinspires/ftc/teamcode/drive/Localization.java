package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.MatrixUtils;


public class Localization {

    boolean headingSourceisImu = false;
    private BNO055IMU imu;
    public  static double leftMulti = 1;
    public static double X_MULTIPLIER = 1.017656780029573 * 0.975609756097561 * 0.9969481180061038; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 0.9783938035059111 * 0.9921414538310413; // Multiplier in the Y direction

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public Localization(HardwareMap hardwareMap){
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));

        initializeIMU(hardwareMap);

        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public Pose2d getRobotPose(){
        double[] poseArray = calculatePose(getEncoderCounts(), 1.49606, 9.9102, 1.25);
        Pose2d robotPose = new Pose2d(poseArray[0], poseArray[1], poseArray[2]);
        return robotPose;
    }

    private double[] getEncoderCounts(){
        double leftEncoderCounts = leftEncoder.getCurrentPosition() * X_MULTIPLIER * leftMulti;
        double rightEncoderCounts = rightEncoder.getCurrentPosition() * X_MULTIPLIER;
        double perpEncoderCounts = frontEncoder.getCurrentPosition() * Y_MULTIPLIER;
        double[] encoders = {leftEncoderCounts, rightEncoderCounts, perpEncoderCounts};
        return encoders;
    }

    public void setHeadingSourceImu(boolean flag){
        headingSourceisImu = flag;
    }



    public double[] calculatePose(double[] encoders, double wheelDiameter, double wheelDistance, double forwardOffset) {
        double TICKS_PER_REVOLUTION = 8192;
        double ticksPerInch = TICKS_PER_REVOLUTION / (Math.PI * wheelDiameter);
        double wheelCircumference = Math.PI * wheelDiameter;
        double distancePerTick = wheelCircumference / TICKS_PER_REVOLUTION;

        double leftTicks = encoders[0];
        double rightTicks = encoders[1];
        double perpTicks = encoders[2];

        double x = (leftTicks + rightTicks) * 0.5 * distancePerTick;
        double y = 0;
        if (perpTicks != 0) {
            y = perpTicks * distancePerTick - forwardOffset;
        }

        double theta;
        if(headingSourceisImu){
            theta = getNormalizedHeading();
        }else {
            theta = (rightTicks - leftTicks) * distancePerTick / wheelDistance;
        }
//        Vector2d pose = new Vector2d(x, y);
//        pose = pose.rotated(theta);

        return new double[]{x, y, Math.toDegrees(theta)};
    }

    private void initializeIMU(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    private double getNormalizedHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double heading = angles.firstAngle;  // Raw heading value

        // Adjust heading to a range of 0° to 360°
        if (heading < 0) {
            heading += 360;
        }

        // Handle wraparound if heading exceeds 360°
        if (heading > 360) {
            heading -= 360;
        }

        return heading;
    }
}
