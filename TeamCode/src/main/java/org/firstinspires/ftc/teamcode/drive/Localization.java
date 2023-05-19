package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
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



    private double fieldX;
    private double fieldY;
    private double prevX;
    private double prevY;
    private double heading;
    boolean headingSourceisImu = false;

    private double xOffset, yOffset, thetaOffset;
    private BNO055IMU imu;
    public  static double leftMulti = 1;
    public static double X_MULTIPLIER = 1.017656780029573 * 0.975609756097561 * 0.9969481180061038; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 0.9783938035059111 * 0.9921414538310413; // Multiplier in the Y direction

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public Localization(HardwareMap hardwareMap){
        fieldX = 0.0;
        fieldY = 0.0;
        heading = 0.0;
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
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

    public Pose2d getRobotPoseinFieldCoordinates(){
        double[] poseArray = calculatePose(getEncoderCounts(), 1.49606, 9.9102, 1.25);
        Pose2d robotPose = new Pose2d(poseArray[0], poseArray[1], poseArray[2]);
        updatePosition(robotPose.getX(), robotPose.getY(), robotPose.getHeading());
        return new Pose2d(fieldX, fieldY, heading);
    }

    public void updatePosition(double deltaX, double deltaY, double newHeading) {
        // Convert heading to radians
        double headingRad = Math.toRadians(newHeading);

        // Rotate the change in position
        double changeXTransformed = deltaX * Math.cos(headingRad) - deltaY * Math.sin(headingRad);
        double changeYTransformed = deltaX * Math.sin(headingRad) + deltaY * Math.cos(headingRad);

        // Update the robot's position in the field coordinate system
        fieldX = changeXTransformed;
        fieldY = changeYTransformed;

        heading = newHeading;
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

    public void resetPose(){
        xOffset = getRobotPose().getX() + xOffset;
        yOffset = getRobotPose().getY() + yOffset;
        thetaOffset = getRobotPose().getHeading() + thetaOffset;
    }



    public double[] calculatePose(double[] encoders, double wheelDiameter, double wheelDistance, double forwardOffset) {
        double TICKS_PER_REVOLUTION = 8192;
        double ticksPerInch = TICKS_PER_REVOLUTION / (Math.PI * wheelDiameter);
        double wheelCircumference = Math.PI * wheelDiameter;
        double distancePerTick = wheelCircumference / TICKS_PER_REVOLUTION;

        double leftTicks = encoders[0];
        double rightTicks = encoders[1];
        double perpTicks = encoders[2];

        double theta;
        if(headingSourceisImu){
            theta = getIntegratedHeading();
        }else {
            theta = (rightTicks - leftTicks) * distancePerTick / wheelDistance;
        }

        double x = (leftTicks+rightTicks) * 0.5 * distancePerTick;
        double y = 0;
        if (perpTicks != 0) {
            y = (perpTicks * distancePerTick - (forwardOffset));
        }


//        Vector2d pose = new Vector2d(x, y);
//        pose = pose.rotated(theta);

        return new double[]{x - xOffset, y - yOffset, Math.toDegrees(theta) - thetaOffset};
    }

    private void initializeIMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    private double previousHeading = 0; //Outside of method
    private double integratedHeading = 0;

    /**
     * This method returns a value of the Z axis of the REV Expansion Hub IMU.
     * It transforms the value from (-180, 180) to (-inf, inf).
     * This code was taken and modified from https://ftcforum.usfirst.org/forum/ftc-technology/53477-rev-imu-questions?p=53481#post53481.
     * @return The integrated heading on the interval (-inf, inf).
     */
    private double getIntegratedHeading() {
        double currentHeading = imu.getAngularOrientation().firstAngle;
        double deltaHeading = currentHeading - previousHeading;

        if (deltaHeading < -180) {
            deltaHeading += 360;
        } else if (deltaHeading >= 180) {
            deltaHeading -= 360;
        }

        integratedHeading += deltaHeading;
        previousHeading = currentHeading;

        return integratedHeading;
    }
}
