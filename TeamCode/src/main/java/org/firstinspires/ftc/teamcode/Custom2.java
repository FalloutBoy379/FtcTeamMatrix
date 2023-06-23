package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;


@TeleOp
public class Custom2 extends LinearOpMode {

    private double fieldX;
    private double fieldY;
    private double heading;

    public static double FORWARD_OFFSET = 1.2732;   //inches

    private double xOffset, yOffset, thetaOffset;


    public static double X_MULTIPLIER = 1.017656780029573 * 0.975609756097561 * 0.9969481180061038; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 0.9783938035059111 * 0.9921414538310413; // Multiplier in the Y direction

    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private Encoder frontEncoder;

    private double leftEncoderPrev, rightEncoderPrev, frontEncoderPrev;


    @Override
    public void runOpMode() throws InterruptedException {
        fieldX = 0.0;
        fieldY = 0.0;
        heading = 0.0;
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));


        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);

        resetPose();

        waitForStart();



        while (opModeIsActive()){
            Pose2d pose = getRobotPose();
            telemetry.addData("X: ", pose.getX());
            telemetry.addData("Y", pose.getY());
            telemetry.addData("Heading: ", Math.toDegrees(pose.getHeading()));
            telemetry.update();
        }
    }




    public Pose2d getRobotPose(){
        double L = 10.2474;
        double currentLeft = getDistance(leftEncoder) * X_MULTIPLIER;
        double currentRight = getDistance(rightEncoder) * X_MULTIPLIER;
        double currentFront = getDistance(frontEncoder) * Y_MULTIPLIER;
        double deltaLeft = currentLeft - leftEncoderPrev;
        double deltaRight = currentRight - rightEncoderPrev;
        double deltaFront = currentFront - frontEncoderPrev;

        double deltaX = (deltaLeft + deltaRight)/2;
        double deltaY = deltaFront - FORWARD_OFFSET * ((deltaLeft - deltaRight)/L);
        double deltaThetha = ((deltaRight-deltaLeft)/L);

        fieldX += (deltaX*(Math.cos(heading)))-(deltaY*(Math.sin(heading)));
        fieldY += (deltaX*(Math.sin(heading)))+(deltaY*(Math.cos(heading)));
        heading += deltaThetha;


        leftEncoderPrev = currentLeft;
        rightEncoderPrev = currentRight;
        frontEncoderPrev = currentFront;
        return new Pose2d(fieldX - xOffset, fieldY - yOffset, heading - thetaOffset);
    }


    public double getDistance(Encoder encoder){
        double wheelDiameter = 1.49606;
        double TICKS_PER_REVOLUTION = 8192;
        double wheelCircumference = Math.PI * wheelDiameter;
        double distancePerTick = wheelCircumference / TICKS_PER_REVOLUTION;
        return (encoder.getCurrentPosition() * distancePerTick);
    }



    public void resetPose(){
        xOffset = getRobotPose().getX() + xOffset;
        yOffset = getRobotPose().getY() + yOffset;
        thetaOffset = getRobotPose().getHeading() + thetaOffset;
    }
}
