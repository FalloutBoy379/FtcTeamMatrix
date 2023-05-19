package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.drive.Localization;

@TeleOp
public class Teleop extends LinearOpMode {

Localization localization;

    private final double FIELD_WIDTH_INCHES = 144.0; // Field width in inches
    private final double FIELD_HEIGHT_INCHES = 144.0; // Field height in inches
    private final int CANVAS_WIDTH = 800; // Canvas width in pixels
    private final int CANVAS_HEIGHT = 800; // Canvas height in pixels

TelemetryPacket telemetryPacket;
Drivetrain drive;

    private FtcDashboard dashboard;
    private ElapsedTime runtime;

    @Override
    public void runOpMode() throws InterruptedException {

        dashboard = FtcDashboard.getInstance();
        runtime = new ElapsedTime();

//        telemetry = new MultipleTelemetry(telemetry,);

        localization = new Localization(hardwareMap);
        localization.resetPose();
//        drive = new Drivetrain(hardwareMap, telemetry);



        waitForStart();


        while(opModeIsActive()){
            telemetryPacket = new TelemetryPacket();

            Canvas canvas = telemetryPacket.fieldOverlay();
            canvas.clear();



            if(gamepad1.a){
                localization.resetPose();
            }
//            drive.calculateMotorSpeeds(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            Pose2d robotPose = localization.getRobotPose();
            Pose2d robotPoseField = localization.getRobotPoseinFieldCoordinates();


            double robotXPixels = robotPoseField.getX() / FIELD_WIDTH_INCHES * CANVAS_WIDTH;
            double robotYPixels = robotPoseField.getY() / FIELD_HEIGHT_INCHES * CANVAS_HEIGHT;

            canvas.setStroke("#FF0000"); // Set color to red
            canvas.strokeCircle(robotXPixels, robotYPixels, 10); // Adjust the circle radius as needed
            canvas.strokeLine(robotXPixels, robotYPixels, robotXPixels + Math.cos(Math.toRadians(robotPoseField.getHeading())) * 20, robotYPixels + Math.sin(Math.toRadians(robotPoseField.getHeading())) * 20);
            telemetryPacket.addLine("Runtime"+ runtime.toString());

            telemetryPacket.addLine("X Robot: "+ robotPose.getX());
            telemetryPacket.addLine("Y Robot: "+ robotPose.getY());
            telemetryPacket.addLine("Heading Robot: "+ robotPose.getHeading());
            telemetryPacket.addLine("X: "+ robotPoseField.getX());
            telemetryPacket.addLine("Y: "+ robotPoseField.getY());
            telemetryPacket.addLine("Heading: "+ robotPoseField.getHeading());
            dashboard.sendTelemetryPacket(telemetryPacket);
//            telemetry.update();
        }
    }
}
