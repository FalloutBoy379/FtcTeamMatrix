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

        telemetry = new MultipleTelemetry(telemetry,dashboard.getTelemetry());

        localization = new Localization(hardwareMap);
        localization.resetPose();
        drive = new Drivetrain(hardwareMap, telemetry);



        waitForStart();


        while(opModeIsActive()){

            if(gamepad1.a){
                localization.resetPose();
            }
            if(gamepad1.b){
                localization.setHeadingSourceImu(true);
            }
            else{
                localization.setHeadingSourceImu(false);
            }
            drive.calculateMotorSpeeds(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            Pose2d robotPose = localization.getRobotPose();
            Pose2d robotPoseField = localization.getRobotPoseinFieldCoordinates();

            telemetry.addLine("Runtime"+ runtime.toString());

            telemetry.addLine("X Robot: "+ robotPose.getX());
            telemetry.addLine("Y Robot: "+ robotPose.getY());
            telemetry.addLine("Heading IMU: "+ Math.toDegrees(localization.imu.getAngularOrientation().firstAngle));
            telemetry.addLine("X: "+ robotPoseField.getX());
            telemetry.addLine("Y: "+ robotPoseField.getY());
            telemetry.addLine("Heading Pods: "+ robotPoseField.getHeading());
            telemetry.update();
        }
    }
}
