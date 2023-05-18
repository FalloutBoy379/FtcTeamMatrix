package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.drive.Localization;

@TeleOp
public class Teleop extends LinearOpMode {

Localization localization;
Drivetrain drive;

    @Override
    public void runOpMode() throws InterruptedException {
        localization = new Localization(hardwareMap);
        drive = new Drivetrain(hardwareMap, telemetry);



        waitForStart();


        while(opModeIsActive()){
            drive.calculateMotorSpeeds(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            Pose2d robotPose = localization.getRobotPose();
            telemetry.addData("X: ", robotPose.getX());
            telemetry.addData("Y: ", robotPose.getY());
            telemetry.addData("Heading: ", robotPose.getHeading());
            telemetry.update();
        }
    }
}
