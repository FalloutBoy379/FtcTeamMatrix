package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveController;
import org.firstinspires.ftc.teamcode.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.drive.Localization;


@Autonomous
public class autoTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        Localization localization = new Localization(hardwareMap);
        localization.resetPose();
        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);
        DriveController driveController = new DriveController(drivetrain, localization, telemetry);



        waitForStart();


        driveController.coordinateSequencer(new Pose2d(2, -51, Math.toRadians(0)),
                new Pose2d(-10,-51,0));
//        driveController.goTo(2, -51, 0);
        while(opModeIsActive()){
            driveController.update();
        }
    }
}
