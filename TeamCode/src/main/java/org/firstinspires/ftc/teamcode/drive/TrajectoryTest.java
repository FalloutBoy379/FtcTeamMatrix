package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class TrajectoryTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime timer = new ElapsedTime();

        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);
        Localization localization = new Localization(hardwareMap);
        localization.resetPose();
        DriveController driveController = new DriveController(drivetrain, localization);
        waitForStart();


        while(opModeIsActive()){
            timer.reset();
            if(gamepad1.a){
                driveController.goTo(2,-50,0);
            }
            if(gamepad1.b){
                driveController.goTo(10,-50,0);
            }



            driveController.update();

            telemetry.addData("TargetX: ", driveController.targetX);
            telemetry.addData("TargetY: ", driveController.targetY);
            telemetry.addData("TargetHeading: ", driveController.targetHeading);
            telemetry.addData("isFinished: ", driveController.isFinished());
            telemetry.addData("X: ", localization.getRobotPose().getX());
            telemetry.addData("Y: ", localization.getRobotPose().getY());
            telemetry.addData("Heading: ", localization.getRobotPose().getHeading());
            telemetry.addData("Timing: ", timer.milliseconds());
            telemetry.update();
        }
    }
}
