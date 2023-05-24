package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class TrajectoryTest extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime timer = new ElapsedTime();
        boolean prevStart = false;

        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);
        Localization localization = new Localization(hardwareMap);
        localization.resetPose();
        DriveController driveController = new DriveController(drivetrain, localization, telemetry);
        waitForStart();

        driveController.goTo(2,-50,0);
        while(!driveController.isFinished());
        timer.reset();
        while ((timer.seconds() < 1)){
            driveController.update();
        }
        driveController.goTo(10, -50, 0);

        while(opModeIsActive()){
            timer.reset();
            if(gamepad1.a){

            }
            if(gamepad1.b){
                driveController.goTo(10,-50,0);
            }
            if(gamepad1.start && !prevStart){
                localization.setHeadingSourceImu(true);
            }

            prevStart = gamepad1.start;
            driveController.update();
            localization.setHeadingSourceImu(false);
        }
    }
}
