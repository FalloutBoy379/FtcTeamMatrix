package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.drive.Localization;
import org.firstinspires.ftc.teamcode.drive.PIDFController;


@Autonomous
@Config
public class DrivePIDTest extends LinearOpMode {

    public static double x=0,y=0,theta=0;
    public static double Kpx= -0.08, Kix= 0, Kdx = -0.4, Kfx = 0;
    public static double Kpy= 0.25, Kiy= 0, Kdy = 0.3, Kfy = 0;

    public static double Kptheta= -0.05, Kitheta= 0, Kdtheta = -0.05, Kftheta = 0;

    PIDFController xcontroller, ycontroller, thetacontroller;

    @Override
    public void runOpMode() throws InterruptedException {

        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);
        Localization localization = new Localization(hardwareMap);
        xcontroller = new PIDFController(Kpx, Kix, Kdx, Kfx);
        ycontroller = new PIDFController(Kpy, Kiy, Kdy, Kfy);
        thetacontroller = new PIDFController(Kptheta, Kitheta, Kdtheta, Kftheta);

        Pose2d target;
        waitForStart();



        while(opModeIsActive()){
            xcontroller.setCoeffs(Kpx,Kix,Kdx,Kfx);
            ycontroller.setCoeffs(Kpy, Kiy, Kdy, Kfy);
            thetacontroller.setCoeffs(Kptheta, Kitheta, Kdtheta, Kftheta);

            if(gamepad1.a){

            }
            if(gamepad1.b){
                y = -48;
                x = 20;
                theta = 0;
            }
            double powerx = xcontroller.calculate(localization.getRobotPose().getX(), x);
            double powery = ycontroller.calculate(localization.getRobotPose().getY(), y);
            double powertheta = thetacontroller.calculate(localization.getRobotPose().getHeading(), theta);

            drivetrain.calculateMotorSpeeds(-powery, -powerx,powertheta);
            telemetry.addData("X:", localization.getRobotPose().getX());
            telemetry.addData("Y:", localization.getRobotPose().getY());
            telemetry.addData("Heagin:", localization.getRobotPose().getHeading());
//            telemetry.addData("power", powerx);
            telemetry.update();
        }
    }
}
