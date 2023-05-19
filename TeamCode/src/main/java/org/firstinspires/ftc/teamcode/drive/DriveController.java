package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class DriveController {

    ElapsedTime timer;

    Localization localization;
    MotionProfile driveProfile;
    Drivetrain drivetrain;
//    public static double x=0,y=0,theta=0;

    public double targetX, targetY, targetHeading;
public static double Kpx= -0.08, Kix= 0, Kdx = -0.4, Kfx = 0;
    public static double Kpy= 0.25, Kiy= 0, Kdy = 0.3, Kfy = 0;

    public static double Kptheta= -0.01, Kitheta= 0, Kdtheta = -0.05, Kftheta = 0;

    PIDFController xcontroller, ycontroller, thetacontroller;

    MotionProfile motionProfilex, motionProfiley, motionProfiletheta;

    public DriveController(Drivetrain drivetrain, Localization localization){
        this.drivetrain = drivetrain;
        this.localization = localization;
        xcontroller = new PIDFController(Kpx, Kix, Kdx, Kfx);
        ycontroller = new PIDFController(Kpy, Kiy, Kdy, Kfy);
        thetacontroller = new PIDFController(Kptheta, Kitheta, Kdtheta, Kftheta);
        timer = new ElapsedTime();

    }

    public void setPosition(double x, double y, double theta){
        xcontroller.setCoeffs(Kpx,Kix,Kdx,Kfx);
        ycontroller.setCoeffs(Kpy, Kiy, Kdy, Kfy);
        thetacontroller.setCoeffs(Kptheta, Kitheta, Kdtheta, Kftheta);

        double powerx = xcontroller.calculate(localization.getRobotPose().getX(), x);
        double powery = ycontroller.calculate(localization.getRobotPose().getY(), y);
        double powertheta = thetacontroller.calculate(localization.getRobotPose().getHeading(), theta);

        drivetrain.calculateMotorSpeeds(-powery, -powerx,powertheta);
    }

    public boolean isAtTarget(){
        return xcontroller.isAtTarget() && ycontroller.isAtTarget() && thetacontroller.isAtTarget();
    }

    public boolean isFinished(){
        if(motionProfiley !=null) {
            boolean stateFinishedX = areWithinRange(localization.getRobotPose().getX(), targetX, 3.0);
            boolean stateFinishedY = areWithinRange(localization.getRobotPose().getY(), targetY, 3.0);
            boolean stateFinishedHeading = areWithinRange(localization.getRobotPose().getHeading(), targetHeading, 2.0);
            return stateFinishedX && stateFinishedY && stateFinishedHeading;
        }
        else return true;
    }

    public void update(){
        if(motionProfiletheta != null) {
            MotionState xState = motionProfilex.get(timer.seconds());
            MotionState yState = motionProfiley.get(timer.seconds());
            MotionState thetaState = motionProfiletheta.get(timer.seconds());
            setPosition(xState.getX(), yState.getX(), thetaState.getX());
        }

    }

    public void goTo(double x, double y, double theta){
        this.goTo(x, y, theta, 60,60,180,180);
    }

    public void goTo(double x, double y, double theta, double maxVel, double maxAccel, double maxAngVel, double maxAngAccel){


        targetX = x;
        targetY = y;
        targetHeading = theta;
        double currentHeading = localization.getRobotPose().getHeading();
        double currentx = localization.getRobotPose().getX();
        double currenty = localization.getRobotPose().getY();
        motionProfilex = MotionProfileGenerator.generateSimpleMotionProfile(
           new MotionState(currentx,0,0),
                new MotionState(x, 0, 0),
                maxVel,
                maxAccel
        );

        motionProfiley = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currenty,0,0),
                new MotionState(y, 0, 0),
                maxVel,
                maxAccel
        );

        motionProfiletheta = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentHeading, 0, 0),
                new MotionState(theta,0,0),
                maxAngVel,
                maxAngAccel
        );
        timer.reset();
    }

    public boolean areWithinRange(double num1, double num2, double range) {
        double difference = Math.abs(num1 - num2);
        return difference <= range;
    }


}
