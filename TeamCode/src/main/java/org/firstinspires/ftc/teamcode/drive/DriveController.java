package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
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
    public static double Kpx= 0.08, Kix= 0, Kdx = 0.4, Kfx = 0;
    public static double Kpy= -0.25, Kiy= 0, Kdy = -0.3, Kfy = 0;

    public static double Kptheta= -0.05, Kitheta= 0, Kdtheta = -0.05, Kftheta = 0;

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

    public void update(){
        MotionState xState = motionProfilex.get(timer.seconds());
        MotionState yState = motionProfiley.get(timer.seconds());
        MotionState thetaState = motionProfiletheta.get(timer.seconds());

        setPosition(xState.getX(), yState.getX(), thetaState.getX());
    }

    public void goTo(double x, double y, double theta){
        this.goTo(x, y, theta, 60,60,180,180);
    }

    public void goTo(double x, double y, double theta, double maxVel, double maxAccel, double maxAngVel, double maxAngAccel){
        double currentx = localization.getRobotPose().getX();
        double currenty = localization.getRobotPose().getY();
        double currentHeading = localization.getRobotPose().getHeading();
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


}
