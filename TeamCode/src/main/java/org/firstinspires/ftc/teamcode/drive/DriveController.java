package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class DriveController {

    public static double Kpx = -0.08, Kix = 0, Kdx = -0.2, Kfx = 0;
    public static double Kpy = 0.25, Kiy = 0, Kdy = 0.3, Kfy = 0;
    public static double Kptheta = -0.01, Kitheta = 0, Kdtheta = -0.05, Kftheta = 0;
    public double targetX, targetY, targetHeading;
    ElapsedTime timer;
    Localization localization;
    Drivetrain drivetrain;
    PIDFController xcontroller, ycontroller, thetacontroller;

    MotionProfile motionProfilex, motionProfiley, motionProfiletheta;
    Telemetry telemetry;

    public DriveController(Drivetrain drivetrain, Localization localization){
        this(drivetrain, localization, null);
    }
    public DriveController(Drivetrain drivetrain, Localization localization, Telemetry telemetry) {
        this.drivetrain = drivetrain;
        this.localization = localization;
        this.telemetry = telemetry;
        xcontroller = new PIDFController(Kpx, Kix, Kdx, Kfx);
        ycontroller = new PIDFController(Kpy, Kiy, Kdy, Kfy);
        thetacontroller = new PIDFController(Kptheta, Kitheta, Kdtheta, Kftheta);
        timer = new ElapsedTime();

    }

    public void setPosition(double x, double y, double theta) {
        xcontroller.setCoeffs(Kpx, Kix, Kdx, Kfx);
        ycontroller.setCoeffs(Kpy, Kiy, Kdy, Kfy);
        thetacontroller.setCoeffs(Kptheta, Kitheta, Kdtheta, Kftheta);

        double powerx = xcontroller.calculate(localization.getRobotPoseinFieldCoordinates().getX(), x);
        double powery = ycontroller.calculate(localization.getRobotPoseinFieldCoordinates().getY(), y);
        double powertheta = thetacontroller.calculate(localization.getRobotPoseinFieldCoordinates().getHeading(), theta);

        drivetrain.calculateMotorSpeeds(-powery, -powerx, powertheta);
    }

    public boolean isAtTarget() {
        return xcontroller.isAtTarget() && ycontroller.isAtTarget() && thetacontroller.isAtTarget();
    }

    public boolean isFinished() {
        if (motionProfiley != null) {
            boolean stateFinishedX = areWithinRange(localization.getRobotPoseinFieldCoordinates().getX(), targetX, 3.0);
            boolean stateFinishedY = areWithinRange(localization.getRobotPoseinFieldCoordinates().getY(), targetY, 3.0);
            boolean stateFinishedHeading = areWithinRange(localization.getRobotPoseinFieldCoordinates().getHeading(), targetHeading, 2.0);
            return stateFinishedX && stateFinishedY && stateFinishedHeading;
        } else return true;
    }

    public void update() {
        this.telemetry.addData("TargetX: ", this.targetX);
        telemetry.addData("TargetY: ", this.targetY);
        telemetry.addData("TargetHeading: ", this.targetHeading);
        telemetry.addData("isFinished: ", this.isFinished());
        telemetry.addData("X: ", localization.getRobotPoseinFieldCoordinates().getX());
        telemetry.addData("Y: ", localization.getRobotPoseinFieldCoordinates().getY());
        telemetry.addData("Heading: ", localization.getRobotPoseinFieldCoordinates().getHeading());
        telemetry.update();
        if (motionProfiletheta != null) {
            MotionState xState = motionProfilex.get(timer.seconds());
            MotionState yState = motionProfiley.get(timer.seconds());
            MotionState thetaState = motionProfiletheta.get(timer.seconds());
            setPosition(xState.getX(), yState.getX(), thetaState.getX());
        }
    }

    public void goTo(double x, double y, double theta) {
        this.goTo(x, y, theta, 60, 90, 180, 180);
    }

    public void goTo(double x, double y, double theta, double maxVel, double maxAccel, double maxAngVel, double maxAngAccel) {
        targetX = x;
        targetY = y;
        targetHeading = theta;
        double currentHeading = localization.getRobotPoseinFieldCoordinates().getHeading();
        double currentx = localization.getRobotPoseinFieldCoordinates().getX();
        double currenty = localization.getRobotPoseinFieldCoordinates().getY();
        motionProfilex = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentx, 0, 0),
                new MotionState(x, 0, 0),
                maxVel,
                maxAccel
        );

        motionProfiley = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currenty, 0, 0),
                new MotionState(y, 0, 0),
                maxVel,
                maxAccel
        );

        motionProfiletheta = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentHeading, 0, 0),
                new MotionState(theta, 0, 0),
                maxAngVel,
                maxAngAccel
        );
        timer.reset();
    }

    public void coordinateSequencer(Pose2d... robotPosition) {
        for (Pose2d pathSegment : robotPosition) {
            goTo(pathSegment.getX(), pathSegment.getY(), pathSegment.getHeading());
            while (!this.isFinished()) {
                this.update();
            }
            localization.setHeadingSourceImu(true);
            timer.reset();
            while (timer.seconds() < 5) {
                this.update();
            }
            localization.setHeadingSourceImu(false);
        }
    }

    public boolean areWithinRange(double num1, double num2, double range) {
        double difference = Math.abs(num1 - num2);
        return difference <= range;
    }


}
