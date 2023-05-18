package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Drivetrain {

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;

    Telemetry telemetry;
    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void calculateMotorSpeeds(double forwardBackward, double strafe, double rotation) {
        double frontLeftSpeed = forwardBackward + strafe + rotation;
        double frontRightSpeed = forwardBackward - strafe - rotation;
        double rearLeftSpeed = forwardBackward - strafe + rotation;
        double rearRightSpeed = forwardBackward + strafe - rotation;

        // Normalize the speeds
        double maxSpeed = Math.max(Math.max(Math.abs(frontLeftSpeed), Math.abs(frontRightSpeed)),
                Math.max(Math.abs(rearLeftSpeed), Math.abs(rearRightSpeed)));

        if (maxSpeed > 1.0) {
            frontLeftSpeed /= maxSpeed;
            frontRightSpeed /= maxSpeed;
            rearLeftSpeed /= maxSpeed;
            rearRightSpeed /= maxSpeed;
        }

        // Calculate the actual motor speeds (in RPM or other units based on your motor specifications)
        double frontLeftMotorSpeed =  (frontLeftSpeed * 1);
        double frontRightMotorSpeed =  (frontRightSpeed * 1);
        double rearLeftMotorSpeed =  (rearLeftSpeed * 1);
        double rearRightMotorSpeed =  (rearRightSpeed * 1);

        // Set the motor speeds
        setMotorSpeed(frontLeftMotorSpeed, frontRightMotorSpeed, rearLeftMotorSpeed, rearRightMotorSpeed);
    }


    private void setMotorSpeed(double frontLeft, double frontRight, double rearLeft, double rearRight) {
        telemetry.addData("Left Front: ", leftFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Right Front: ", rightFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Left Rear: ", leftRear.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Right Rear: ", rightRear.getCurrent(CurrentUnit.AMPS));
        leftFront.setPower(frontLeft);
        leftRear.setPower(rearLeft);
        rightFront.setPower(frontRight);
        rightRear.setPower(rearRight);
    }
}
