package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by bachmhun on 11/21/2017.
 */
@Autonomous (name = "Block")
public class Block extends LinearOpMode {

    private DcMotor leftDriveMotor;
    private DcMotor rightDriveMotor;

    public void runOpMode() throws InterruptedException{

        motorHardwareMap();
        motorReversal();

        waitForStart();

        turn(1);
        sleep(2000);
        driveForward(1);
        sleep(2000);
        turn(-1);
        sleep(4000);
        driveForward(1);
        sleep(4000);
        turn(1);
        sleep(2000);
        driveForward(1);
        sleep(10000);
        stopDriving();
    }

    public void motorHardwareMap() {
        leftDriveMotor = hardwareMap.dcMotor.get("leftDriveMotor");
        rightDriveMotor = hardwareMap.dcMotor.get("rightDriveMotor");
    }

    public void motorReversal() {
        rightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void stopDriving() {
        leftDriveMotor.setPower(0);
        rightDriveMotor.setPower(0);
    }

    public void tankDrive() {
        while (opModeIsActive()) {

            leftDriveMotor.setPower(gamepad1.left_stick_y);
            rightDriveMotor.setPower(gamepad1.right_stick_y);
        }
    }

    public void driveForward(double speed) {
        leftDriveMotor.setPower(speed);
        rightDriveMotor.setPower(speed);
    }

    public void turn(double turnRate) {
        leftDriveMotor.setPower(turnRate);
        rightDriveMotor.setPower(-turnRate);
    }

    public void driveBackwards(double speed) {
        leftDriveMotor.setPower(-speed);
        rightDriveMotor.setPower(-speed);

    }
}
