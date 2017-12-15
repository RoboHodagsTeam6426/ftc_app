package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by bachmhun on 2/2/2017.
 */
@Disabled
public class ConorPractice extends LinearOpMode {
    private DcMotor leftDriveMotor;
    private DcMotor rightDriveMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        leftDriveMotor = hardwareMap.dcMotor.get("leftDriveMotor");
        rightDriveMotor = hardwareMap.dcMotor.get("rightDriveMotor");

        rightDriveMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        forwardDrive(1);
        Thread.sleep(2);  // 2 seconds until stopped
        while (opModeIsActive()) {
            leftDriveMotor.setPower(gamepad1.left_stick_y);
            rightDriveMotor.setPower(gamepad1.right_stick_y);

        }


    }

    public void forwardDrive(double power){
        leftDriveMotor.setPower(power);
        rightDriveMotor.setPower(power);
    }
}
