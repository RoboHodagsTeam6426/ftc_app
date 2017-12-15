package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by bachmhun on 11/9/2017.
 */

@TeleOp(name = "Teacher")
public class Teacher extends LinearOpMode {

    private DcMotor leftDriveMotor;
    private DcMotor rightDrivemotor;

    @Override
    public void runOpMode() throws InterruptedException {

        leftDriveMotor = hardwareMap.dcMotor.get("leftDriveMotor");
        rightDrivemotor = hardwareMap.dcMotor.get("rightDriveMotor");

        rightDrivemotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        /*
        turn(1);
        sleep(1000);
        stopDriving();
        */
        while(opModeIsActive()) {

            leftDriveMotor.setPower(gamepad1.left_stick_y);
            rightDrivemotor.setPower(gamepad1.right_stick_y);
        }

    }
    /*
    public void turn(double turnValue) {
        leftDriveMotor.setPower(turnValue);
        rightDrivemotor.setPower(-turnValue);
    }

    public void stopDriving() {
        leftDriveMotor.setPower(0);
        rightDrivemotor.setPower(0);
    }
*/
}

