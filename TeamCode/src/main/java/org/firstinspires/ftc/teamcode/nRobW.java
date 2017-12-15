package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Hunter on 6/29/2017.
 */
@TeleOp
public class nRobW extends LinearOpMode {

    private DcMotor leftDriveMotor;
    private DcMotor rightDriveMotor;

    private Servo grabServo;
    private CRServo armServo;

    double grabServoPos1 = 0.5;
    double grabServoPos2 = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {

        leftDriveMotor = hardwareMap.dcMotor.get("leftDriveMotor");
        rightDriveMotor = hardwareMap.dcMotor.get("rightDriveMotor");

        grabServo = hardwareMap.servo.get("grabServo");
        armServo = hardwareMap.crservo.get("armServo");

        rightDriveMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        grabServo.setPosition(grabServoPos1);

        while (opModeIsActive()) {
            leftDriveMotor.setPower(gamepad1.left_stick_y);
            rightDriveMotor.setPower(gamepad1.right_stick_y);

            armServo.setPower(gamepad2.left_stick_y);

            if(gamepad2.a) {
                grabServo.setPosition(grabServoPos2);
            }

            if (gamepad2.b) {
                grabServo.setPosition(grabServoPos1);
            }

        }
    }
}
