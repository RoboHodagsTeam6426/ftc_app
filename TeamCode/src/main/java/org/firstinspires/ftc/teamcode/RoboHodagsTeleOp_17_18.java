package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by bachmhun on 11/9/2017.
 */
@TeleOp(name = "RoboHodagsTeleOp_17_18")
public class RoboHodagsTeleOp_17_18 extends LinearOpMode{

    private DcMotor leftDriveMotor;
    private DcMotor rightDriveMotor;
    private DcMotor topArmMotor;
    private DcMotor bottomArmMotor;
    private DcMotor winchMotor;
    private DcMotor scissorMotor;

    private Servo rightArmServo;
    private Servo leftArmServo;
    private Servo ballServo;

    double rightArmServoClosed = 0.5;
    double leftArmServoClosed = 0.5;
    double rightArmServoPart = 0.7;
    double leftArmServoPart = 0.3;
    double rightArmServoOpen = 0.9;
    double leftArmServoOpen = 0.1;

    double trimValueR = 1;
    double trimValueL = 1;

    double drivePower = 1;

    double leftDrivePower;
    double rightDrivePower;

    boolean pause = false;

    public void runOpMode() throws  InterruptedException {

        leftDriveMotor = hardwareMap.dcMotor.get("leftDriveMotor");
        rightDriveMotor = hardwareMap.dcMotor.get("rightDriveMotor");
        topArmMotor = hardwareMap.dcMotor.get("topArmMotor");
        bottomArmMotor = hardwareMap.dcMotor.get("bottomArmMotor");

        rightArmServo = hardwareMap.servo.get("rightArmServo");
        leftArmServo = hardwareMap.servo.get("leftArmServo");
        ballServo = hardwareMap.servo.get("ballServo");

        winchMotor = hardwareMap.dcMotor.get("winchMotor");
        scissorMotor = hardwareMap.dcMotor.get("scissorMotor");

        leftDriveMotor.setDirection(DcMotor.Direction.REVERSE);

        //leftArmServo.setPower(0);
        //rightArmServo.setPower(0);

        waitForStart();

        ballServo.setPosition(0);

        while (opModeIsActive()) {

            leftDrivePower = Range.clip(gamepad1.left_stick_y, -trimValueL, trimValueL);
            rightDrivePower = Range.clip(gamepad1.right_stick_y, -trimValueR, trimValueR);

            leftDriveMotor.setPower(leftDrivePower * drivePower);
            rightDriveMotor.setPower(rightDrivePower * drivePower);

            topArmMotor.setPower(gamepad2.left_stick_y/4);
            bottomArmMotor.setPower(gamepad2.left_stick_y/4);

            //leftArmServo.setPower(-gamepad2.right_stick_y);
            //rightArmServo.setPower(gamepad2.right_stick_y);

            scissorMotor.setPower(gamepad2.right_stick_y/10);

            if (gamepad2.right_bumper) {
                topArmMotor.setPower(-0.03);
                bottomArmMotor.setPower(-0.03);
            }

            if (gamepad2.dpad_right) {
                ballServo.setPosition(0.4);
            }

            if (gamepad2.dpad_up) {
                winchMotor.setPower(-0.5);
            }else if(gamepad2.dpad_down) {
                winchMotor.setPower(0.5);
            }else if(gamepad2.dpad_left) {
                winchMotor.setPower(0.04);
            }else {
                winchMotor.setPower(0);
            }

            if (gamepad2.x) {
                leftArmServo.setPosition(leftArmServoOpen);
                rightArmServo.setPosition(rightArmServoOpen);
            }

            if (gamepad2.b) {
                leftArmServo.setPosition(leftArmServoClosed);
                rightArmServo.setPosition(rightArmServoClosed);
            }

            if (gamepad2.y) {
                leftArmServo.setPosition(leftArmServoPart);
                rightArmServo.setPosition(rightArmServoPart);
            }

            if (gamepad1.dpad_left) {
                drivePower = 1;
            }

            if (gamepad1.dpad_right) {
                drivePower = 0.5;
            }

            //drive motor trim
            if (gamepad1.x && pause != true) {
                if (gamepad1.dpad_up) {
                    trimValueL = trimValueL + 0.05;
                    Range.clip(trimValueL, -1, 1);
                    pause = true;
                }
                else if (gamepad1.dpad_down) {
                    trimValueL = trimValueL - 0.05;
                    Range.clip(trimValueL, -1, 1);
                    pause = true;
                }
            }

            if (gamepad1.b && pause != true) {
                if (gamepad1.dpad_up) {
                    trimValueR = trimValueR + 0.05;
                    Range.clip(trimValueR, -1, 1);
                    pause = true;
                }
                else if (gamepad1.dpad_down) {
                    trimValueR = trimValueR - 0.05;
                    Range.clip(trimValueR, -1, 1);
                    pause = true;
                }
            }

            //makes sure dpad is unpressed before it can be pressed again
            if (!gamepad1.dpad_down && !gamepad1.dpad_up) {
                pause = false;
            }
        }
    }
}
