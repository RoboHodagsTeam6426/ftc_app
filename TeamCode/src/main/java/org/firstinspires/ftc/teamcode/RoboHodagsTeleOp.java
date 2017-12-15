package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by bachmhun on 1/3/2017.
 */

//for more detailed explanation of soem of the set up check out TestBotSensorTestAuto class/program
//name as a teleop program and name it
@TeleOp(name = "RoboHodagsTeleOp")
public class RoboHodagsTeleOp extends LinearOpMode {

    //declare private classes for each motor and servo
    private DcMotor motorLeftDrive;
    private DcMotor motorRightDrive;

    private DcMotor motorSpool;
    private Servo servoMand;
    private CRServo servoLeft;


    double servoMandPos = 0.2;
    double servoMandPosTwo = 0.5;

    /*double leftJoyPowerY;
    double rightPowerY;
    double leftJoyPowerX;
    double rightPowerX;
    double deadZone = 0.05;
    double leftDrivePower;
    double rightDrivePower;

    boolean controlTank = true;*/

    //class for what to do when opmode is run
    @Override
    public void runOpMode() throws InterruptedException {

        //tells program what name in the configuration hardware map corresponds to what class in the code from above
        motorLeftDrive = hardwareMap.dcMotor.get("motorLeftDrive");
        motorRightDrive = hardwareMap.dcMotor.get("motorRightDrive");

        servoMand = hardwareMap.servo.get("servoMand");
        motorSpool = hardwareMap.dcMotor.get("motorSpool");
        servoLeft = hardwareMap.crservo.get("servoLeft");


        //reverse right motor so both go forward
        motorRightDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        servoMand.setPosition(servoMandPos);

        //loop for what to do while opmode is running
        while (opModeIsActive()) {

            motorLeftDrive.setPower(gamepad1.left_stick_y);
            motorRightDrive.setPower(gamepad1.right_stick_y);

            motorSpool.setPower(gamepad2.right_stick_y);

            servoLeft.setPower(gamepad2.left_stick_y);

            if (gamepad1.b) {
                servoMand.setPosition(servoMandPosTwo);
            }

            /*telemetry.addData("Tank", controlTank);
            telemetry.update();
            //sets tank drive for the drive motors

            motorLeftDrive.setPower(-gamepad1.left_stick_y);
            motorRightDrive.setPower(-gamepad1.right_stick_y);

            motorMand.setPower(-gamepad2.right_stick_y);

            //if dpad right pressed it sets the grabbing servo to open

            //motorLeftDrive.setPower(-gamepad1.left_stick_y);
            //motorRightDrive.setPower(-gamepad1.right_stick_y);

            while (controlTank == false) {
                if (gamepad1.b) {
                    controlTank = true;
                }

                if (gamepad1.left_stick_y <= 0) {
                    leftDrivePower = gamepad1.left_stick_y - gamepad1.left_stick_x;
                    rightDrivePower = gamepad1.left_stick_y + gamepad1.left_stick_x;
                    if (gamepad1.b) {
                        controlTank = true;
                    }
                } else {
                    leftDrivePower = gamepad1.left_stick_y + gamepad1.left_stick_x;
                    rightDrivePower = gamepad1.left_stick_y - gamepad1.left_stick_x;
                    if (gamepad1.b) {
                        controlTank = true;
                    }
                }

                motorLeftDrive.setPower((leftDrivePower   /2) * 0.725);
                motorRightDrive.setPower(rightDrivePower /2 );

                motorSpool.setPower(gamepad1.right_stick_y);

                if (gamepad1.b) {
                    controlTank = true;
                }

                telemetry.addData("Tank", controlTank);
                telemetry.update();
            }

               /* leftJoyPowerY = gamepad1.left_stick_y;
                leftJoyPowerX = gamepad1.left_stick_x;

                //checks the values for if in the deadzone
                if (Math.abs(gamepad1.left_stick_y) < deadZone) {
                    leftJoyPowerY = 0;
                }

                if (Math.abs(gamepad1.left_stick_x) < deadZone) {
                    leftJoyPowerX = 0;
                }

                //if up on joystick and right on joystick
                if (gamepad1.left_stick_y >= 0 && gamepad1.left_stick_x >= 0) {
                    leftDrivePower = leftJoyPowerY - leftJoyPowerX;
                    rightDrivePower = leftJoyPowerY + leftJoyPowerX;

                } else if (gamepad1.left_stick_y >= 0 && gamepad1.left_stick_x  <= 0) {
                    leftDrivePower = leftJoyPowerY - leftJoyPowerX;
                    rightDrivePower = leftJoyPowerY + leftJoyPowerX;

                } else if (gamepad1.left_stick_y <= 0 && gamepad1.left_stick_x >= 0) {
                    leftDrivePower = leftJoyPowerY - leftJoyPowerX;
                    rightDrivePower = leftJoyPowerY + leftJoyPowerX;

                } else if (gamepad1.left_stick_y <= 0 && gamepad1.left_stick_x <= 0) {
                    leftDrivePower = leftJoyPowerY - leftJoyPowerX;
                    rightDrivePower = leftJoyPowerY + leftJoyPowerX;

                } else if (gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0) {
                    leftDrivePower = 0;
                    rightDrivePower = 0;
                }
                motorLeftDrive.setPower(-leftDrivePower);
                motorRightDrive.setPower(-rightDrivePower);
            */
            /*while(controlTank == true) {
                if (gamepad1.b) {
                    controlTank = true;
                }

                motorLeftDrive.setPower(gamepad1.left_stick_y * 0.725);
                motorRightDrive.setPower(gamepad1.right_stick_y);

                servoLeft.setPower(gamepad2.left_stick_y);

                if (gamepad1.a) {
                    servoMand.setPosition(servoMandPos);
                }

                if (gamepad1.b) {
                    controlTank = false;
                }

                telemetry.addData("Tank", controlTank);
                telemetry.update();
            }
            //while dpad up is pressed the mand lift motor will go up
            /*while (gamepad2.dpad_up) {
                motorMand.setPower(1);
            }
            motorMand.setPower(-gamepad2.right_stick_x);
            //while the dpad down is pressed the mand lift goes down
            /*while (gamepad1.dpad_down) {
                motorMand.setPower(-1);
            }

            //if dpad right pressed it sets the grabbing servo to said position
>>>>>>> 2f3e042b026657c3783f96392a56bc4ee2ebe278
            while (gamepad1.dpad_right) {
                servoMand.setPosition(servoMandPosTwo);
            }

            //if dpad left pressed sets the servo to closed
            if (gamepad1.dpad_left) {
                servoMand.setPosition(servoMandPos);

            }

            //flips down mandibles
            if (gamepad2.x) {
                servoMandFlip.setPosition(servoMandFlipPos);
                sleep(1000);
                servoMandFlip.setPosition(servoMandFlipPosTwo);
            }

            while (gamepad2.dpad_up) {
                servoPush.setPower(1);
            }

            while (gamepad2.dpad_down) {
                servoPush.setPower(-1);
            }

            }*/

         }
    }
}
