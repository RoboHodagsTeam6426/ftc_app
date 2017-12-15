package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by bachmhun on 1/4/2017.
 */
//sets what type of program and the name that will be displayed on the app
@Autonomous(name = "TestAuto")
//this makes a class called v that is displayed in the android tab and extends another class with more imports and set up to help you.
public class TestAuto extends LinearOpMode {

    //declare motor names in a private class
    private DcMotor leftDriveMotor;
    private DcMotor rightDriveMotor;
    private DcMotor topArmMotor;
    private DcMotor bottomArmMotor;

    private Servo ballServo;
    private Servo leftArmServo;
    private Servo rightArmServo;

    int elapsedTime = 0;
    boolean timeElasped = false;

    //private CRServo servoLeft;

    // color sensor name declaration
    ColorSensor colorSensor;

    //range sensor declaration
    ModernRoboticsI2cRangeSensor rangeSensor;

    //declare gyro in bottom class so as it is accessible by all other classes
    GyroSensor sensorGyro;
    ModernRoboticsI2cGyro gyro;

    //overrides earlier class so that it can exceptions which helps it be less finicky.
    @Override
    // creates a class v, that needs to say runOpMode and it throw unexpected answers so as not to freeze the program.
    public void runOpMode() throws  InterruptedException {

        //drive
        //corresponds the name in coding to the name in the configuration on phone
        leftDriveMotor = hardwareMap.dcMotor.get("leftDriveMotor");
        rightDriveMotor = hardwareMap.dcMotor.get("rightDriveMotor");
        topArmMotor = hardwareMap.dcMotor.get("topArmMotor");
        bottomArmMotor = hardwareMap.dcMotor.get("bottomArmMotor");
        //servoLeft = hardwareMap.crservo.get("servoLeft");
        //code name ^                  configuration name ^

        //reverse one motor so they drive the same direction

        //end of drive

        ballServo = hardwareMap.servo.get("ballServo");
        leftArmServo = hardwareMap.servo.get("leftArmServo");
        rightArmServo = hardwareMap.servo.get("rightArmServo");

        //MR color sensor
        //sets up an array of hsv values
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;

        //corresponds the name in coding to the name in phone. Just the same as the one above for the motors
        //but uses a different name v for what the actual hardware is (motor, servo, sensor, etc.)
        colorSensor = hardwareMap.colorSensor.get("sensorColor");

        //turns the led off when program is started.
        colorSensor.enableLed(true);
        //end of color sensor

        //finish setting up gyro values, target is target heading, and zaccumulated is the current heading
        sensorGyro = hardwareMap.gyroSensor.get("sensorGyro");
        gyro = (ModernRoboticsI2cGyro) sensorGyro;
        int zAccumulated;

        //displays green font while the gyro is calibrating=
        gyro.calibrate();

        // make sure the gyro is calibrated.
        while (opModeIsActive() && gyro.isCalibrating()) {
            telemetry.addData(">", "Gyro Calibrating. Do Not move!");
            telemetry.update();
            waitFunct(50);
        }

        //displays that it has finished calibrating
        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();
        //end of gyro

        //MR range sensor
        // get a reference to our compass
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensorRange");
        //end of range sensor

        waitForStart();

        //run methods for the program here.(what you want the robot to do)
        //ballServo.setPosition(1);
        rangeForwardU(33, 0, -0.5);
        turnAbsolute(90, 0.7);
        rangeForwardU(20, 90, -0.4);
        turnAbsolute(160, 0.8);
        leftDriveMotor.setPower(-0.5);
        rightDriveMotor.setPower(0.5);
        waitFunct(500);
        stopDriving();
        //colorSensorB();

    }

        //this is an example of a method.
        // turns off of current zero/direction currently facing WARNING WILL ACCUMULATE ERROR
        public void turn (int target, double power) throws InterruptedException {

            turnAbsolute(target + gyro.getHeading(), power);
        }

        //turns off of absolute zero and started heading
        public void turnAbsolute (int target, double power) throws InterruptedException {
            //gets current heading
            int heading = gyro.getHeading();

            double calcTurnSpeed = 0.0;

            //gets the current heading related to the target heading and if it is over the leniency (the number) it does the loop
            //sets the motor power proportional to far away from the heading it is, the farther away the faster it turns
            while ((Math.abs(heading - target)) > 5) {

                heading = gyro.getHeading();

                double leftPower;
                double rightPower;

                /*calcTurnSpeed = ;

                //calcTurnSpeed = Range.clip(calcTurnSpeed, -0.3, 0.3);

                //both ifs determine which direction to turn to get the fastest route back to target
                if (heading > target) {

                    leftDriveMotor.setPower(turnSpeed);
                    rightDriveMotor.setPower(turnSpeed);
                } else {

                    leftDriveMotor.setPower(turnSpeed);
                    rightDriveMotor.setPower(turnSpeed);
                }*/
                while (((heading - target) > -180) && ((heading - target) < 0)) {
                    heading = gyro.getHeading();

                    leftPower = (power + ((heading - target) * -1)) / 80;
                    rightPower = (power + ((heading - target) * -1)) / 80;

                    leftPower = Range.clip(leftPower, -1, 1);
                    rightPower = Range.clip(rightPower, -1, 1);

                    leftDriveMotor.setPower(-leftPower);
                    rightDriveMotor.setPower(-rightPower);
                }
                while ((heading -target) <= -180) {
                    heading = gyro.getHeading();

                    leftPower = (power + ((heading - target) * -1)) / 80;
                    rightPower = (power + ((heading - target) * -1)) / 80;

                    leftPower = Range.clip(leftPower, -1, 1);
                    rightPower = Range.clip(rightPower, -1, 1);

                    leftDriveMotor.setPower(leftPower);
                    rightDriveMotor.setPower(rightPower);
                }
                while (((heading - target) < 180) && ((heading - target) > 0)) {
                    heading = gyro.getHeading();

                    leftPower = (power + (heading - target)) / 80;
                    rightPower = (power + (heading - target)) / 80;

                    leftPower = Range.clip(leftPower, -1, 1);
                    rightPower = Range.clip(rightPower, -1, 1);

                    leftDriveMotor.setPower(leftPower);
                    rightDriveMotor.setPower(rightPower);
                }
                while ((heading -target) >= 180) {
                    heading = gyro.getHeading();

                    leftPower = (power + (heading - target)) / 80;
                    rightPower = (power + (heading - target)) / 80;

                    leftPower = Range.clip(leftPower, -1, 1);
                    rightPower = Range.clip(rightPower, -1, 1);

                    leftDriveMotor.setPower(-leftPower);
                    rightDriveMotor.setPower(-rightPower);
                }
            }
            //stops motors if it is within leniency
            stopDriving();
            return;
        }

    //stop driving in case a class does not have a stop feature or run for time feature
        public void stopDriving() throws InterruptedException {
            leftDriveMotor.setPower(0);
            rightDriveMotor.setPower(0);
        }

    public void waitFunct(int milliSec) {
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (opModeIsActive() && timer.time() < milliSec) {
            idle();
        }
    }
        //checks the ball for blue
        public void colorSensorB(double turnSpeed, double power) throws InterruptedException {
            //if it is it runs hit the button
            //checks for red and blue if neither present waits
            if (colorSensor.blue() == 0 && colorSensor.red() == 0) {
                waitFunct(1000);
            }
            //checks for red if red hits button
            if (colorSensor.blue() >= 1) {
                turnAbsolute(90, power);
            }
            //if it gets blue it readjusts so its on the other color of the beacon
            else {
                turnAbsolute(100, power);
                ballServo.setPosition(0);
            }
            stopDriving();
            return;
        }

        //runs the robot forward with autocorrect steering until it reaches a certain distance
        public void rangeForwardU(int targetRange, double target, double power) throws InterruptedException {
            //if the range is greater then the target range it keeps driving it checks for the ultrasonic
            while (targetRange < rangeSensor.cmUltrasonic()) {

                int heading = gyro.getHeading();

                double leftSpeed;
                double rightSpeed;

                //sets a leniency and if it goes outside it sets the speed to keep moving forward but
                //proportionalizes both sides so that it turns at different rates proportional to how
                //far away from the target it is
                if ((heading - target) > 5 && (heading - target) <= 180) {
                    leftSpeed = (power + (heading - target)) / 80;
                    rightSpeed = (power + (heading - target)) / 80;

                    leftSpeed = Range.clip(leftSpeed, -1, 1);
                    rightSpeed = Range.clip(rightSpeed, -1, 1);

                    leftDriveMotor.setPower(leftSpeed);
                    rightDriveMotor.setPower(rightSpeed);
                }
                if ((heading - target) < 355 && (heading - target) >180) {
                    leftSpeed = (power + (((heading - 360) * -1) - target)) / 80;
                    rightSpeed = (power + (((heading - 360) * -1) - target)) / 80;

                    leftSpeed = Range.clip(leftSpeed, -1, 1);
                    rightSpeed = Range.clip(rightSpeed, -1, 1);

                    leftDriveMotor.setPower(-leftSpeed);
                    rightDriveMotor.setPower(-rightSpeed);
                }
                //sets the speed for when inside of the leniency
                if (Math.abs(heading - target) < 10) {
                    leftDriveMotor.setPower(power);
                    rightDriveMotor.setPower(-power);
                }
            }
            stopDriving();
            return;
        }
        public void rangeBackwardU(int targetRange, double target, double power) throws InterruptedException {
            //if the range is greater then the target range it keeps driving it checks for the ultrasonic
            while (targetRange > rangeSensor.cmUltrasonic()) {

                int heading = gyro.getHeading();

                double leftSpeed;
                double rightSpeed;

                //sets a leniency and if it goes outside it sets the speed to keep moving forward but
                //proportionalizes both sides so that it turns at different rates proportional to how
                //far away from the target it is
                if ((heading - target) > 5 && (heading - target) >180) {
                    leftSpeed = (power + (heading - target)) / 90;
                    rightSpeed = (power + (heading - target)) / 90;

                    leftSpeed = Range.clip(leftSpeed, -1, 1);
                    rightSpeed = Range.clip(rightSpeed, -1, 1);

                    leftDriveMotor.setPower(leftSpeed);
                    rightDriveMotor.setPower(-rightSpeed);
                }
                if ((heading - target) < 355 && (heading - target) >180) {
                    leftSpeed = (power + (((heading - 360) * -1) - target)) / 90;
                    rightSpeed = (power + (((heading - 360) * -1) - target)) / 90;

                    leftSpeed = Range.clip(leftSpeed, -1, 1);
                    rightSpeed = Range.clip(rightSpeed, -1, 1);

                    leftDriveMotor.setPower(leftSpeed);
                    rightDriveMotor.setPower(rightSpeed);
                }
                //sets the speed for when inside of the leniency
                if (Math.abs(heading - target) < 10) {
                    leftDriveMotor.setPower(-power);
                    rightDriveMotor.setPower(power);
                }
            }
            stopDriving();

            return;
        }
        //same as for the ultrasonic but for close range with the optical
        public void rangeForwardO(int targetRange, double target, double power) throws InterruptedException {
        while (targetRange < rangeSensor.cmOptical()) {

            int heading =gyro.getHeading();

            double leftSpeed;
            double rightSpeed;

            if ((heading - target) > 5 && (heading - target) >180) {
                leftSpeed = (power + (heading - target)) / 90;
                rightSpeed = (power + (heading - target)) / 90;

                leftSpeed = Range.clip(leftSpeed, -1, 1);
                rightSpeed = Range.clip(rightSpeed, -1, 1);

                leftDriveMotor.setPower(leftSpeed);
                rightDriveMotor.setPower(rightSpeed);
            }
            if ((heading - target) < 355 && (heading - target) >180) {
                leftSpeed = (power + (((heading - 360) * -1) - target)) / 90;
                rightSpeed = (power + (((heading - 360) * -1) - target)) / 90;

                leftSpeed = Range.clip(leftSpeed, -1, 1);
                rightSpeed = Range.clip(rightSpeed, -1, 1);

                leftDriveMotor.setPower(-leftSpeed);
                rightDriveMotor.setPower(-rightSpeed);
            }
            if (Math.abs(heading - target) < 10) {
                leftDriveMotor.setPower(power);
                rightDriveMotor.setPower(-power);
            }
        }
        stopDriving();
        return;
    }
    public void rangeBackwardO(int targetRange, double target, double power) throws InterruptedException {
        while (targetRange > rangeSensor.cmOptical()) {

            int heading = gyro.getHeading();

            double leftSpeed;
            double rightSpeed;

            if ((heading - target) > 5 && (heading - target) >180) {
                leftSpeed = (power + (heading - target)) / 90;
                rightSpeed = (power + (heading - target)) / 90;

                leftSpeed = Range.clip(leftSpeed, -1, 1);
                rightSpeed = Range.clip(rightSpeed, -1, 1);

                leftDriveMotor.setPower(leftSpeed);
                rightDriveMotor.setPower(-rightSpeed);
            }
            if ((heading - target) < 355 && (heading - target) >180) {
                leftSpeed = (power + (((heading - 360) * -1) - target)) / 90;
                rightSpeed = (power + (((heading - 360) * -1) - target)) / 90;

                leftSpeed = Range.clip(leftSpeed, -1, 1);
                rightSpeed = Range.clip(rightSpeed, -1, 1);

                leftDriveMotor.setPower(leftSpeed);
                rightDriveMotor.setPower(rightSpeed);
            }
            if (Math.abs(heading - target) < 10) {
                leftDriveMotor.setPower(-power);
                rightDriveMotor.setPower(power);
            }
        }
        stopDriving();
        return;
    }
}
