/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="RedBeacon2P2Line", group="Nicole")
@Disabled
public class RedBeacon2P2Line extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;

    ColorSensor colorSensor;

    OpticalDistanceSensor lineSensor;





    static final double     COUNTS_PER_MOTOR_REV    = 1100 ;    // eg: TETRIX Motor Encoder 1440
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);


    @Override
    public void runOpMode() throws InterruptedException {

         /* Declare OpMode members. */
        leftFrontMotor  = hardwareMap.dcMotor.get("left front motor");
        rightFrontMotor = hardwareMap.dcMotor.get("right front motor");
        leftBackMotor = hardwareMap.dcMotor.get("left back motor");
        rightBackMotor = hardwareMap.dcMotor.get("right back motor");

        colorSensor = hardwareMap.colorSensor.get("beacon color");
        lineSensor = hardwareMap.opticalDistanceSensor.get("ods");

        colorSensor.enableLed(false);

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        //private ElapsedTime     runtime = new ElapsedTime();

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d :%7d :%7d",
                          rightBackMotor.getCurrentPosition(), leftBackMotor.getCurrentPosition(), leftFrontMotor.getCurrentPosition(), rightFrontMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        double baseV = lineSensor.getRawLightDetected();


        //Getting to the first beacon
        encoderDrive(0.4,  27,  27, 27, 27, 5.0); //Drive forward 27 inches
        sleep(200);
        encoderDrive(0.4, 24, -24, 24, -24, 3.0); //Turn to face the beacon
        sleep(200);
        encoderDrive(0.4, -40, -40, -40, -40, 3.0); //Drive forward 47 inches to the beacon
        sleep(200);
        encoderDrive(0.4, 24, -24, 24, -24, 3.0); //Turn robot to be pararell with beacon
        sleep(200);

        while(baseV + 1 > lineSensor.getRawLightDetected())
        {
            leftFrontMotor.setPower(-0.3);
            rightFrontMotor.setPower(-0.3);
            leftBackMotor.setPower(-0.3);
            rightBackMotor.setPower(-0.3);

            telemetry.addData("ods", "base at %7f", baseV);
            telemetry.addData("ods", "value at %7f", lineSensor.getRawLightDetected());
            telemetry.update();
        }

        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);

        //sleep(1000);

        /*encoderDrive(0.4, 5, 5, 5, 5, 3.0); //Move forward to be allign color sensor with first color
        sleep(200);
        encoderDrive(0.4, 5, -5, -5, 5, 0.3); //Move sideways toward beacon to read color
        sleep(200);

        //First Beacon
        if(colorSensor.blue() > 2){
            encoderDrive(0.3, -3, -3, -3, -3, 3.0);
            sleep(200);
            encoderDrive(0.3, 3.5, -3.5, -3.5, 3.5, 3.0);
            sleep(200);
            encoderDrive(0.3, -4, 4, 4, -4, 3.0);
            sleep(200);
            encoderDrive(0.4, 54, 54, 54, 54, 3.0);
            sleep(200);
        }
        else if(colorSensor.red() > 2)
        {
            encoderDrive(0.3, 5, 5, 5, 5, 3.0);
            sleep(200);
            if(colorSensor.blue() > 2)
            {
                encoderDrive(0.3, 3, 3, 3, 3, 3.0);
                sleep(200);
                encoderDrive(0.3, 3, -3, -3, 3, 3.0);
                sleep(200);
                encoderDrive(0.3, -5, 5, 5, -5, 0.3);
            }
            encoderDrive(0.4, 42.5, 42.5, 42.5, 42.5, 3.0);
            sleep(200);

        }
        else
        {
            encoderDrive(0, 0, 0, 0, 0, 0);
        }


        //Second Beacon
        if(colorSensor.blue() > 2){
            encoderDrive(0.3, -3, -3, -3, -3, 3.0);
            sleep(200);
            encoderDrive(0.3, 4, -4, -4, 4, 3.0);
            sleep(200);

        }
        else if(colorSensor.red() > 2)
        {
            encoderDrive(0.3, 5, 5, 5, 5, 3.0);
            sleep(200);
            if(colorSensor.blue() > 2)
            {
                encoderDrive(0.3, 4, 4, 4, 4, 3.0);
                sleep(200);
                encoderDrive(0.3, 3, -3, -3, 3, 3.0);
                sleep(200);
            }

        }
        else
        {
            encoderDrive(0, 0, 0, 0, 0, 0);
        }





        /*if(colorSensor.blue() > 2){
            encoderDrive(0.3, 44, 44, 44, 44, 3.0);
            sleep(100);
        }
        else if(colorSensor.red() > 2)
        {
            encoderDrive(0, 0, 0, 0, 0, 0);
            telemetry.addData("Error", "Unable to find blue light");
        }*/



        sleep(3000);
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftFrontInches, double rightFrontInches,
                             double leftBackInches, double rightBackInches,
                             double timeoutS) throws InterruptedException {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFrontMotor.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFrontMotor.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget =  leftBackMotor.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBackMotor.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);

            leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            rightFrontMotor.setTargetPosition(newRightFrontTarget);
            leftBackMotor.setTargetPosition(newLeftBackTarget);
            rightBackMotor.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.F
            runtime.reset();
            leftFrontMotor.setPower(Math.abs(speed));
            rightFrontMotor.setPower(Math.abs(speed));
            leftBackMotor.setPower(Math.abs(speed));
            rightBackMotor.setPower(Math.abs(speed));



            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive()/* &&
                   (runtime.seconds() < timeoutS)*/ &&
                   (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() && leftBackMotor.isBusy() && rightBackMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d, :%7d :%7d", newLeftFrontTarget,  newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                                            leftFrontMotor.getCurrentPosition(),
                                            rightFrontMotor.getCurrentPosition(),
                                            leftBackMotor.getCurrentPosition(),
                                            rightBackMotor.getCurrentPosition());
                telemetry.addData("Power",  "Power at %7f :%7f %7f %7f",
                        leftFrontMotor.getPower(),
                        rightFrontMotor.getPower(),
                        leftBackMotor.getPower(),
                        rightBackMotor.getPower());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightBackMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
