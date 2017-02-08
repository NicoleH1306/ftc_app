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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
   The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
   that performs the actual movement.

   This methods assumes that each movement is relative to the last stopping place.
 */

@Autonomous(name="BlueCapParkP1", group="Cap")
//Disabled
public class BlueCapParkP1 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //Declaring drivetrain motors
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;



    static final double     COUNTS_PER_MOTOR_REV    = 1100 ;    // 1100 encoder points per rotation
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No gear reduction because of direct drive
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

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        encoderDrive(0.5,  27,  27, 27, 27, 5.0);  // S1: Forward 48 Inches with 5 Sec timeout
        sleep(100);
        encoderDrive(0.5, 10, -10, 10, -10, 3.0);
        sleep(100);
        encoderDrive(0.5, 44, 44, 44, 44, 3.0);
        sleep(100);


        // encoderDrive(0, 0, 0, 0, 0, 5.0);


        sleep(1000);     // pause for servos to move

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
