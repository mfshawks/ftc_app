/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This program moves robot forward 5 inches.
 */

@Autonomous(name="MEncoderTest")
public class MecanumAutoEncoderTest extends LinearOpMode {

    /* Declare OpMode members. */
    Mecanum1 robot   = new Mecanum1();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    /**
     * Neverest 60:
     * There is an encoder mounted to the back side of this motor.
     * It is a 7 pulse per revolution (ppr), hall effect encoder.
     * Since the motor's gearbox has a 60:1 reduction, then the NeverRest 60 output shaft provides 420 ppr.
     */
    static final double     COUNTS_PER_MOTOR_REV    = 420 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.LRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();


        // Step through each leg of the path
        encoderDriveMove(DRIVE_SPEED, direction.FORWARD, 5, 3);
    }

    public enum direction {
        FORWARD, BACKWARD, LEFT, RIGHT
    }

    public void encoderDriveMove(double speed,
                             direction direction,
                             double distanceInInch,
                             double timeoutS) {
        int newLeftRearTarget = 0;
        int newRightRearTarget = 0;
        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            switch (direction) {
                case LEFT:
                    newLeftRearTarget = robot.LRMotor.getCurrentPosition() + (int)(distanceInInch * COUNTS_PER_INCH);
                    newRightRearTarget = robot.RRMotor.getCurrentPosition() - (int)(distanceInInch * COUNTS_PER_INCH);
                    newLeftFrontTarget = robot.LFMotor.getCurrentPosition() - (int)(distanceInInch * COUNTS_PER_INCH);
                    newRightFrontTarget = robot.LFMotor.getCurrentPosition() + (int)(distanceInInch * COUNTS_PER_INCH);
                    break;
                case RIGHT:
                    newLeftRearTarget = robot.LRMotor.getCurrentPosition() - (int)(distanceInInch * COUNTS_PER_INCH);
                    newRightRearTarget = robot.RRMotor.getCurrentPosition() + (int)(distanceInInch * COUNTS_PER_INCH);
                    newLeftFrontTarget = robot.LFMotor.getCurrentPosition() + (int)(distanceInInch * COUNTS_PER_INCH);
                    newRightFrontTarget = robot.LFMotor.getCurrentPosition() - (int)(distanceInInch * COUNTS_PER_INCH);
                    break;
                case BACKWARD:
                    newLeftRearTarget = robot.LRMotor.getCurrentPosition() - (int)(distanceInInch * COUNTS_PER_INCH);
                    newRightRearTarget = robot.RRMotor.getCurrentPosition() - (int)(distanceInInch * COUNTS_PER_INCH);
                    newLeftFrontTarget = robot.LFMotor.getCurrentPosition() - (int)(distanceInInch * COUNTS_PER_INCH);
                    newRightFrontTarget = robot.LFMotor.getCurrentPosition() - (int)(distanceInInch * COUNTS_PER_INCH);
                    break;
                case FORWARD:
                    newLeftRearTarget = robot.LRMotor.getCurrentPosition() + (int)(distanceInInch * COUNTS_PER_INCH);
                    newRightRearTarget = robot.RRMotor.getCurrentPosition() + (int)(distanceInInch * COUNTS_PER_INCH);
                    newLeftFrontTarget = robot.LFMotor.getCurrentPosition() + (int)(distanceInInch * COUNTS_PER_INCH);
                    newRightFrontTarget = robot.LFMotor.getCurrentPosition() + (int)(distanceInInch * COUNTS_PER_INCH);
                    break;
            }
            // Determine new target position, and pass to motor controller

            robot.LRMotor.setTargetPosition(newLeftRearTarget);
            robot.RRMotor.setTargetPosition(newRightRearTarget);
            robot.LFMotor.setTargetPosition(newLeftFrontTarget);
            robot.RFMotor.setTargetPosition(newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            robot.LRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.LRMotor.setPower(Math.abs(speed));
            robot.LFMotor.setPower(Math.abs(speed));
            robot.RRMotor.setPower(Math.abs(speed));
            robot.RFMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.LRMotor.isBusy() && robot.RRMotor.isBusy()) && robot.LFMotor.isBusy() && robot.RFMotor.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newRightFrontTarget,  newLeftFrontTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            robot.RFMotor.getCurrentPosition(),
                                            robot.LFMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.LFMotor.setPower(0);
            robot.LRMotor.setPower(0);
            robot.RFMotor.setPower(0);
            robot.RRMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.LRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
