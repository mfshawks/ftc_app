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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;



@Autonomous(name="M Encoder Red")
public class MecanumAutoEncoderRed extends LinearOpMode {

    /* Declare OpMode members. */
    Mecanum1 robot   = new Mecanum1();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1718 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    OpenGLMatrix lastLocation = null;
    int a;
    VuforiaLocalizer vuforia;

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
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.LRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          robot.LRMotor.getCurrentPosition(),
                          robot.RRMotor.getCurrentPosition());
        telemetry.update();

        robot.LFMotor.setPower(0);
        robot.RFMotor.setPower(0);
        robot.LRMotor.setPower(0);
        robot.RRMotor.setPower(0);
        robot.arm.setPosition(0.0);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AQRju9v/////AAAAGYVVWUuAKEAPph69Ouwpq+8CMlbZi/zomEEP" +
                "MAzyzRvCU9xj4/W+fiPSxFB1xv8BdlL55c6vD9wbMdHCPpMKKIqrNIJpXw06LgCF8xDeBXJEOEo" +
                "oeyamPY8gLwHvkHDA0EEP52F+b7J1IDRbmJedlK6GdOIYFLiSBVISCf0+vdiWJvJiWiPTgggSiC" +
                "RsV8IsK/OYwOqv5VoPv/m7no+VACFqPTcsKaAv5F49zqYXIncFbNKv9onHg5CEkZ4aMf7D/zcAp" +
                "hCO5Gb3BN+DtyUmrHM4oALhoMFgqRw59plwzxfD45Uzfyu6Jn2k7LPTCqs94SpprMfOqOotLtBHx" +
                "T19Rkl5toHI5buxqLlQDOX8y/jQ";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        robot.RHand.setPosition(0.8); //arm \ /
        robot.LHand.setPosition(0.8);
        robot.Lclaw.setPosition(0.0); //arm up
        robot.Rclaw.setPosition(0.0);
        waitForStart();
//>>>>>>>>>>>>>>>>>>>>>>>>>>START>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

        relicTrackables.activate();

        RelicRecoveryVuMark column = RelicRecoveryVuMark.UNKNOWN;

        runtime.reset();
        robot.Lclaw.setPosition(0.5);
        robot.Rclaw.setPosition(0.5);
        while (opModeIsActive() && runtime.seconds() < 3.0) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);
                column = vuMark;

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            } else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }

        robot.RHand.setPosition(1.0);
        robot.LHand.setPosition(1.0);
        //encoderDriveMove(0.3, direction.RIGHT, 3, 3);

        robot.arm.setPosition(0.7);
        liftMotorDrive(1.0, 20, 5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Move sensor arm: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("Red", String.valueOf(robot.armColorSensor.red()));
            telemetry.addData("Blue", String.valueOf(robot.armColorSensor.blue()));
            telemetry.update();
        }

        boolean isRed = (robot.armColorSensor.red() > robot.armColorSensor.blue());
        double backwardInch = 24; // The distance to move forward afterward
        if (isRed) {
            // Move back
            encoderDriveMove(0.3, direction.BACKWARD, 3, 1);
            backwardInch -= 3;
        } else {
            // Move forward
            encoderDriveMove(0.3, direction.FORWARD, 3, 1);
            backwardInch += 3;
        }

        robot.arm.setPosition(0.0);
        encoderDriveMove(1.0, direction.BACKWARD, backwardInch, 5);

        double distanceToTheLeft = 15.5;
        switch (column) {
            case RIGHT:
                distanceToTheLeft = 7.5;
                break;
            case LEFT:
                distanceToTheLeft = 23.5;
                break;
            case CENTER:
                distanceToTheLeft = 15.5;
                break;
            case UNKNOWN:
                break;
        }

        encoderDriveMove(0.5, direction.RIGHT, distanceToTheLeft, 5);

        telemetry.addData("Path", "Complete");
        telemetry.update();
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
                   robot.LRMotor.isBusy() && robot.LFMotor.isBusy() && robot.LRMotor.isBusy()) {

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



    public void liftMotorDrive(double speed,
                                 double distanceInInch,
                                 double timeoutS) {
        int newLiftTarget = 0;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            newLiftTarget = robot.tiltMotor.getCurrentPosition() + (int)(distanceInInch * COUNTS_PER_INCH);
            // Determine new target position, and pass to motor controller

            robot.tiltMotor.setTargetPosition(newLiftTarget);

            // Turn On RUN_TO_POSITION
            robot.tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.tiltMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    robot.tiltMotor.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Lift mptor running to %7d", newLiftTarget);
                telemetry.addData("Path2",  "Running at %7d",
                        robot.tiltMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.tiltMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.tiltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}