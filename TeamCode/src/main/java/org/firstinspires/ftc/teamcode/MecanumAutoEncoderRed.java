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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;



@Autonomous(name="M Encoder Red")
public class MecanumAutoEncoderRed extends LinearOpMode {

    /* Declare OpMode members. */
    Mecanum1 robot   = new Mecanum1();   // Use Mecanum 1 robot
    private ElapsedTime runtime = new ElapsedTime();

    // OpenGLMatrix lastLocation = null;
    // int a;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        MecanumDrive mecanumDrive = new MecanumDrive(robot, this);

        mecanumDrive.resetEncoders();
        mecanumDrive.gyroInit();

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
        robot.RHand.setPosition(0.8); //arm \ /
        robot.LHand.setPosition(0.8);
        robot.LClaw.setPosition(0.0); //arm up
        robot.RClaw.setPosition(0.0);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
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

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; // Use back camera
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();
//>>>>>>>>>>>>>>>>>>>>>>>>>>START>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

        relicTrackables.activate();

        RelicRecoveryVuMark column = RelicRecoveryVuMark.UNKNOWN;

        runtime.reset();
        robot.LClaw.setPosition(0.5);
        robot.RClaw.setPosition(0.5);
        while (opModeIsActive() && runtime.seconds() < 1.5) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);
                column = vuMark;

                break; // Break the while loop after the vuMark is found
            } else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }

        robot.RHand.setPosition(1.0);
        robot.LHand.setPosition(1.0);

        robot.arm.setPosition(0.7);
        mecanumDrive.liftMotorDrive(1.0, 9, 5); // Lift the block up 9 inches
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Move sensor arm: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("Red", String.valueOf(robot.armColorSensor.red()));
            telemetry.addData("Blue", String.valueOf(robot.armColorSensor.blue()));
            switch (column) {
                case RIGHT:
                    telemetry.addData("Position",new Func<String>() {
                        @Override public String value() {
                            return "Right";
                        }
                    });
                    break;
                case LEFT:
                    telemetry.addData("Position",new Func<String>() {
                        @Override public String value() {
                            return "Left";
                        }
                    });
                    break;
                case CENTER:
                    telemetry.addData("Position",new Func<String>() {
                        @Override public String value() {
                            return "Center";
                        }
                    });
                    break;
                case UNKNOWN:
                    telemetry.addData("Position",new Func<String>() {
                        @Override public String value() {
                            return "Unknown";
                        }
                    });
                    break;
            }
            telemetry.update();
        }

        boolean isRed = (robot.armColorSensor.red() > robot.armColorSensor.blue());
        double backwardInch = 27; // The distance to move forward afterward
        if (isRed) {
            // Move back
            mecanumDrive.encoderDriveMove(0.3, direction.BACKWARD, 3, 1);
            backwardInch -= 3;
        } else {
            // Move forward
            mecanumDrive.encoderDriveMove(0.3, direction.FORWARD, 3, 1);
            backwardInch += 3;
        }

        robot.arm.setPosition(0.0);
        mecanumDrive.encoderDriveMove(1.0, direction.BACKWARD, backwardInch, 5);

        double distanceToTheRight = 15.5;
        switch (column) {
            case RIGHT:
                distanceToTheRight = 8.0;
                break;
            case LEFT:
                distanceToTheRight = 24.0;
                break;
            case CENTER:
                distanceToTheRight = 16.0;
                break;
            case UNKNOWN:
                break;
        }

        mecanumDrive.encoderDriveMove(0.5, direction.RIGHT, distanceToTheRight, 4);

        mecanumDrive.gyroTurn(0.8, 180);

        mecanumDrive.encoderDriveMove(0.7, direction.FORWARD, 12, 3);

        robot.RHand.setPosition(0.8); //arm \ /
        robot.LHand.setPosition(0.8);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Move arm apart: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        mecanumDrive.encoderDriveMove(0.3, direction.BACKWARD, 2, 1);

        sleep(1000);

        mecanumDrive.encoderDriveMove(0.3, direction.FORWARD, 2, 1);

        mecanumDrive.liftMotorDrive(0.8, -5, 1);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

//    String format(OpenGLMatrix transformationMatrix) {
//        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
//    }
}
