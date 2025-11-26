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

package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClasses.basicHardware;

/**
 * This is our basic TeleOp - just a way for the four wheels on the robot to move in all directions.
 * It is not meant to have any attachments move or even link to odometers - just a basic TeleOp.
 */

@TeleOp(name="basicTele", group="Linear Opmode")

public class basicTele extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private basicHardware robot = new basicHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double verticalComponent = -gamepad1.left_stick_y;
            double lateralComponent = gamepad1.left_stick_x;
            double turnComponent = gamepad1.right_stick_x;
            double verticalComponent2 = -gamepad2.left_stick_y;
            double lateralComponent2 = gamepad2.left_stick_x;
            double turnComponent2 = gamepad2.right_stick_x;

            boolean slowFront = gamepad2.dpad_up;
            boolean slowBack = gamepad2.dpad_down;
            boolean slowLeft = gamepad2.dpad_left;
            boolean slowRight = gamepad2.dpad_right;

            double SPEED_MULTIPLIER = 0.9;


            double normalizingFactor = Math.max(Math.abs(verticalComponent)
                    + Math.abs(lateralComponent) + Math.abs(turnComponent), 1);

            double fl = SPEED_MULTIPLIER * (verticalComponent + lateralComponent + turnComponent) / normalizingFactor;
            double fr = SPEED_MULTIPLIER * (verticalComponent - lateralComponent - turnComponent) / normalizingFactor;
            double bl = SPEED_MULTIPLIER * (verticalComponent - lateralComponent + turnComponent) / normalizingFactor;
            double br = SPEED_MULTIPLIER * (verticalComponent + lateralComponent - turnComponent) / normalizingFactor;
            fl += 0.4 * (verticalComponent2 + lateralComponent2 + turnComponent2) / normalizingFactor;
            fr += 0.4 * (verticalComponent2 - lateralComponent2 - turnComponent2) / normalizingFactor;
            bl += 0.4 * (verticalComponent2 - lateralComponent2 + turnComponent2) / normalizingFactor;
            br += 0.4 * (verticalComponent2 + lateralComponent2 - turnComponent2) / normalizingFactor;

            //Slow Movements - DPAD

            if (slowFront){
                fl = 0.35;
                fr = 0.35;
                bl = 0.35;
                br = 0.35;
            }
            else if (slowBack){
                fl = 0.35 * -1;
                fr = 0.35 * -1;
                bl = 0.35 * -1;
                br = 0.35 * -1;
            }
            else if (slowLeft){
                fl = -1 * 0.5;
                fr = 0.5;
                bl = 0.5;
                br = -1 * 0.5;
            }
            else if (slowRight){
                fl = 0.5;
                fr = -1 * 0.5;
                bl = -1*0.5;
                br = 0.5;
            }

            robot.leftFrontDrive.setPower(fl);
            robot.rightFrontDrive.setPower(fr);
            robot.leftBackDrive.setPower(bl);
            robot.rightBackDrive.setPower(br);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}