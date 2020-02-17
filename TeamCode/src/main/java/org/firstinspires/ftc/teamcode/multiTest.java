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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="multi-test")
@Disabled
public class multiTest extends LinearOpMode
{


    private class DriveThread extends Thread
        {
            public DriveThread()
            {
                this.setName("DriveThread");
                telemetry.addData("test","test");
                telemetry.update();
            }
            @Override
            public void run()
            {
                DcMotor encoded_motor = null;
                encoded_motor = hardwareMap.dcMotor.get("encoded");
                waitForStart();
                double target = 0;
                telemetry.addData("start","start");
                telemetry.update();
                while (!isInterrupted())
                {
                    if(gamepad1.a)
                    {
                        target = 250;
                    }
                    else if(gamepad1.b)
                    {
                        target = 0;
                    }
                    else if(gamepad1.x)
                    {
                        target =  -250;
                    }
                    double kp = 1.0/200;
                    double position = encoded_motor.getCurrentPosition();
                    double error = target  - position;
                    double power = error *  kp;
/*                    telemetry.addData("error",error);
                    telemetry.addData("kp",kp);
                    telemetry.addData("position",position);
                    telemetry.update();*/
                    double max_power = 0.15;
                    if(power > max_power) power = max_power;
                    if(power < -max_power) power = -max_power;
                    encoded_motor.setPower(-power);
                }
                }


        }

        @Override
        public void runOpMode()
        {
            Thread test = new DriveThread();
            test.start();
            waitForStart();
            DcMotor secondMotor = hardwareMap.dcMotor.get("second");
            double secondTarget = 0;
            while (opModeIsActive())
            {
                if(gamepad1.y)
                {
                    secondTarget = 250;
                }
                else if(gamepad1.right_bumper)
                {
                    secondTarget = 0;
                }
                else if(gamepad1.left_bumper)
                {
                    secondTarget =  -250;
                }
                double kp = 1.0/200;
                double secondPosition = secondMotor.getCurrentPosition();
                double secondError = secondTarget  - secondPosition;
                double secondPower = secondError *  kp;
                telemetry.addData("second error",secondError);
                telemetry.addData("second position",secondPosition);
                telemetry.addData("second power",secondPower);
                telemetry.update();
                double max_power = 0.15;
                if(secondPower > max_power) secondPower = max_power;
                if(secondPower < -max_power) secondPower = -max_power;
                secondMotor.setPower(-secondPower);
            }
        }
}






