package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Test", group="Linear Opmode")
@Disabled
public class Servotest extends LinearOpMode {
    private Servo leftDrive = null;
    @Override
    public void runOpMode()
    {
        double leftPower;
        hardwareMap.get(Servo.class, "servoMotor");
        leftDrive  = hardwareMap.get(Servo.class, "servoMotor");
        waitForStart();
        while (opModeIsActive())
        {
            if(gamepad1.a)
            {
                telemetry.addData("val",180);
                leftDrive.setPosition((180));
            }
            else  if (gamepad2.b)
            {
                leftDrive.setPosition((90));
                telemetry.addData("val",90);
            }
            else
            {
                leftDrive.setPosition(0);
                telemetry.addData("val",0);
            }

            telemetry.update();
        }
    }
}

