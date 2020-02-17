package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo arm", group="Linear Opmode")
@Disabled

public class ServoArm extends LinearOpMode {
    private Servo UpperServo = null;
    private Servo LowerServo = null;
    @Override
    public void runOpMode()
    {

        UpperServo = hardwareMap.get(Servo.class,"Upper");
        LowerServo = hardwareMap.get(Servo.class, "Lower");
        waitForStart();
        //List<Servo> servos = new ArrayList<Servo>();
        while (opModeIsActive())
        {
            telemetry.addData("test","test");
            if(gamepad1.a)
            {
                UpperServo.setPosition(0.5);
                telemetry.addData("Upper val",0.5);
            }
            if(gamepad1.b)
            {
                UpperServo.setPosition(0);
                telemetry.addData("Upper val",0);
            }

            if(gamepad1.x)
            {
                LowerServo.setPosition(0.5);
                telemetry.addData("Lower val",0.5);
            }
            if(gamepad1.y)
            {
                LowerServo.setPosition(0);
                telemetry.addData("Lower val",0);
            }

        }


    }
}
