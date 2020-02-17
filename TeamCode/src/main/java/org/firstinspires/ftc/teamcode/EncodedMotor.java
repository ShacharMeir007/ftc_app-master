package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name="encoded motor")
@Disabled
public class EncodedMotor extends LinearOpMode
{
    private DcMotor m = null;
    private DcMotor l = null;
    @Override
    public void runOpMode()
    {
        m = hardwareMap.dcMotor.get("m");
        l = hardwareMap.dcMotor.get("l");
        waitForStart();
        while (opModeIsActive())
        {
           m.setPower(gamepad1.left_stick_y / 3);
           l.setPower(-gamepad1.left_stick_y / 3);
        }
    }
}

