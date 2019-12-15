package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="auto")

public class Auto extends LinearOpMode
{
    static final double FORWARD_VAL = 0.7;
    static final double STOP_VAL = 0;

    private static final double RIGHT_SERVO_OFFSET = 0.05;
    private static final double RIGHT_SERVO_RANGE = 0.5;

    private static final double LEFT_SERVO_OFFSET = 0.85;
    private static final double LEFT_SERVO_RANGE = -0.5;

    private static final double MID_SERVO_OFFSET = 0;
    private static final double MID_SERVO_RANGE = 0.5;

    static final double DEAD_ZONE = 0.1;

    private static final double DC_90_DEGREE = 280;


    private static final double  DC_SERVO_MAX_POWER  = 0.15;


/*    private class AutoDcServo extends Thread
    {
        String name;
        private DcMotor encoded_motor;
        private double target = 0;
        private double offset;
        boolean flag = true;
        AutoDcServo(String m_name)
        {
            name = m_name;
            encoded_motor = hardwareMap.dcMotor.get(name);
            offset = encoded_motor.getCurrentPosition();
        }

        private void setTarget(double newVal)
        {
            target = newVal;
        }

        void setFlag(boolean newState)
        {
            flag = false;
        }

        @Override
        public void run()
        {
            telemetry.addData("start", "start");
            telemetry.update();
            double kp =  -1.0 / 200;
            while (flag)
            {
                double position = encoded_motor.getCurrentPosition() - offset;
                double error = target - position;
                double power = error * kp;


                if (power > DC_SERVO_MAX_POWER) power = DC_SERVO_MAX_POWER;
                if (power < -DC_SERVO_MAX_POWER) power = -DC_SERVO_MAX_POWER;
                encoded_motor.setPower(-power);
            }


*//*                telemetry.addData("target",target);
                telemetry.addData("position",position);
                telemetry.update();*//*
        }
    }

    class Autoforklift extends Thread
    {
        Servo midCatch = null;
        Servo rightCatch = null;
        Servo leftCatch = null;
        DcMotor heightMotor = null;
        AutoDcServo sideMotor = null;
        String rightCatchName;
        String leftCatchName;
        String midCatchName;
        String heightMotorName;
        String sideMotorName;
        boolean flag = true;
        Autoforklift(String m_rightCatchName, String m_leftCatchName , String m_midCatchName, String m_heightMotorName , String m_sideMotorName){
            rightCatchName = m_rightCatchName;
            leftCatchName = m_leftCatchName;
            midCatchName = m_midCatchName;
            heightMotorName = m_heightMotorName;
            sideMotorName = m_sideMotorName;
            midCatch = hardwareMap.get(Servo.class,midCatchName);
            leftCatch = hardwareMap.get(Servo.class,leftCatchName);
            rightCatch = hardwareMap.get(Servo.class,rightCatchName);
            heightMotor = hardwareMap.dcMotor.get(heightMotorName);
            sideMotor = new AutoDcServo(sideMotorName);
        }

        void setFlag(boolean state)
        {
            flag = state;
        }

        @Override
        public void run()
        {
*//*          midCatch.setPosition(0.5);
            leftCatch.setPosition(1);
            rightCatch.setPosition(0);*//*
            sideMotor.start();
            while (flag)
            {
                if (gamepad2.dpad_left) //close arm
                {
                    sideMotor.setTarget(0);
                }
                if (gamepad2.dpad_up) // open arm to the middle
                {
                    sideMotor.setTarget(DC_90_DEGREE);
                }
                if(gamepad2.dpad_right) //open arm to max position
                {
                    sideMotor.setTarget(DC_90_DEGREE * 2);
                }
                double power = gamepad2.left_stick_y;
                if(Math.abs(power) < 0.1 ) //deadzone function for height motor
                {
                    power = 0;
                }
                if(power > 0)
                {
                    power /= 4;
                }



                heightMotor.setPower((-power)*0.75);

                if (gamepad2.right_bumper) // open catchers
                {
                    leftCatch.setPosition(LEFT_SERVO_OFFSET);
                    rightCatch.setPosition(RIGHT_SERVO_OFFSET);
                }

                if(gamepad2.left_bumper) // close catchers
                {
                    leftCatch.setPosition(LEFT_SERVO_OFFSET + LEFT_SERVO_RANGE);
                    rightCatch.setPosition(RIGHT_SERVO_OFFSET + RIGHT_SERVO_RANGE);
                }

                if(gamepad2.y) //move catcher arm back to initial position
                {
                    midCatch.setPosition(MID_SERVO_OFFSET);
                }

                if(gamepad2.a) //move the catcher arm sideways
                {
                    midCatch.setPosition(MID_SERVO_RANGE + MID_SERVO_OFFSET);
                }
                telemetry.addData("mid",midCatch.getPosition());
                telemetry.addData("right",rightCatch.getPosition());
                telemetry.addData("left",leftCatch.getPosition());
                telemetry.update();
            }
            sideMotor.setFlag(false);
        }
    }*/





    class AutodriveThread extends Thread
    {
        private String motor1Name;
        private String motor2Name;
        private String motor3Name;
        private String motor4Name;
         double motor1Val;
         double motor2Val;
         double motor3Val;
         double motor4Val;
        DcMotor motor1;
        DcMotor motor2;
        DcMotor motor3;
        DcMotor motor4;
        boolean flag = true;
        AutodriveThread(String m_motor1,String m_motor2,String m_motor3,String m_motor4)
        {
             motor1Name = m_motor1;
             motor2Name = m_motor2;
             motor3Name = m_motor3;
             motor4Name = m_motor4;

             motor1 = hardwareMap.dcMotor.get(motor1Name);
             motor2 = hardwareMap.dcMotor.get(motor2Name);
             motor3 = hardwareMap.dcMotor.get(motor3Name);
             motor4 = hardwareMap.dcMotor.get(motor4Name);
        }

        void setFlag(boolean newState)
        {
            flag = newState;
        }

        Double normelize(Double value)
        {
            double maxSpeed = 1;
            if(value > maxSpeed) value = maxSpeed;
            else if (value < -maxSpeed) value = -maxSpeed;
            return value;
        }

        Double sumValues(double val1, double val2 , double val3)
        {
            return val1+val2+val3;
        }


        public void run()
        {
            /*
            int numMotors = 4;
            List<Double> movmentValues = new ArrayList<Double>();
            List<Double> normelizedVals = new ArrayList<Double>();*/

/*                int r = 0;
                if(gamepad1.left_bumper)
                {
                    r = 1;
                }
                if (gamepad1.right_bumper)
                {
                    r = -1;
                }*/
/*                motor1Val = normelize(sumValues(gamepad1.left_stick_x , gamepad1.left_stick_y , r));
                motor2Val = normelize(sumValues(gamepad1.left_stick_x ,- gamepad1.left_stick_y ,- r));
                motor3Val = normelize(sumValues(gamepad1.left_stick_x, - gamepad1.left_stick_y ,+ r));
                motor4Val = normelize(sumValues(gamepad1.left_stick_x , gamepad1.left_stick_y ,- r));*/
                motor1.setPower(FORWARD_VAL);
                motor2.setPower(FORWARD_VAL);
                motor3.setPower(FORWARD_VAL);
                motor4.setPower(FORWARD_VAL);
                try
                {
                    sleep(1000);
                } catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
                motor1.setPower(STOP_VAL);
                motor2.setPower(STOP_VAL);
                motor3.setPower(STOP_VAL);
                motor4.setPower(STOP_VAL);
                setFlag(false);


        }
    }





    @Override
    public void runOpMode()
    {
        //creating drive

        AutodriveThread drive = new AutodriveThread("FL","FR","BL","BR");
        drive.start();


        //creating catcher arm
/*        Thread catchers = new c_Catchers("leftArm","rightArm");
        catchers.start()*/;

        //creating lift arm
        waitForStart();
        //noinspection StatementWithEmptyBody
        while (opModeIsActive());
        drive.setFlag(false);
        //catchers.interrupt();
    }
}

