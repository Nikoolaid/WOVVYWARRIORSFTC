package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//gamepad 1 is drive, gamepad 2 is weapons!
//mecanum wheels directions :
/*
F = front B = back
Forward: +L & +R
Backward: -L & -R
Left:  -FL&-BR +BL&+FR
Right: +FL&+BR -BL&-FR
Turn Left : -L & +R
Turn Right : +L & -R
fl O----O fr
   |    |
   |    |
bl O----O br
robot top view ^
*/


@TeleOp (name="TestOhNo", group="Linear Opmode")
public class TestOhNo extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private double fl;
    private double bl;
    private double fr;
    private double br;
    private double leftY;
    private double leftX;
    private double rightY;
    private double rightX;
    private boolean leftStick;
    private boolean rightStick;
    private boolean bothStick;

    public void init() {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
    }
    public void loop() {
        // driving stuff !!!!!!!!!
        rightStick = ((rightX != 0) || (rightY != 0));
        leftStick = ((leftX != 0) || (leftY != 0));
        bothStick = leftStick && rightStick;
        leftY = deadband(gamepad1.left_stick_y , .01);
        leftX = deadband(gamepad1.left_stick_x, .01);
        rightY = deadband(gamepad1.right_stick_y, .01);
        rightX = deadband(gamepad1.right_stick_x, .01);
        if(bothStick) {
            leftY = leftY*.5;
            leftX = leftX*.5;
            rightY = rightY*.5;
            rightX = rightX*.5;
        }
        fl = leftY + leftX + rightY;
        fr = leftY - rightX - leftX;
        bl = leftY + rightX - leftX;
        br = leftY - rightX  + leftX;
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }
    private double deadband(double initialVal, double deadband) {
        if ( (initialVal > deadband) || (initialVal < -deadband) ) {
            return 0;
        } else {
            return initialVal;
        }
    } 
    @Override
    public void runOpMode() {
        waitForStart();
        runtime.reset(); //?

        while(opModeIsActive()) {
                 
        }
    }
}