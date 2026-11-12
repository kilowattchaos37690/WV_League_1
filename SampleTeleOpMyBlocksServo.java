import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo; // Continuous Rotation Servo Object

@TeleOp(name="WindmillControl", group="Test")
public class WindmillServoTest extends LinearOpMode {

    // Declare your continuous rotation servo
    private CRServo windmillServo;

    @Override
    public void runOpMode() {
        // Initialize the servo from your physical hardware mapping
        windmillServo = hardwareMap.get(CRServo.class, "windmill");

        // Ensure the windmill starts completely stationary
        windmillServo.setPower(0.0);

        telemetry.addData("Status", "Initialized. Press Start.");
        telemetry.update();

        waitForStart();

        // Runtime loop
        while (opModeIsActive()) {
            
            // BUTTON A (Start Moving)
            if (gamepad1.a) {
                // For a CR Servo, 1.0 is full speed forward. Adjust between 0.1 and 1.0 to change speed.
                windmillServo.setPower(1.0); 
                telemetry.addData("Windmill State", "Spinning");
            } 
            
            // BUTTON B (Stop Moving)
            else if (gamepad1.b) {
                // 0.0 cuts off the rotation signal entirely, making it stop
                windmillServo.setPower(0.0); 
                telemetry.addData("Windmill State", "Stopped");
            }

            telemetry.update();
        }
    }
}