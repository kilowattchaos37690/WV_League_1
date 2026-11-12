package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Kid-facing Pinpoint pose checker — CONVERTIBLE to Blocks.
 *
 * Push the robot by hand (or drive with sticks) and watch X / Y / heading.
 * Fix encoder directions / pod offsets in RobotPinpointMyBlocks until:
 *   - push forward → X increases ~inches moved
 *   - strafe left → Y increases
 *   - spin CCW → heading increases
 */
@TeleOp(name = "Sample Pinpoint Pose (Kids)", group = "Sample")
public class SamplePinpointPoseKids extends LinearOpMode {

  @Override
  public void runOpMode() {
    RobotMyBlocks.bindFromLinearOpMode(this);
    RobotPinpointMyBlocks.bindFromLinearOpMode(this);

    RobotMyBlocks.initRobot();
    RobotPinpointMyBlocks.initPinpoint();
    RobotPinpointMyBlocks.setPose(0.0, 0.0, 0.0);

    telemetry.addData("Status", "Ready — Sample Pinpoint Pose (Kids)");
    telemetry.addData("Tip", "A=reset pose 0,0,0; prefer HAND push for dir check");
    telemetry.update();
    waitForStart();

    while (opModeIsActive()) {
      if (gamepad1.a) {
        RobotPinpointMyBlocks.setPose(0.0, 0.0, 0.0);
      }

      double forward = -gamepad1.left_stick_y;
      double strafe = -gamepad1.left_stick_x; // stick right → strafe right
      double turn = gamepad1.right_stick_x;
      RobotMyBlocks.driveArcade(forward, strafe, turn);

      RobotPinpointMyBlocks.reportPose();
      telemetry.update();
      sleep(20);
    }

    RobotMyBlocks.stopDrive();
  }
}
