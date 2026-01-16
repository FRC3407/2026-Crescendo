# TODO:
* Test on 2025 robot with RPi and PhotonVision
    * Test with the current Arducam_OV9281 and Raspberry Pi.  Verify that the `VisionSubsystem` returns an appropriate `Pose2d` whenever it sees an AprilTag.  The `Pose2d` should be pushed to the `DriveSubsystem` to update its odometry.   Watch that the `Field2d` widget on the dashboard always displays an appropriate robot location.
    * Retest with another USB camera, such as a [Microsoft LifeCam](https://en.wikipedia.org/wiki/LifeCam).   Determine if cheaper USB cameras are as effective as the Arducams.
    * Retest with a [Raspberry Pi](https://www.raspberrypi.com/documentation/accessories/camera.html) camera.  Determine if this camera is a workable choice.
    * Retest with multiple cameras connected and configured.  It should be able to get `Pose2d` data from multiple cameras.
* VisionSubsystem
    * Write a `DriveToPoseCommand` that will drive robot to a specified pose based on PIDs.
    * Suppose that this year's game involves shooting a game piece at an AprilTag.  Something similar to the [Speaker](https://www.frcmanual.com/2024/arena#_561-speaker) in our 2024 game.  Write a `DriveToTargetCommand` that calculates the pose five feet in front of an AprilTag and pointing directly at the tag, and then drives the robot to that pose.
* Test [Elastic Dashboard](https://frc-elastic.gitbook.io/docs) integration.
    * Test `setTab()`
    * Build larger dashboards.  Test Field widget.  Add camera view widgets.
    * Add notification if RIO can't connect to photonvision.  Or also maybe connection to lights.
* Test [AdvantageScope](https://docs.advantagescope.org/) logging.  Is there anything else we should add for this?
* Test that [PathPlanner](https://pathplanner.dev/) works well with this template.  It should be easy to just add Paths and Autonomous routines and get them to work.  Also, verify that PathPlanner makes good use of the updated odometry from the `VisionSubsystem`
* Support simulation
* Add a checklist in the README for how to apply this template to a new Robot

