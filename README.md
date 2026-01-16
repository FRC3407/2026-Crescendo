# 3407 Java Swerve Robot Template

This is a modification of the [RevRobotics MAXSwerve-Java-Template](https://github.com/REVrobotics/MAXSwerve-Java-Template).  It adds some infrastructure functionality that team 3407 typically uses.

## Modifications from the base template:
* Switched gyro to NavX
* Switch the `DriveSubsystem` to use `SwerveDrivePoseEstimator` for odometry collection.
* Log odometry for [AdvantageScope](https://docs.advantagescope.org/) use.
* Display [odometry on dashboard](https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/field2d-widget.html) using `Field2d`.
* Added [PathPlanner](https://pathplanner.dev/) code for autonomous commands.
* Added [Elastic Dashboard](https://docs.wpilib.org/en/stable/docs/software/dashboards/elastic.html) support.
* Added `VisionSubsystem` which uses [PhotonVision](https://photonvision.org/).

## How to use this template:
Make as many robot programs as you like.  Code is as impermanent as scratch paper. Don't be afraid to throw it away and start over.
To create a new robot program:
* Visit our 'Java_Template' repository.  Click the green "Use this template" button in the upper right corner of the page.  Select "Create a new repository".
* Give the project a new repository name in your own account.  Turn off the "Include all branches" toggle.  Create the repository.
* In your new repository page:
    * Click the green "Code" button.
    * Copy the URL.
* Clone the code back to your laptop:
    * Start up [Visual Studio Code](https://code.visualstudio.com/docs/getstarted/getting-started).
    * Hit Control-Shift-P to bring up the command palette.
    * Type "git clone".
    * Paste in the URL.
* Make changes.
* Push your repository back to Github.
    * As you develop your code, commit and publish your work frequently.  Try to always make your commit messages meaningful, so the commit history will show how the project is evolving.
    
