# Muhkeighnzeigh_2.0 [ 2023 Charged Up Revised ]

This is the revision and redemption of [The Metal Jackets](https://www.metaljackets.org/) after our utter failure of a year, 2023.
Worlds #3, 2024

## Docs / Resources

* [FIRST Robotics Competition Docs](https://docs.wpilib.org/en/latest/)
* [SPARK MAX](https://www.revrobotics.com/sparkmax-software/) and [Status Lights/Codes](https://www.revrobotics.com/sparkmax-quickstart/#status-led)
* [PathPlanner](https://github.com/mjansen4857/pathplanner)
* [LimeLight](https://docs.limelightvision.io/en/latest/)

## VSCode and the WPILib Dev Environment

Follow the [WPILib Installation Guide](https://docs.wpilib.org/en/latest/docs/getting-started/getting-started-frc-control-system/wpilib-setup.html) to set up your environment.

### Setting up the JDK

You should've already set the location of your JDK installation in the above tutorial. If you need to set it again, here are the instructions.
1. Navigate to `File -> Preferences -> Settings`
1. Search for "jdk" in the search bar
1. Click `Java Configuration` on the left-hand sidebar. The only setting visible should be `Java: Home`
1. Click on `Edit in settings.json`
1. The right-hand side stores any settings made by the user. Add a line like this at the end of the file: `"java.home": "/Path/To/JDK/Installation"`
    - If you don't know where your JDK installation is, it's probably in `C:\Users\<Your Username>\frc2020\jdk`.
1. You're done! Wait a bit for the Java Language Server to start up and recognize your project (you should see a little spinning icon at the bottom left of your screen), then test it out by clicking on a variable type (like `Module` or `Drive` or `Double`) and pressing <kbd>F12</kbd>. If all goes well, you should be taken to the definition of that class.

### Opening Projects

It's pretty easy. `File -> Open Folder...`, then navigate to the repository you have cloned (The folder named `Muhkeignzeigh_2.0` this year). 

### Want to learn more?

[Code Navigation](https://code.visualstudio.com/docs/editor/editingevolved)

[Basic Editing](https://code.visualstudio.com/docs/editor/codebasics)

## Building and Deploying

- Run these commands from Git Bash (or through the VS Code interface)
- To build, run `./gradlew build`
- To deploy to the robot, run `./gradlew deploy`
    - Remember to **build** before you **deploy**
- To do both at once, run `./gradlew build deploy`


## Contributing

Here's how to get your code into the main robot repository:

### Any time you want to make a change:

We use a feature branch workflow. You can read more about that [here](https://www.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow).

Commands:
- `git clone <insert branch name>` (eg. git clone https://github.com/Team2068/Muhkeighnzeigh_2.0/)
- `git checkout -b <insert branch name>` (to make a new branch)
- `git add .` (to add all your changes)
- `git commit -m "<insert a short summary of what you did> "` (to commit your change and state what you did, you can also add another -m with a longer description of what you did if it is better described in detail than in summary)
- `git push` (to push up-steam/online)
- `git commit --amend` (if you want to adjust what you did in your last commit you can use the `--amend` to make your changes on the commit, but you have your changes staged before they can be added to your ammendment

We have a protected branch, so pull-requests (pr) are need before your changes can be put on the main branch, and for THE LOVE OF GOD REFACTOR YOUR CODE BEFORE CREATING A PR!!! If you don't I will stirke your pr down with nothing but an army of criticisms that will force you to do the refactor you were avoiding. For overall refactors, don't worry, that'll be in its own branch, but for the branch your about to create a pr for, you should clean up everything before I start roasting your code on stream.
