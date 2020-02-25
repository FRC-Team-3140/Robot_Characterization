# Robot_Characterization

This repo contains different robot characterization projects for each subsystem of the robot; 
each project tackles one subsystem and occupies it's own branch.

When you fetch this project add the Robot_Characterization folder to your vscode workspace.
Then whenever you change branches you will have all of the files needed for your project in that folder.
The code that is deployed to the robot will be found inside of the charachterization-project (or similar) folder.

Since the code is in a subfolder we run into an issue where the wpilib commands in vscode for build and deploy break
since gradle can not see the folder with it's code. To fix this right click on either of the custom made batch files and select run 
code to replace the wpilib extension functionality. These files are buildProject.bat and deployProject.bat
