# UNSW MTRN2500 2019 T3 Assignment 2

The assignment specification is available in `assignment2_spec.md`


#### Setup instructions:

1. Open a terminal and navigate to the ROS workspace:  
    a. `ctrl + alt + t`
    b. `cd ~/mtrn2500_ws`      
2. Remove the `old-src` directory (folder):  
     `rm -r old-src`  
3. Set up public key with VM and github account:  
    a. https://help.github.com/en/github/authenticating-to-github/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent  
    b. https://help.github.com/en/github/authenticating-to-github/adding-a-new-ssh-key-to-your-github-account  
4. Clone (copy) your assignment repository:  
    `git clone url` where `url` is your assignment git repository.  
    - The repository url should look like git@github.com:UNSW-MTRN2500/assignment2-YOUR_USER_NAME.git and can be found by clicking the "clone or download" button on the repo home page.

#### Launching the visualisation environment rviz2:  
1. Open a new terminal or another terminal tab  
    `ctrl + alt + t` or `ctrl + shift + T`  
2. Complile:  
    `. mtrn2500_make`  
~3. Run the visualisation node~  
~`ros2 launch assignment2 rviz2.py`~ 
 Previous launch script was not working. 
3. Start rviz2:
    `ros2 run rviz2 rviz2`
4. Config rviz2 by clicking on `file` in rviz2 and select open config. There are two new config file in the launch folder.
    * `rviz_task0_config.rviz` set the global frame to `z0000000/local_frame`, this will show a rectangle marker at the origin. This works does not require working transform broadcast.
    * `rviz_task0_config.rviz` set the global frame to `world_frame`, the provided `TranformBroadcaster` provided node will calculate send the required `world_frame` to `z0000000/local_frame` transform based on `z0000000/pose` message. Working task 2 is required for this.
`

#### Then compile and run assignment2:  
4. Start another terminal tab  
    `ctrl + shift + T`  
5. Complile:  
    `. mtrn2500_make`  
6. Run the assignment2 node (will only do something once you've implemented it)    
    `ros2 run assignment2 assignment2`

## FAQ

### Q: Why do I need to create the review branch now?
A: The purpose of the review branch is for you to be able to create a pull request with all the commit you have made. By getting you to make the review branch at the start of the project, all the commit you make to the master branch from now on will show up when you make the pull request. This way the tutor will be able to access all of your work.

### Q: Will my work be assessed on the master or review branch?
A: Your last commit to the master branch before the assignment due date will be counted as your submission. You should not be pulling any code into the review branch. You just need to make a pull request from the master branch to the review branch, it will automatically be kept up to date as you add commit to the master branch.

### Q: Is the following the style guide mandatory?
A: Yes, it is important to keep a code base nicely formatted, this make the code easier to understand for other people that are also working on the same project. For lots of the stylistic choices, there are often no clear best option, the key is to pick one style and follow it consistently. The style guide is the documentation of the choices selected for the project. When you are working with an existing code base, you should follow their existing style.

 ### Q: How can I check if task 0 is working correctly.
A: You can list all the topic that currently exist with the command:

`ros2 topic list`

You can display message sent to a topic with the command:

`ros2 topic echo topic_id`

### Q: Why is my VirtualBox installation on macOS failing?
A: macOS can prevent VirtualBox from installing correctly due to the default setting where only applications from the App Store are permitted to be installed. To fix this:

* Go to System
* Click on Security and Privacy
* Near the bottom of the window, you will find a button that will allow you to override the default behaviour of macOS that prevents you from installing it (as it is not from the App Store).

### Q: I get an error saying "Missing VirtualBox Extension Pack", why?
A: In order for the VM to function correctly, you need to have the VirtualBox Extension Pack installed. Download this from the same Oracle website that you used to install VirtualBox, and double click on the file. This will provoke VirtualBox to install this extension pack. Ensure that it is installed successfully.

 
### Q: I haven't changed any settings on the VM yet it won't start, why?
A: Please ensure that you have installed VirtualBox and it's relevant extension pack successfully. In some cases, VirtualBox can appear to install correctly however it still fails. In that case, re-install VirtualBox and ensure that it informs you that it has installed successfully. If you have already ensured that everything has been installed correctly, you may have invalid VM settings. The default RAM allocation is 8192MB and the default CPU core allocation is 2 CPU cores. On physical machines (such as a laptop or desktop computer), you must first verify how much RAM is installed and how many cores your machine has.

 
You may need to change the VM settings. You can access the settings by ensuring that the correct VM is highlighted, and clicking the settings button at the top of the VirtualBox window. Click on the System tab. If you have less than 16GB of RAM on your machine, ensure that the RAM allocation is half of the RAM available on your machine. If your machine has 8GB of RAM installed, set this value to 4096MB (4GB). With 4GB of installed RAM, set it to 2048MB (2GB) of RAM, and so on. If your machine has 3 or less cores, set the CPU core allocation to 1.

 ### Q: vscode flashes on the VM, and it's getting really annoying...
A: When starting vscode using the code command, instead start it with `code --disable-gpu`. Alternatively, run:

`echo "alias code=\"code --disable-gpu\"" >> ~/.bash_aliases`

then close all terminals, start vscode using the `code` command as normal and the flashing should not be present anymore.
 
### Q: How do I create a shared folder so I can transfer files between my VM and host machine?
A: Create a shared folder (sf) in the VirtualBox (vbox) settings by adding a folder, selecting the folder within the directory you want and selecting auto mount. This will mount the folder in `/media/sf_FOLDER_NAME`. By default the user is root and the group is vboxsf. Students need to add their user to that group. You can add your username by running `sudo adduser $USER vboxsf` in a terminal and rebooting to take effect. They will now be able to open the folder (will show up on the desktop) without entering a password.

Mac (and Linux) users are now finished.

Windows have shared privileges off by default so they will have to open the shared folder in file explorer:
	
* Right click Preferences > Sharing > Advanced settings
* Select share.

You will now be able to edit files in both and once they refresh whatever they have it open in the changes from the other will appear.

 

### Q: I'm having problems saving files I edit when I used the `gedit` text editor, how can I work around this problem?
A: See this link: https://askubuntu.com/questions/537799/save-in-gedit-without-in-virtualbox#538095

 
### Q: Suggested Task 1 Starting point
Study the ros2 tutorial section of assignment 2 spec. Start task 1 by subscribing to `z0000000/joy` topic, how the callback that handles the joystick message work by printing some debug text in the callback function.

### Q: List of recommanded plugin for vscode:

* [C/C++ tools](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools) : Add C++ language support to visual studio code.
* [Cmake tools](https://marketplace.visualstudio.com/items?itemName=vector-of-bool.cmake-tools) : Provides configuration info to power the code completion feature of C/C++ tools.
* [ROS](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros) : Provide some ros specific tools.
* [Gitlens](https://marketplace.visualstudio.com/items?itemName=eamodio.gitlens) : Additional git features to complement features built into vscode.

If you encountered any problems and figured out a fix to it, let us know and we can add it to the FAQ.
