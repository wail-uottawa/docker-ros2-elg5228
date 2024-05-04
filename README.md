***Docker image with ROS 2 Humble on Ubuntu (22.04) + Gazebo + HTML5 VNC interface + several robot packages***

**Maintainer:** *Wail Gueaieb*

[![Docker Pulls](https://img.shields.io/docker/pulls/realjsk/docker-ros2-humble-elg5228)](https://hub.docker.com/r/realjsk/docker-ros2-humble-elg5228 )
[![Docker Automated build](https://img.shields.io/docker/automated/realjsk/docker-ros2-humble-elg5228)](https://hub.docker.com/r/realjsk/docker-ros2-humble-elg5228 )
[![GitHub last commit (branch)](https://img.shields.io/github/last-commit/wail-uottawa/docker-ros2-elg5228/main)](https://github.com/wail-uottawa/docker-ros2-elg5228 )
[![GitHub license](https://img.shields.io/github/license/wail-uottawa/docker-ros2-elg5228)](https://github.com/wail-uottawa/docker-ros2-elg5228/blob/main/LICENSE )

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [Overview](#overview)
- [Running the Docker Image](#running-the-docker-image)
    - [Connecting to the Image by Running it on your Local Machine (Host)](#connecting-to-the-image-by-running-it-on-your-local-machine-host)
        - [Connecting Through Web Browser](#connecting-through-web-browser)
        - [Stopping the Image](#stopping-the-image)
    - [Connecting to the Image Through a uOttawa's Virtual Machine ](#connecting-to-the-image-through-a-uottawas-virtual-machine)
        - [Local Acess](#local-acess)
        - [Remote Acess (Through Virtual Desktop Infrastructure (VDI))](#remote-acess-through-virtual-desktop-infrastructure-vdi)
        - [Starting the Virtual Machine](#starting-the-virtual-machine)
        - [Starting/Stopping the Docker Image from Within the Virtual Machine](#startingstopping-the-docker-image-from-within-the-virtual-machine)
    - [Connecting to the Image by Running it on Ontario Research & Education VCL Cloud ](#connecting-to-the-image-by-running-it-on-ontario-research--education-vcl-cloud)
- [Operation Workspace ](#operation-workspace)
- [Installed Robots](#installed-robots)
    - [Wheeled Mobile Robots](#wheeled-mobile-robots)
    - [Fixed Manipulators](#fixed-manipulators)
- [Utilities](#utilities)
    - [Text Editors](#text-editors)
    - [Terminal Emulators](#terminal-emulators)
    - [Web Browsers](#web-browsers)
    - [Document Viewers](#document-viewers)
    - [FTP Clients](#ftp-clients)
- [Getting the Docker Image](#getting-the-docker-image)
    - [Pulling the Docker Image from Docker Hub](#pulling-the-docker-image-from-docker-hub)
    - [Building the Docker Image Locally](#building-the-docker-image-locally)
- [Running the Container](#running-the-container)
    - [Connect & Control](#connect--control)
    - [Environment Settings](#environment-settings)
        - [Overriding VNC and container environment variables](#overriding-vnc-and-container-environment-variables)
- [Acknowledgment](#acknowledgment)
- [Disclaimer](#disclaimer)

<!-- markdown-toc end -->


# Overview

This is a docker image to support teaching ROS-based robotic courses (including "ELG 5228: Mobile Robots" at the University of Ottawa) and to help researchers and hobbyists to experiment with a number of robots. It provides enough power and flexibility to cover various robotic topics (e.g., navigation, control, path planning, fixed manipulators, wheeled mobile robots, etc.) with the ease to add more as needed. It comes with the following main components:
* ROS 2 Humble installed on Ubuntu 22.04
* Gazebo 11
* HTML5 VNC interface to facilitate remote access
* ROS 2 packages of a number of robots, as detailed below in Section [Installed Robots](#installed-robots)


The Dockerfile is inspired by the following works: of tiryoh/ros-desktop-vnc:
* [https://github.com/Tiryoh/docker-ros-desktop-vnc](https://github.com/Tiryoh/docker-ros-desktop-vnc)
* [https://github.com/AtsushiSaito/docker-ubuntu-sweb](https://github.com/AtsushiSaito/docker-ubuntu-sweb)

Most of their documentations is still valid for this image.

# Running the Docker Image

## Connecting to the Image by Running it on your Local Machine (Host)
Probably, the easiest way to run the docker image is to run the provided shell script file `docker-run.sh`, for Linux and Mac users, or `docker-run.bat`, for Windows users (or copy and paste its content), at a command line of your personal computer, while a docker server is running in the background. You may get the files `docker-run.sh` and `docker-run.bat` from the image's Github repository [https://github.com/wail-uottawa/docker-ros2-elg5228](https://github.com/wail-uottawa/docker-ros2-elg5228).

---

<span style="color:red">**WARNING (Use of Volumes):**</span>  
* All changes made to any file/directory within the file system of a docker container are not permanent. They are lost once the container is stopped. 
* To avoid this problem, the `docker-run.sh` (or `docker-run.bat`) script maps a local folder on your computer (i.e., host) onto another in the container file system. For instance, in the case of the provided `docker-run.sh` (or `docker-run.bat`) the folder `~/OneDrive-uOttawa/Docker-ELG5228-ROS2/course_dir` (or `/C/OneDrive-uOttawa/Docker-ELG5228-ROS2/course_dir`) on the host system is mapped onto folder `/home/ubuntu/ros2_ws/src/course_dir` in the docker container. 
* It is highly recommended that you dedicate a local folder on your computer as a ROS working folder (e.g., throughout the course). It can have any name and path (for example: `/C/path/to/course_dir` for Windows hosts or `~/path/to/course_dir` for Linux and Mac hosts). 
* To be even safer, you might want to have this folder as part of a cloud drive that is automatically synchronized on your local machine (such as Google Drive, OneDrive, etc.) as in the above examples, although this is not necessary. 
* Inside the file `docker-run.sh` (or `docker-run.bat`), replace `~/OneDrive-uOttawa/Docker-ELG5228-ROS2` (or `/C/OneDrive-uOttawa/Docker-ELG5228-ROS2`) part with the path to your dedicated local folder. That way, each time you run the docker image through `docker-run.sh` (or `docker-run.bat`) your local dedicated folder is automatically mapped onto `/home/ubuntu/ros2_ws/src/course_dir` in the docker container. As such, whenever you make changes on your local dedicated folder or/and on `/home/ubuntu/ros2_ws/src/course_dir` from within the container, those changes remain permanent on the local drive and are automatically made visible from within the container at `/home/ubuntu/ros2_ws/src/course_dir` every time you run the image. 
* You can learn more about volumes on this designated [docker reference page](https://docs.docker.com/storage/volumes/).

---

<span style="color:red">**WARNING (Windows Users):**</span> If you are running the docker image from a Windows host, please take note of the following remarks:
* You may not be able to run the file `docker-run.sh` as a shell script. Instead, run the file `docker-run.bat` at the DOS prompt. 
* Note that the full path of the local folder, which you would like to map to `/home/ubuntu/ros2_ws/src/course_dir` on the docker image, must be in the format `/C/...`; for example, `/C/Courses/Mobile-robotics/ros2_ws/src/course_dir`. Of course, you can use other drives if your folder isn't on the C drive. 
* When you create a file in a Windows machine (e.g., `program.py`) and then you try to run a ROS command on it from inside the docker container (e.g., `ros2 run`) you may get an error message of the form "`[...]\r`". This is due to the mismatch between the way Windows and Linux systems encode a carriage return (to mark the end of of a line). There are a few ways to go around this problem: 
	* Use any of the commands described in this link [[html](https://www.cyberciti.biz/faq/howto-unix-linux-convert-dos-newlines-cr-lf-unix-text-format/)] to convert the file to a "Linux-compatible" file.
	* Create the file inside the docker container. Then you should be able to edit it from the Windows machine without a problem. 

---

<span style="color:red">**NOTE:**</span> The first time you run the script in `docker-run.sh` (or `docker-run.bat`), it will take a relatively long time to pull the image from the docker hub ([https://hub.docker.com/r/realjsk/docker-ros2-humble-elg5228](https://hub.docker.com/r/realjsk/docker-ros2-humble-elg5228)), due to the image's large size. However, subsequent runs should be much faster since the image will be cached locally by docker.

### Connecting Through Web Browser
Successfully running `docker-run.sh` (or `docker-run.bat`) takes you to a shell command inside the docker container. However, this doesn't allow you to run graphical applications from within the container (yet). To do so, while the image is running, simply point your web browser to: [http://127.0.0.1:6080](http://127.0.0.1:6080) 

### Stopping the Image
After finishing working with the docker container, you can stop it in one of the following methods:
* Graphically through Docker Dashboard/Desktop (if available)
* In the same terminal where you run `docker-run.sh` (or `docker-run.bat`), which is now showing the command line within the container, type Ctrl-c. This should kill the running of the container. 
* From the command line on your host computer by running `docker stop IMAGE_ID:tag`, where `IMAGE_ID` and `tag` are the ID and tag of the image you want to stop(e.g., `docker stop realjsk/docker-ros2-humble-elg5228:20240503`). Another way is to use the command `docker stop $(docker ps -a -q)`, which will stop *all* docker images running on your computer.

## Connecting to the Image Through a uOttawa's Virtual Machine 
<span style="color:red">**NOTE:**</span> This method of connecting to the image is only available to uOttawa affiliates. <br />
Generally, it is prefered to run the image on your local computer. However, if it doesn't have enough processing or memory power, you can run it on a virtual machine that was specially set up by the IT team at the Faculty of Engineering. The virtual machine is running Ubuntu~20.04 and can be accessed locally or remotely, as described below. 

### Local Acess
1. Walk into a computer lab (e.g., STE-0110) or a teaching lab (e.g., STE-0130), but NOT an instrumentation lab. 
2. Logon to any lab computer 

### Remote Acess (Through Virtual Desktop Infrastructure (VDI))
1. Launch **VMware Horizon Client** (which you can access at [`https://uovdi.uottawa.ca`](https://uovdi.uottawa.ca))
2. Logon to the connection server **vdi-genie.uottawa.ca** <br />
    NB: You need to connect through a VPN if you are off-campus. More about VPN can be found at [`https://www.uottawa.ca/about-us/information-technology/services/internet/vpn`](https://www.uottawa.ca/about-us/information-technology/services/internet/vpn). 

### Starting the Virtual Machine
1. If you want to be able to access files on the virtual machine from the local machine (the host), or vice versa, then do the following **before** launching the target virtual machine (**Ubuntu WG**):
	* Right-click on the desired VM and select **Settings**, or Open **Connection -> Settings**, or Hit the **cog wheel** in the upper right corner
	* Select **Drive Sharing** (in the left pane, often near the top) <br />
    NB: on a Mac, **Drive Sharing** is found under **Preferences**
	* Select the drive to be access from the virtual machine 
		* If you you are on a personnal computer, you can
			* enable **Share your home folder** to access your home directory from the viirtual machine 
			* and/or select specific directories by clicking **Add** to browse to them and **Open** them
		* In all cases, you can also enable **Allow access to removable storage** to access data from a USB key or CD
	* Hit **Apply**
	* Hit **OK**
	* The local directories and/or devices will show up in the home folder of the virtual machine (after it is started) under **tsclient**
2. Double click on the target virtual machine (**Ubuntu WG**) to start it 

### Starting/Stopping the Docker Image from Within the Virtual Machine
* Once inside the virtual machine, open a terminal and run the script in file `docker-run-vm.sh`. 
* At this point, you are almost as if you are running the image from your local machine. Follow the rest of the instructions from Section
[Connecting to the Image by Running it on your Local Machine (Host)](#connecting-to-the-image-by-running-it-on-your-local-machine-host)).
* The only exception is that, in this case, drive `/home/ubuntu/ros2_ws/src/course_dir` in the docker container is mapped to drive `course_dir` in the home directory in the virtual machine. You need to manually copy the files/folders you need to work with from your local machine (found under **tsclient** in the virtual machine as described in Section [Starting the Virtual Machine](#starting-the-virtual-machine)) to `~/course_dir` in the virtual machine. Do NOT forget to copy them back to your local machine BEFORE terminating the running of the docker container. Remember that once the container is terminated, all the files under the file system of the container are lost permanently. 

## Connecting to the Image by Running it on Ontario Research & Education VCL Cloud 
<span style="color:red">**(This method is no longer supported by uOttawa)**</span> <br />
<span style="color:red">**NOTE:**</span> This method of connecting to the image is only available to uOttawa affiliates. <br />
Generally, it is prefered to run the image on your local computer. However, if it doesn't have enough processing or memory power, you can run it on a virtual machine on Ontario Research & Education VCL Cloud. Access to the virtual machine is achieved in a few steps, from on-campus or off-campus devices alike. Since this is a remote connection, you may experience some delay depending on the speed of your internet connection and where you are connecting from. 

1. Browse to the VCL portal: [https://orec.rdc.uottawa.ca](https://orec.rdc.uottawa.ca)
2. Login:
	* Select **University of Ottawa (Azure)**
	* Login using your **uoAccess** credentials ([https://it.uottawa.ca/uoaccess](https://it.uottawa.ca/uoaccess))
	* You will need to confirm your credentials using the MFA system 
3. Make a New Reservation, selecting the image: **ELG5228_20240503**
4. Wait for the environment to be initialized
5. Once "Pending" has changed to "Connect"
	* Hit "Connect" to obtain information to connect to your virtual machine 
	* Take note of the **IP**, **UserID** and **Password** shown, as you will need them in the next step to open an SSH session
6. Connect to the remote server using one of the following methods:
	1. If you are connecting from a personnal computer running Windows, it is recommended that you connect to the server using **MobaXTerm** ([https://mobaxterm.mobatek.net](https://mobaxterm.mobatek.net)) by initiating an SSH connection (using the **IP**, **UserID** and **Password** shown earllier).
	2. If you are connecting from a local machine running Linux or Mac OS, then connect to the server as follows:
		* Esure that "X11 Forwarding" is enabled by entering `xhost +` at a terminal 
		* At the same terminal, type `ssh -L 6901:localhost:6901 -Y UserID@IP` while substituting the **IP** and **UserID** shown earlier, as well as the **Password** when asked for it.
	3. If you are connecting from a Windows computer in one of the computer labs in uOttawa's Faculty of Engineering, you may connect to the server via Bitvise/VcXsvr as follows:
		* Start Menu / Utilities / VcXsrv 1.20.1.2 / XLaunch 
			* Uncheck "Native OpenGL" if your ssh window dissapears on launch
		* Start Menu / Bitvise SSH Client 8.34 / Bitvise SSH Client
			* (Tab:Login) Login/Host: **IP**
			* (Tab:Login) Authentication/Username: **UserID**
			* (Tab:Terminal) X11 Forwarding: **Enable**
			* (Tab:C2S)
				* **Enabled**
				* List. Port: **6901**
				* Dest. Port: **6901**
			* Hit "Login" at the bottom, accept the certificate and provide the **Password** when prompted
7. Once you are connected to the server on the cloud, you can launch the docker image by running the command inside the file `docker-run.sh` as a sudo command (`sudo docker run ...`)
8. To be able to run graphical applications, connect your computer to the docker image using one of the methods explained in sections [Connecting Through Web Browser](#connecting-through-web-browser) or [Connecting Through VNC Viewer](#connecting-through-vnc-viewer).
9. Disconnecting from the virtal machine can easily be done through the graphical interface of the VCL portal ([https://orec.rdc.uottawa.ca](https://orec.rdc.uottawa.ca)). Under "Current Reservations", click the "Delete Reservation" icon. This is also done automatically once your session time expires. 

---
<span style="color:red">**WARNING:**</span> Connecting to the image via Ontario Research & Education VCL Cloud does not allow mapping a local folder on your computer onto folder `/home/ros/catkin_ws/src/course_dir` in the docker image as it was explained in section [Running the Image](#running-the-image). The only way to avoid losing your work under `/home/ros/catkin_ws/src/course_dir` is to FTP all your fles/folders under this directory to your local computer BEFORE terminating the connection. This can be done using the FTP client installed on the image (see section [FTP Clients](#ftp-clients)). Forgetting or neglecting to do so will result in the loss of all your data under that folder. This is a major disadvantage of connecting to the image via Ontario Research & Education VCL Cloud, which is why this connection method should be left as a last resort.

---


# Operation Workspace 
* Throughout the course, we will be building ROS 2 packages and experimenting with them. This is all done inside a ROS 2 workspace, which is referred to here as the "operation workspace". It is already built and set up for you in the docker image under path__`/home/ubuntu/ros2_ws`.

* Some ROS 2 packages are installed in their own workspaces, including those of some of the robots listed in section [Installed Robots](#installed-robots).

* To allow for more customization without having to rebuild the docker image, place the file `customizations.bash` in the root of the mapped drive on your host computer. Include any customizations needed in this file. The file is automatically sourced in `.bashrc` in the home folder of the docker file system. The customization inside `customizations.bash` will be in effect for the terminals launched from that moment on. To have them reflected in the already open terminals, run the command `source ~/.bashrc` in each of them. 

# Installed Robots
The image comes loaded with pre-installed ROS 2 packages for a number of robots.

## Wheeled Mobile Robots
* Turtlebot3 [[Official page](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)] 
    * Installed from source under its own workspace `/home/ubuntu/turtlebot3_ws` (sourced automatically in `~/.bashrc`)
    * Details on how to run the robot in simulation can be found in the [simulation page](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation) of its manual
    * The directive `"export TURTLEBOT3_MODEL=burger"` is already included in `~/.bashrc`
    * To change the TurtleBot3 model, modify that directive accordingly and don't forget to run `source ~/.bashrc` 
* Turtlebot4 [[Official page](https://turtlebot.github.io/turtlebot4-user-manual/)] 
    * Installed from source under its own workspace `/home/ubuntu/turtlebot4_ws` (sourced automatically in `~/.bashrc`)
    * Details on how to run the robot in simulation can be found in the [simulation page](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html) of its manual
* Husarion ROSbot XL [[Official page](https://husarion.com) | [Github](https://github.com/husarion)]
    * Installed from source under its own workspace `/home/ubuntu/rosbotxl_ws` (sourced automatically in `~/.bashrc`)
    * Details on how to run the robot in simulation can be found [here](https://github.com/husarion/rosbot_xl_ros)
* Husarion ROSbot 2R and ROSbot 2 PRO [[Official page](https://husarion.com) | [Github](https://github.com/husarion)]
    * Installed from source under its own workspace `/home/ubuntu/rosbotpr2_ws` (sourced automatically in `~/.bashrc`)
    * Details on how to run the robot in simulation can be found [here](https://github.com/husarion/rosbot_ros)

## Fixed Manipulators
* Universal Robots [[Official page](https://www.universal-robots.com) | [Github](https://github.com/ros-industrial/universal_robot)]
    * To simulate UR robots, run the command `ros2 launch ur_description view_ur.launch.py ur_type:=ur5e` or `ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true`, as documented in their [manual](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/usage.html#usage-with-official-ur-simulator).
    * The supported ur_type values are: ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e, ur20, ur30

# Utilities
To facilitate working from within the docker image, it is loaded with a few useful utilities. 

## Text Editors
* Linux' iconic text editors vi, vim, and Emacs, along with VSCodium. 

## Terminal Emulators
The image comes with a few standard Linux terminals, such as `XTerm`, `UXTerm`, and `Xfce Terminal`, which can be accessed through the *System* submenu under the *Applications* menu. However, when dealing with ROS, it might be more convenient to use multi-pane terminals. The following two are provided for this purpose. Although they offer many features, you will probably mostly be interested in adding and removing panes.
* [tmux](https://github.com/tmux/tmux): You can launch it at any terminal by running the command `tmux`. A gentle introduction can be found here [[html](https://www.ocf.berkeley.edu/~ckuehl/tmux)].
* The terminal emulator [Terminator](https://gnome-terminator.org): It allows you to add and close panes either using the mouse right button or through keyboard shortcut keys. You can find a brief tutorial at [[html](https://www.tecmint.com/terminator-a-linux-terminal-emulator-to-manage-multiple-terminal-windows/)].

## Web Browsers
* Firefox

## Document Viewers
* Evince (document viewer; e.g., pdf, ps, djvu, tiff, dvi, and more)
* Viewnior (image viewer)

## FTP Clients
* [FileZilla](https://filezilla-project.org)

<br /><br />

---
<span style="color:red">**NOTE:**</span> The sections above are all what you need to run the docker image. The rest of the information down below provide details which are less likely to be relevant if you are no expert in docker or/and Linux. You are still encouraged to go through them but don't be alarmed if you don't understand them. In that case, the chances that you won't need them are significant. 

---

<br /><br />


# Getting the Docker Image
The docker image can be either pulled directly from Docker Hub, or built locally on your personal computer. The former method may be much more convenient. 

## Pulling the Docker Image from Docker Hub
Currently, the docker image lives in a Docker Hub repository [realjsk/docker-ros2-humble-elg5228](https://hub.docker.com/r/realjsk/docker-ros2-humble-elg5228). It can be pulled using the docker command:

	docker pull realjsk/docker-ros2-humble-elg5228:<tag>
	
where `<tag>` is the tag you prefer to pull. A list of available tags is found at [https://hub.docker.com/r/realjsk/docker-ros2-humble-elg5228/tags](https://hub.docker.com/r/realjsk/docker-ros2-humble-elg5228/tags). For instance, you can replace `<tag>` in the above command by `20240503`.

## Building the Docker Image Locally
The source files to build the docker image on a local computer are stored at the Github repository [https://github.com/wail-uottawa/docker-ros2-elg5228](https://github.com/wail-uottawa/docker-ros2-elg5228).

1. Start by cloning the repository:  
   `git clone https://github.com/wail-uottawa/docker-ros2-elg5228.git`
2. Then, cd to the directory including the file `Dockerfile` and (with the docker server running) build the image:  
   `docker build --squash -t name:<tag>  .` (note the dot at the end)  
   where `name` and `<tag>` are the name and tag you want to give to the built image.  
   This can also be done by editing and running the provided shell script `docker-build.sh` using the command:  
   `sh docker-build.sh`

# Running the Container

## Connect & Control
The default username and password in the container are `ubuntu` and `ubuntu`.

## Environment Settings

### Overriding VNC and container environment variables
The following VNC environment variables can be overwritten within the docker run command to customize the desktop environment inside the container:
* `VNC_COL_DEPTH`, default: `24`
* `VNC_RESOLUTION`, default: `1920x1080`

For example, you can include `-e VNC_RESOLUTION=800x600` in the `docker run` command. 

# Acknowledgment
Credit goes primarily to the maintainers of the following projects:

* [Tiryoh/docker-ros-desktop-vnc](https://github.com/Tiryoh/docker-ros-desktop-vnc) - developed the base Dockerfile used for this image
* [ConSol/docker-headless-vnc-container](https://github.com/ConSol/docker-headless-vnc-container) 

# Disclaimer
The main purpose of this repository and docker image is to facilitate instructors and researchers efforts in experimenting and conducting realistic simulations of various types of robotic systems. However, it comes with no warranty. Please use it at your own discretion. 

I am no docker expert. It is very likely that the generated docker image and the provided `Dockerfile` are by no means optimal. 


