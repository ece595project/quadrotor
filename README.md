# Quadrotor Search and Rescue
This repo contains the design details of the quadrotor build, as well as the main code base for the project.

## Table of Contents
1. [Development Guidelines](#development-guidelines)

## Development Guidelines
- All code must be tested.
- Use the Vagrant box for testing.

## Instructions for Use

All development should take place using Vagrant as the default VM. The repo comes with a supplied Vagrantfile.

__NOTE:__ Vagrant is an application that creates a pseudo-VM that runs in the background for you to use during development. It ensures that everyone working on the project has the same environment to avoid conflicts.

1. Install Vagrant by going to [Vagrant's website](https://www.vagrantup.com). There are installers for Mac, Windows, and Linux, so if you want to run Vagrant inside a VM, that's fine, too.

2. Clone the Git repository into a directory on your computer.

    ```Shell
    $ mkdir <some_directory> && cd "$_"
    $ git clone https://github.com/ece595project/quadrotor.git
    ```

3. Run `vagrant up` in the directory where you have the Git repo stored. This will download the required image, `hashicorp/boot2docker` and install the Docker image with a full install of ROS and start the Gazebo server.

    __NOTE:__ You may need to type in a password as the machines start up. The default password is `tcuser`. This is set by the creators of the boot2docker image, and cannot be changed by the user.

    This creates a virtual environment with ROS running in Docker containers in a VM. If it sounds complex, it is because we are trying to keep everything running in a pre-defined environment that has an easy 'up' process and a consistent environment.

    Here is a breakdown of the pseudostructure:

    - Your Machine
        - The Vagrant VM with boot2docker installed.
            - Docker images for each ROS instance.
            - Docker images for Gazebo.

    Each of these Docker images is directly connected to the folder you are currently working in, so you can create code in the local directory and have it run directly in ROS, without having to go through the process of spinning up a VM, installing ROS for your particular platform/architecture combination, starting 3 different terminal windows, and executing `roscore`, `roslaunch`, and `python code.py`. Instead, the entire process is replaced by a single command: `vagrant up`.

4. You can then ssh into the container using the following command:

    ```Shell
    $ ssh -p 2222 docker@localhost
    ```

5. Now, you can pass commands directly to the ros container using the following syntax:

    ```Shell
    $ vagrant docker-exec -it ros -- <command_to_be_run>
    ```
