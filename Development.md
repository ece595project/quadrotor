All development should take place using Docker to provide the ROS images.

__NOTE:__ Docker is an application that creates a pseudo-VM that packages an application (in our case, ROS) and runs it on top of your current system. Using Docker ensures that everyone working on the project will have the same environment to avoid conflicts.

1. Install and run Docker by following the instructions on [Docker's website](https://docs.docker.com/engine/installation/). Make sure to install Docker Community Edition (free). There are installers for Mac, Windows, and Linux.

    You can make sure Docker is installed using:

    ```Shell
    $ docker -v
    Docker version 17.03.0-ce, build 60ccb22
    ```

    __NOTE:__ Take a look at Docker-Machine if you want to run Docker as a fully encapsulated VM. That's fine, too. The commands referenced below should be virtually identical if you are using Docker-Machine, but make sure to take a look at the [docs](https://docs.docker.com/machine/overview/) for more information.

2. Clone the Git repository into a directory on your computer.

    ```Shell
    $ mkdir <some_directory> && cd "$_"
    $ git clone https://github.com/ece595project/quadrotor.git
    ```

3. The developers at ROS have put together a Docker image for every ROS version since Indigo. To get started, let's run a Docker image to start `roscore`. This is required for every running ROS system, and it will be a good way to start.

    Open up a terminal and type:

    ```Shell
    $ docker run -dit --rm --name ros ros:kinetic roscore
    ```

    Here, we are telling Docker to start an interactive container in the background named `ros`. We are using the image named `ros:kinetic`, and running the command `roscore`.

    This will start an instance of `roscore` inside Docker. The command may take some time to complete, since Docker will need to download the `ros:kinetic` image from Docker Hub, but once it is downloaded, it won't need to download it again for any future ROS Kinetic containers.

    Make sure the container is running using `docker ps`. You should see something like:

    ```Shell
    CONTAINER ID        IMAGE               COMMAND                  CREATED             STATUS              PORTS               NAMES
    7b0a69281cd8        ros:kinetic         "/ros_entrypoint.s..."   29 minutes ago      Up 5 seconds                            ros
    ```

    If, for some reason, you want to inspect the `roscore` image, you can enter into it to run somewhat normal commands using:

    ```Shell
    docker exec -it ros bash
    ```

    Here, we are telling Docker that we want to execute a command in an interactive 'terminal' from the `ros` container, and that command is: `bash`, a type of command-line.

    __NOTE:__ We could just as easily tell the image to run another command using this format. For example, we could very easily start another background process.

    This will open up a pseudo-terminal for you to interact with the image. The Docker container is not a full VM, so you will not have access to everything that is contained in a normal installation of Ubuntu, but the ROS image we downloaded earlier is based on a stripped-down version of Ubuntu, and commands here will work similar to what you are used to.

    For example, we can see the running processes:

    ```Shell
    $ ps aux
    root         1  0.5  2.6 320112 54120 ?        Ssl+ 04:25   0:00 /usr/bin/python /opt/ros/kinetic/bin/roscore
    root        40  0.6  2.5 603584 52580 ?        Ssl  04:25   0:00 /usr/bin/python /opt/ros/kinetic/bin/rosmaster --core -p 11311 -w 3 __log:=/root/.ros/log/c640a9b8-0f80-11e7-b6a9-0242ac110
    root        53  0.3  0.4 258208  9988 ?        Ssl  04:25   0:00 /opt/ros/kinetic/lib/rosout/rosout __name:=rosout __log:=/root/.ros/log/c640a9b8-0f80-11e7-b6a9-0242ac110002/rosout-1.log
    root        70  0.2  0.1  19912  3788 ?        Ss   04:26   0:00 bash
      root        79  0.0  0.1  36084  3280 ?        R+   04:26   0:00 ps aux
    ```

    Here you can see that `roscore` is running in this image.

    If you enter the command `rostopic list`, you should see something that resembles the following:

    ```Shell
    root@7b0a69281cd8:/# rostopic list
    /rosout
    /rosout_agg
    ```

    Here you can see that `roscore` is showing that two topics are being published. This shows us that `roscore` is running from inside our image.

    Type `exit` to return to the normal command line.

    To take the container down, we can either reference the container by name or by ID. Use `docker ps` to find the ID of the container if you have any unnamed images. Type the following commands to stop and remove our `ros` image:

    ```Shell
    $ docker stop ros
    $ docker rm ros
    ```

    Great! Now we have a way to run ROS on virtually any OS without having to spin up a full VM. Starting an instance of ROS takes seconds, instead of minutes, and we can start up a standardized environment without having to ensure that everyone has the same host, the same software, and the same VM.

4. From here, we can take advantage of a few scripts to get us up and running for development.
