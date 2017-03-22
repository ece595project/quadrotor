# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant.configure("2") do |config|

  # config.vm.boot_timeout = 600
  # config.ssh.insert_key = true
  # config.ssh.private_key_path = "~/.ssh/id_rsa.pub"
  # config.ssh.insert_key = true

  # Set the default VM to use the Ubuntu Xenial box that we specified in
  # `images/hostmachine/Vagrantfile`.

  # config.ssh.private_key_path = "~/.ssh/id_rsa"

  config.vm.provider "docker" do |d|
    d.vagrant_vagrantfile = "./images/hostmachine/Vagrantfile"
  end

  # Start an instance of 'roscore'.
  # config.vm.define "roscore" do |roscore|
  #   roscore.vm.provider "docker" do |d|
  #     d.image = "ros:kinetic"
  #     d.cmd = ["roscore"]
  #     d.name = "roscore"
  #     d.remains_running = true
  #     d.has_ssh = true
  #   end
  #   # roscore.ssh.username = "docker"
  #   # roscore.ssh.password = "tcuser"
  #   # roscore.ssh.insert_key = false
  #   # roscore.ssh.port = 22
  # end

  # Start an instance of gzserver.
  # config.vm.define "gzserver" do |gzserver|
  #   gzserver.vm.provider "docker" do |d|
  #     d.image = "gazebo:gzserver8"
  #     d.cmd = ["gzserver"]
  #     d.name = "gzserver"
  #     d.remains_running = true
  #     d.has_ssh = false
  #   end
  #   # gzserver.ssh.username = "docker"
  #   # gzserver.ssh.password = "tcuser"
  #   # gzserver.ssh.port = 22
  # end

  # Start an instance of 'ros' that we can SSH into.
  config.vm.define "ros" do |ros|
    ros.vm.provider "docker" do |d|
      d.image = "ros:kinetic"
      # d.build_dir = "./images/ROS/"
      # d.cmd = ["roslaunch", "my-ros-app my-ros-app.launch"]
      d.name = "ros"
      d.create_args = ["-dit"]
      d.has_ssh = true

      # d.vagrant_vagrantfile = "./images/hostmachine/Vagrantfile"
    end

    # ros.ssh.username = "ubuntu"
    # ros.ssh.password = ""
    ros.ssh.insert_key = true
    ros.ssh.port = 22
  end


end
