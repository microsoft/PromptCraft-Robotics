# sjtu_drone_description

This package contains the xacro/urdf/sdf model description of the sjtu drone and the corresponding plugin for Gazebo 11 and ROS 2 Humble.


## Structure

* __models__: Gazebo sdf model and model meshes
* __include__: Header files for the PID controller and drone plugin
* __src__: Source code for the drone plugin and PID controller
* __urdf__: Xacro and urdf model description files
* __worlds__: Contains one playground world


## Worlds
To fully load the world you need to donwload the gazebo models first:
```
curl -L https://github.com/osrf/gazebo_models/archive/refs/heads/master.zip -o /tmp/gazebo_models.zip \
    && unzip /tmp/gazebo_models.zip -d /tmp && mkdir -p ~/.gazebo/models/ && mv /tmp/gazebo_models-master/* ~/.gazebo/models/ \
    && rm -r /tmp/gazebo_models.zip
```

## TF Tree

![TF Tree](../imgs/tf_tree.png)