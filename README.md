# Visual Perception Robocup

It is about the development of a crowd of tools to integrate perception capability in a turtlebot using an xtion camera and neronal
network [YOLO](https://github.com/leggedrobotics/darknet_ros).

# Usage Steps

* First of all, *darket ros* neuronal network has to be installed. You have to do it following [this steps](https://github.com/fgonzalezr1998/darknet_ros).

* Once *darknet ros* is installed, we have to execute it typing this two commands:

**Launch camera driver:**
```
$ roslaunch openni2_launch openni2.launch
```

**Launch Darknet Ros:**

```
$ roslaunch darknet_ros darknet_ros.launch
```

Now, we should have *darknet ros* executing, so we can launch *follow person*, *object detection* or *number persons recognition*.

## Number Persons Recognition Package:
It publish in a topic named ``/person_count`` the number of persons that camera is watching.

You can launch this package typing:
```
$ roslaunch number_persons_recognition person_recognition.launch
```

## Object Detection:
It publish in a topic named ``/object_detected`` the object of a list of objects specificated in his launch file that camera is watching.

You can launch this package typing:
```
$ roslaunch object_detection object_detector.launch
```
