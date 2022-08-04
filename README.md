## WhyCon

### A precise, efficient and low-cost localization system 

_WhyCon_ is a version of a vision-based localization system that can be used with low-cost web cameras, and achieves millimiter precision with very high performance.
The system is capable of efficient real-time detection and precise position estimation of several circular markers in a video stream. 
It can be used both off-line, as a source of ground-truth for robotics experiments, or on-line as a component of robotic systems that require real-time, precise position estimation.
_WhyCon_ is meant as an alternative to widely used and expensive localization systems. It is fully open-source.
_WhyCon-orig_ is WhyCon's original, minimalistic version that was supposed to be ROS and openCV independent.


| WhyCon example application (video)  | Scenario description |
| ------ | ----------- |
|[![WhyCon applications](https://raw.githubusercontent.com/wiki/gestom/WhyCon/pics/whycon.png)](https://www.youtube.com/watch?v=KgKrN8_EmUA"AAAA")|-precise docking to a charging station (EU project STRANDS),<br/> -fitness evaluation for self-evolving robots (EU proj. SYMBRION),<br/>-relative localization of UAV-UGV formations (CZ-USA project COLOS),<br/>-energy source localization in (EU proj REPLICATOR),<br/>-robotic swarm localization (EU proj HAZCEPT).|

The _WhyCon_ system was developed as a joint project between the University of Buenos Aires, Czech Technical University and University of Lincoln, UK.
The main contributors were [Matias Nitsche](https://scholar.google.co.uk/citations?user=Z0hQoRUAAAAJ&hl=en&oi=ao), [Tom Krajnik](http://scholar.google.co.uk/citations?user=Qv3nqgsAAAAJ&hl=en&oi=ao) and [Jan Faigl](https://scholar.google.co.uk/citations?user=-finD_sAAAAJ&hl=en). Further contributors are [Peter Lightbody](https://scholar.google.com/citations?user=tBUM-8oAAAAJ&hl=cs&oi=ao) and [Jiří Ulrich](https://scholar.google.com/citations?hl=cs&user=vMtZ5FcAAAAJ). Each of these contributors maintains a slightly different version of WhyCon.

| WhyCon version  | Application | Main features | Maintainer|
| --------------- | ----------- | ------ | ----- |
| [WhyCon-orig](../../) | general | 2D, 3D, ROS, lightweight, autocalibration | [Tom Krajnik](http://scholar.google.co.uk/citations?user=Qv3nqgsAAAAJ&hl=en&oi=ao)|
| [WhyCon-ROS](https://github.com/lrse/whycon) | general | 2D, ROS | [Matias Nitsche](https://scholar.google.co.uk/citations?user=Z0hQoRUAAAAJ&hl=en&oi=ao) |
| [SwarmCon](https://github.com/gestom/CosPhi/tree/master/Localization) | μ-swarms | 2D, individual IDs, autocalibration | [Tom Krajnik](http://scholar.google.co.uk/citations?user=Qv3nqgsAAAAJ&hl=en&oi=ao) |
| [Caspa-WhyCon](http://robotics.fel.cvut.cz/faigl/caspa/) | UAVs | embedded, open HW-SW solution | [Jan Faigl](https://scholar.google.co.uk/citations?user=-finD_sAAAAJ&hl=en) |
| [Social-card](https://github.com/strands-project/strands_social/tree/hydro-devel/social_card_reader) | HRI | ROS, allows to command a robot | [Tom Krajnik](http://scholar.google.co.uk/citations?user=Qv3nqgsAAAAJ&hl=en&oi=ao) |

#### Where is it described ?

<i>WhyCon</i> was first presented on International Conference on Advanced Robotics 2013 [[2](#references)], later in the Journal of Intelligent and Robotics Systems [[1](#references)] and finally at the Workshop on Open Source Aerial Robotics during the International Conference on Intelligent Robotic Systems, 2015 [[3](#references)]. Its early version was also presented at the International Conference of Robotics and Automation, 2013 [[4](#references)]. An extension of the system, which used a necklace code to add ID's to the tags, achieved a best paper award at the SAC 2017 conference [[5](#references)].
If you decide to use this software for your research, please cite <i>WhyCon</i> using the one of the references provided in this [bibtex](http://raw.githubusercontent.com/wiki/gestom/CosPhi/papers/WhyCon.bib) file.

-----

### Setting up WhyCon in ROS

#### Quick setup for initial testing

0. Have ROS Noetic and appropriate camera driver installed. Also have a <a href="http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration">calibrated camera</a> with distortion model "plumb bob".
0. Make sure your camera is running, e.g., run <i>roslaunch</i> usb_cam_test.launch of the usb_cam package:
```
roslaunch usb_cam usb_cam-test.launch
``` 
0. Check the required <a href="#dependencies">libraries</a>
0. Download the code from GitHub into a catkin workspace.
0. Compile the code - just type `catkin_make` in workspace directory.
0. Source setup script in package directory into shell enviroment e.g. `source devel/setup.bash`
0. Download, resize and print one circular <a href="id/test.pdf">pattern</a> - you have the pattern also in the <i>id/test.pdf</i> file.
0. Run code by <i>roslaunch</i> and remap subsribed camera topics either on start up through arguments, e.g.,
```
roslaunch whycon whycon-test.launch cam_info:=/usb_cam/camera_info cam_raw:=/usb_cam/image_raw
```
or rewrite file <a href="launch/whycon.launch">whycon.launch</a> so default values of tags <i>arg</i> called <i>cam_info</i> and <i>cam_raw</i> will match topics <i>camera_info</i> and <i>image_raw</i>. Then it's just
```
roslaunch whycon whycon-test.launch
```
0. If using patterns with encoded ID keep the option <i>identify</i> turned on in <i>rqt_reconfigure</i> and if without ID, then turn it off!!!
0. You can dynamically change the parameters in <i>rqt_reconfigure</i>.

#### Generating tags with ID

1. see [whycode-gen](https://github.com/jiriUlr/whycode-gen)
2. Run `./whycon-id-gen` followed by a number of bits and it will create tags in the working directory.
3. Other program parameters are specified in help `./whycon-id-gen -h`
4. Number of ID bits has to be then passed to whycon on start up. The default value can be set in the launch file.
```
roslaunch whycon whycon.launch [...] id_bits:=...
```
5. Other ID parameters are treated the same way. ID samples as `id_samples:=...` and Hamming distance as `hamming_dist:=...`

#### Setting up the coordinate system

1. If you have resized the markers (their default size is 122mm), then adjust their diameter in the <i>rqt_reconfigure</i>.
2. Print additional four circular <a href="id/test.pdf">markers</a> and place to the corners of your (reclangular) operational space.
3. Position and fixate your camera so that it has all four circles in it's field of view.
4. Run whycon and modify the dimensions of the operation space in the <i>rqt_reconfigure</i> - the system will now assume that the four markers are at positions [0,0],[fieldLength,0], [0,fieldWidth],[fieldLength,fieldWidth].
5. Start autocalibration or manual calibration through GUI or through rosservice tool.

#### GUI key binding

- see [rqt-whycon-ros](https://github.com/jiriUlr/rqt-whycon-ros)

#### Topics
##### Published
1. /whycon_ros/markers - you can find its header files in <a href="msg/">msg</a> folder
   - Header header
   - whycon_ros/Marker[] markers
     - float32 u                        # camera coordinate
     - float32 v                        # camera coordinate
     - int32 size                       # size of the segment in pixels
     - int8 id                          # ID of pattern
     - geometry_msgs/Pose position      # position with quaternion as orientation
     - geometry_msgs/Vector3 rotation   # vector of Euler angles - pitch, roll, yaw
##### Subscribed
1. /&lt;camera&gt;/camera_info - camera matrix and distortion coeffs
2. /&lt;camera&gt;/image_raw - raw image data without correction

#### Some additional remarks

1. At this point, you can start experimenting with the syste by adding whatever features you might think useful.
2. The code is avaiable only for academic, educational, research, and non-profit useage. In any other case or when You are not sure, please, contact us!
2. We have tried to comment the code so an experienced programmer should be able to alter the system accordingly. However, if you have any questions regarding the code, feel free to contact [Tom Krajnik](http://scholar.google.co.uk/citations?user=Qv3nqgsAAAAJ&hl=en&oi=ao) or [Jiří Ulrich](https://scholar.google.com/citations?user=vMtZ5FcAAAAJ&hl=cs&oi=ao)
3. If you use this localization system for your research, please don't forget to cite at least one relevant paper from these [bibtex](http://raw.githubusercontent.com/wiki/gestom/CosPhi/papers/WhyCon.bib) records.

<hr>

### <a name="dependencies">Dependencies</a>

- OpenCV
- ROS basic packages (see <a href="package.xml">package.xml</a>)

### References

1. T. Krajník, M. Nitsche et al.: <b>[A Practical Multirobot Localization System.](http://raw.githubusercontent.com/wiki/gestom/CosPhi/papers/2015_JINT_whycon.pdf)</b> Journal of Intelligent and Robotic Systems (JINT), 2014. [[bibtex](http://raw.githubusercontent.com/wiki/gestom/CosPhi/papers/2015_JINT_whycon.bib)].
2. T. Krajník, M. Nitsche et al.: <b>[External localization system for mobile robotics.](http://raw.githubusercontent.com/wiki/gestom/CosPhi/papers/2013_icar_whycon.pdf)</b> International Conference on Advanced Robotics (ICAR), 2013. [[bibtex](http://raw.githubusercontent.com/wiki/gestom/CosPhi/papers/2013_icar_whycon.bib)].
3. M. Nitsche, T. Krajník et al.: <b>[WhyCon: An Efficent, Marker-based Localization System.](http://raw.githubusercontent.com/wiki/gestom/CosPhi/papers/2015_irososar_whycon.pdf)</b> IROS Workshop on Open Source Aerial Robotics, 2015. [[bibtex](http://raw.githubusercontent.com/wiki/gestom/CosPhi/papers/2015_irososar_whycon.bib)].
4. J. Faigl, T. Krajník et al.: <b>[Low-cost embedded system for relative localization in robotic swarms.](http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=6630694)</b> International Conference on Robotics and Automation (ICRA), 2013. [[bibtex](http://raw.githubusercontent.com/wiki/gestom/CosPhi/papers/2013_icra_whycon.bib)].
5. P. Lightbody, T. Krajník et al.: <b>[A versatile high-performance visual fiducial marker detection system with scalable identity encoding.](http://eprints.lincoln.ac.uk/25828/1/4d0bd9e8a3b3b5ad6ca2d56c1438fbbc.pdf)</b>Symposium on Applied Computing, 2017.[[bibtex](http://raw.githubusercontent.com/wiki/gestom/CosPhi/papers/2017_sac_whycon.bib)].

### Acknowledgements

The development of this work is currently supported by the Czech Science Foundation project 17-27006Y _STRoLL_.
In the past, the work was supported by EU within its Seventh Framework Programme project ICT-600623 _STRANDS_.
The Czech Republic and Argentina have given support through projects 7AMB12AR022, ARC/11/11 and 13-18316P.
