## WhyCode / WhyCon

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
| [WhyCon/WhyCode](https://github.com/jiriUlr/whycon-ros) | general | **actively maintained**, ROS | [Jiri Ulrich](https://scholar.google.com/citations?user=vMtZ5FcAAAAJ&hl=cs&oi=ao) |
| [WhyCon-orig](https://github.com/gestom/whycon-orig) | general | 2D, 3D, ROS, lightweight, autocalibration | [Tom Krajnik](http://scholar.google.co.uk/citations?user=Qv3nqgsAAAAJ&hl=en&oi=ao)|
| [WhyCon-ROS](https://github.com/lrse/whycon) | general | 2D, ROS | [Matias Nitsche](https://scholar.google.co.uk/citations?user=Z0hQoRUAAAAJ&hl=en&oi=ao) |
| [SwarmCon](https://github.com/gestom/CosPhi/tree/master/Localization) | μ-swarms | 2D, individual IDs, autocalibration | [Tom Krajnik](http://scholar.google.co.uk/citations?user=Qv3nqgsAAAAJ&hl=en&oi=ao) |
| [Caspa-WhyCon](http://robotics.fel.cvut.cz/faigl/caspa/) | UAVs | embedded, open HW-SW solution | [Jan Faigl](https://scholar.google.co.uk/citations?user=-finD_sAAAAJ&hl=en) |
| [Social-card](https://github.com/strands-project/strands_social/tree/hydro-devel/social_card_reader) | HRI | ROS, allows to command a robot | [Tom Krajnik](http://scholar.google.co.uk/citations?user=Qv3nqgsAAAAJ&hl=en&oi=ao) |

#### Where is it described ?

<i>WhyCon</i> was first presented on International Conference on Advanced Robotics 2013 [[2](#references)], later in the Journal of Intelligent and Robotics Systems [[1](#references)] and finally at the Workshop on Open Source Aerial Robotics during the International Conference on Intelligent Robotic Systems, 2015 [[3](#references)]. Its early version was also presented at the International Conference of Robotics and Automation, 2013 [[4](#references)]. An extension of the system, which used a necklace code to add ID's to the tags, achieved a best paper award at the SAC 2017 conference [[5](#references)].
If you decide to use this software for your research, please cite <i>WhyCon</i> using the one of the references provided below.

-----

#### Generating tags with ID

1. see [whycode-gen](https://github.com/jiriUlr/whycode-gen)
2. Run `./whycon-id-gen` followed by a number of bits and it will create tags in the working directory.
3. Other program parameters are specified in help `./whycon-id-gen -h`
4. Number of ID bits has to be then passed to whycon on start up. The default value can be set in the launch file.
5. Other ID parameters are treated the same way.

#### Some additional remarks

1. At this point, you can start experimenting with the syste by adding whatever features you might think useful.
2. We have tried to comment the code so an experienced programmer should be able to alter the system accordingly. However, if you have any questions regarding the code, feel free to contact [Tom Krajnik](http://scholar.google.co.uk/citations?user=Qv3nqgsAAAAJ&hl=en&oi=ao) or [Jiří Ulrich](https://scholar.google.com/citations?user=vMtZ5FcAAAAJ&hl=cs&oi=ao)
3. If you use this localization system for your research, please don't forget to cite at least one relevant paper below.

#### License
Please, see the <a href="LICENSE">LICENSE</a> file.

#### <a name="dependencies">Dependencies</a>

- OpenCV
- ROS 2 Jazzy basic packages (see <a href="package.xml">package.xml</a>)

#### References
1. J. Ulrich, A. Alsayed et al.: **[Towards fast fiducial marker with full 6 DOF pose estimation](https://dl.acm.org/doi/abs/10.1145/3477314.3507043)**. Symposium on Applied Computing, 2022. [[bibtex](https://gist.github.com/jiriUlr/7d333e90c43e6b41c79e5150c7a59267)].
1. J. Ulrich: **[Fiducial marker-based multiple camera localisation system](https://dspace.cvut.cz/bitstream/handle/10467/101526/F3-DP-2022-Ulrich-Jiri-main.pdf?sequence=-1&isAllowed=y)**. Master's thesis. Czech Technical University in Prague, 2022. [[bibtex](https://gist.github.com/jiriUlr/e8d53c7edd6b14c824e67e60596a489f)].
3. K. Zampachu: **[Visual analysis of beehive queen behaviour](https://dspace.cvut.cz/bitstream/handle/10467/101048/F3-BP-2022-Zampachu-Kristi-main.pdf?sequence=-1&isAllowed=y)**. Bachelor's thesis. Czech Technical University in Prague, 2022. [[bibtex](https://gist.github.com/jiriUlr/eb08ee4b183c615e312ab2db767e9b18)].
1. J. Ulrich: **[Fiducial Marker Detection for Vision-Based Mobile Robot Localisation](https://dspace.cvut.cz/bitstream/handle/10467/89879/F3-BP-2020-Ulrich-Jiri-main.pdf?sequence=-1&isAllowed=y)**. Bachelor's thesis. Czech Technical University in Prague, 2020. [[bibtex](https://gist.github.com/jiriUlr/348d42b7a1cdd08b94953adedc50c5d7)].
1. T. Krajník, M. Nitsche et al.: <b>A Practical Multirobot Localization System</b>. Journal of Intelligent and Robotic Systems (JINT), 2014.
1. P. Lightbody, T. Krajník et al.: <b>An Efficient Visual Fiducial Localisation System</b>. Applied Computing Review, 2017.
1. T. Krajník, M. Nitsche et al.: <b>External localization system for mobile robotics</b>. International Conference on Advanced Robotics (ICAR), 2013.
1. J. Faigl, T. Krajník et al.: <b>Low-cost embedded system for relative localization in robotic swarms</b>. International Conference on Robotics and Automation (ICRA), 2013.
1. M. Nitsche, T. Krajník et al.: <b>WhyCon: An Efficent, Marker-based Localization System</b>. IROS Workshop on Open Source Aerial Robotics, 2015.

#### Acknowledgements

The development of this work is currently supported by the EU FET Open programme under grant agreement No.964492 project _RoboRoyale_.
The development of this work was supported by the Czech Science Foundation project 17-27006Y _STRoLL_.
In the past, the work was supported by EU within its Seventh Framework Programme project ICT-600623 _STRANDS_.
The Czech Republic and Argentina have given support through projects 7AMB12AR022, ARC/11/11 and 13-18316P.
