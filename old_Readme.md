


[
You can also use the MKL libraries:
1.- Make sure you have an Intel processor (run `lscpu`), and if so, install the Intel MLK libraries using the script of the section "installing the Intel MKL" of https://csantill.github.io/RPerformanceWBLAS/

2.- adding this to the bashrc
source /opt/intel/parallel_studio_xe_2018.2.046/psxevars.sh intel64

3.- and then, in the command below, use this flag
--with-blas=' -L${MKLROOT}/lib/intel64 -lmkl_intel_ilp64 -lmkl_core -lmkl_intel_thread -lpthread -lm -ldl'
But right now Matlab crashes when doing that
]



Extract it
cd coinhsl-XXXX
Note that the name of a subfolder is called 'metis-XXXX'. 
(la version tambien la puedes ver en el archivo "configure", esta hand-coded all'i)
Download metis-XXXX.tar.gz from http://glaros.dtc.umn.edu/gkhome/metis/metis/download, extract it, and place it inside the folder `coinhsl-XXXX`


--------------NOT NEEDED
Download metis-5.1.0.tar.gz from http://glaros.dtc.umn.edu/gkhome/metis/metis/download
Extract it
[Move it if you want to, for example, the installation folder that I usually use]
cd metis-5.1.0
make config shared=1
sudo make install  //File now availble at /usr/local/lib/libmetis.so
-----------END OF NOT NEEDED


(note that you could also run simply `sudo make` (without install), then create the symbolic link, in the folder that contains the `libhsl.so` and then export LD_LIBRARY_PATH=path_to_that_folder)
Make sure you have BLAS installed: run this command to make sure
	locate libblas.so


	========================
To download the repo, install all the dependencies and compile simply run these commands:

```bash
cd ~/ && mkdir ws && cd ws && mkdir src && cd src
git clone https://github.com/mit-acl/panther.git
cd ..
bash panther/install_and_compile.sh
```

The [bash script](https://github.com/mit-acl/panther/blob/master/install_and_compile.sh) will install [NLopt v2.6.2](https://nlopt.readthedocs.io/en/latest/), [CGAL v4.12.4](https://www.cgal.org/), [GLPK](https://www.gnu.org/software/glpk/) and other ROS packages (check the script for details). This bash script assumes that you already have ROS installed in your machine. 


roslaunch panther single_agent_simulation.launch gazebo:=false perfect_tracker:=false use_gui_mission:=true

----
Sim without gui: `roslaunch panther single_agent_simulation.launch use_gui_mission:=false`
Sim with gui: `roslaunch panther single_agent_simulation.launch use_gui_mission:=true`. You can also the z value of `panther_specific.launch` to set the initial position of the drone
----