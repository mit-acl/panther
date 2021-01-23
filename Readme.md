roslaunch mader single_agent_simulation.launch gazebo:=false perfect_tracker:=false use_gui_mission:=true

----
Sim without gui: `roslaunch mader single_agent_simulation.launch use_gui_mission:=false`
Sim with gui: `roslaunch mader single_agent_simulation.launch use_gui_mission:=true`. You can also the z value of `mader_specific.launch` to set the initial position of the drone
----

Make sure you have BLAS installed: run this command to make sure
	locate libblas.so


Note: The part below follows essentially https://github.com/casadi/casadi/wiki/Obtaining-HSL
(but I put -O3 instead)

Go to http://www.hsl.rl.ac.uk/ipopt/ and click on "Coin-HSL Full(Stable)". Note: I think I could also select RC
Fill the form, and wait one day, and download the link in the email you receive
Extract it
cd coinhsl-XXXX
Note the name of a subfolder called 'metis-XXXX'. 
Download metis-XXXX.tar.gz from http://glaros.dtc.umn.edu/gkhome/metis/metis/download, extract it, and place it inside the folder `coinhsl-XXXX`

[
You can also use the MKL libraries:
1.- Make sure you have an Intel processor (run `lscpu`), and if so, install the Intel MLK libraries using the script of the section "installing the Intel MKL" of https://csantill.github.io/RPerformanceWBLAS/

2.- adding this to the bashrc
source /opt/intel/parallel_studio_xe_2018.2.046/psxevars.sh intel64

3.- and then, in the command below, use this flag
--with-blas=' -L${MKLROOT}/lib/intel64 -lmkl_intel_ilp64 -lmkl_core -lmkl_intel_thread -lpthread -lm -ldl'
But right now Matlab crashes when doing that
]

cd coinhsl-XXXX
`sudo make uninstall`
`sudo make clean`
`./configure LIBS="-llapack" --with-blas="-L/usr/lib -lblas" CXXFLAGS="-g -O3 -fopenmp" FCFLAGS="-g -O3 -fopenmp" CFLAGS="-g -O3 -fopenmp"` (make sure the output says `checking for metis to compile... yes`)
`sudo make install` (the files will go to /usr/local/lib)
cd /usr/local/lib
`sudo ln -s libcoinhsl.so libhsl.so` (This creates a symbolic link `libhsl.so` pointing to `libcoinhsl.so`). It's needed because Casadi expects to find `libhsl.so`, see https://github.com/casadi/casadi/issues/1437

And at the end of your `~/.bashrc`, add `export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/usr/local/lib"`

(note that you could also run simply `sudo make` (without install), then create the symbolic link, in the folder that contains the `libhsl.so` and then export LD_LIBRARY_PATH=path_to_that_folder)


--------------NOT NEEDED
Download metis-5.1.0.tar.gz from http://glaros.dtc.umn.edu/gkhome/metis/metis/download
Extract it
[Move it if you want to, for example, the installation folder that I usually use]
cd metis-5.1.0
make config shared=1
sudo make install  //File now availble at /usr/local/lib/libmetis.so
-----------END OF NOT NEEDED



----

# MADER: Trajectory Planner in Multi-Agent and Dynamic Environments #

Single-Agent               |  Multi-Agent           | 
:-------------------------:|:-------------------------:|
[![MADER: Trajectory Planner in Multi-Agent and Dynamic Environments](./mader/imgs/single_agent1.gif)](https://www.youtube.com/user/AerospaceControlsLab "MADER: Trajectory Planner in Multi-Agent and Dynamic Environments")      |  [![MADER: Trajectory Planner in Multi-Agent and Dynamic Environments](./mader/imgs/circle.gif)](https://www.youtube.com/user/AerospaceControlsLab "MADER: Trajectory Planner in Multi-Agent and Dynamic Environments") |  
[![MADER: Trajectory Planner in Multi-Agent and Dynamic Environments](./mader/imgs/single_agent2.gif)](https://www.youtube.com/user/AerospaceControlsLab "MADER: Trajectory Planner in Multi-Agent and Dynamic Environments")       |  [![MADER: Trajectory Planner in Multi-Agent and Dynamic Environments](./mader/imgs/sphere.gif)](https://www.youtube.com/user/AerospaceControlsLab "MADER: Trajectory Planner in Multi-Agent and Dynamic Environments")    |  

## Citation

When using MADER, please cite [this paper](https://www.google.com/):

```bibtex
@article{tordesillas2020mader,
  title={{MADER}: Trajectory Planner in Multi-Agent and Dynamic Environments},
  author={Tordesillas, Jesus and How, Jonathan P},
  journal={arXiv preprint},
  year={2020}
}
```

## General Setup

MADER has been tested with 
* Ubuntu 16.04/ROS Kinetic
* Ubuntu 18.04/ROS Melodic 

To download the repo, install all the dependencies and compile simply run these commands:

```bash
cd ~/ && mkdir ws && cd ws && mkdir src && cd src
git clone https://github.com/mit-acl/mader.git
cd ..
bash mader/install_and_compile.sh
```

The [bash script](https://github.com/mit-acl/mader/blob/master/install_and_compile.sh) will install [NLopt v2.6.2](https://nlopt.readthedocs.io/en/latest/), [CGAL v4.12.4](https://www.cgal.org/), [GLPK](https://www.gnu.org/software/glpk/) and other ROS packages (check the script for details). This bash script assumes that you already have ROS installed in your machine. 

### Running Simulations

#### Single-agent
```
roslaunch mader single_agent_simulation.launch
```
Now you can press `G` (or click the option `2D Nav Goal` on the top bar of RVIZ) and click any goal for the drone. 

To run many single-agent simulations in different random environments, you can go to the `scripts` folder and execute `python run_many_sims_single_agent.py`.

#### Multi-agent

Change these parameters in `mader.yaml`:

```yaml
drone_radius: 0.05
v_max: [2.5, 2.5, 2.5]     
a_max: [30.0, 30.0, 9.6]  
num_pol: 3
a_star_fraction_voxel_size: 0.0
a_star_bias: 7.0
```

and then open four terminals and run these commands:

```
roslaunch mader mader_general.launch type_of_environment:="dynamic_forest"
roslaunch mader many_drones.launch action:=start
roslaunch mader many_drones.launch action:=mader
roslaunch mader many_drones.launch action:=send_goal
```

#### Octopus Search
You can run the octopus search with a dynamic obstacle by simply running

```
roslaunch mader octopus_search.launch
```
And you should obtain this:

![](./mader/imgs/octopus_search.png) 

(note that the octopus search has some randomness in it, so you may obtain a different result each time you run it).

## Credits:
This package uses some C++ classes from the [DecompROS](https://github.com/sikang/DecompROS) repo (included in the `thirdparty` folder), so credit to it as well. 

