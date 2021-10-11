# PANTHER: Perception-Aware Trajectory Planner in Dynamic Environments #

[![PANTHER: Perception-Aware Trajectory Planner in Dynamic Environments](./panther/imgs/four_compressed.gif)](https://www.youtube.com/watch?v=jKmyW6v73tY "PANTHER: Perception-Aware Trajectory Planner in Dynamic Environments")      |  [![PANTHER: Perception-Aware Trajectory Planner in Dynamic Environments](./panther/imgs/five_compressed.gif)](https://www.youtube.com/watch?v=jKmyW6v73tY "PANTHER: Perception-Aware Trajectory Planner in Dynamic Environments") |  
:-------------------------:|:-------------------------:|
[![PANTHER: Perception-Aware Trajectory Planner in Dynamic Environments](./panther/imgs/eight_compressed.gif)](https://www.youtube.com/watch?v=jKmyW6v73tY "PANTHER: Perception-Aware Trajectory Planner in Dynamic Environments")       |  [![PANTHER: Perception-Aware Trajectory Planner in Dynamic Environments](./panther/imgs/sim_compressed.gif)](https://www.youtube.com/watch?v=jKmyW6v73tY "PANTHER: Perception-Aware Trajectory Planner in Dynamic Environments")    |  

## Citation

When using PANTHER, please cite [PANTHER: Perception-Aware Trajectory Planner in Dynamic Environments](https://arxiv.org/abs/2103.06372) ([pdf](https://arxiv.org/abs/2103.06372) and [video](https://www.youtube.com/watch?v=jKmyW6v73tY)):

```bibtex
@article{tordesillas2021panther,
  title={{PANTHER}: Perception-Aware Trajectory Planner in Dynamic Environments},
  author={Tordesillas, Jesus and How, Jonathan P},
  journal={arXiv preprint arXiv:2103.06372},
  year={2021}
}
```

---

**If you are looking for a Planner** 

* **In Multi-Agent and Dynamic Environments, you may be interesed also in [MADER](https://github.com/mit-acl/mader)** ([pdf](https://arxiv.org/abs/2010.11061), [video](https://www.youtube.com/watch?v=aoSoiZDfxGE)):
* **In Static Unknown environments, you may be interesed also in [FASTER](https://github.com/mit-acl/faster)** ([pdf](https://arxiv.org/abs/1903.03558), [video](https://www.youtube.com/watch?v=fkkkgomkX10))

## General Setup

PANTHER has been tested with 

* Ubuntu 18.04/ROS Melodic
* Ubuntu 20.04/ROS Noetic

Other Ubuntu/ROS version may need some minor modifications, feel free to [create an issue](https://github.com/mit-acl/panther/issues) if you have any problems.

**You can use PANTHER with only open-source packages**. 
Matlab is only needed if you want to introduce modifications to the optimization problem.

### <ins>Dependencies<ins>


#### CGAL
These commands will install [CGAL v4.12.4](https://www.cgal.org/):
```bash
sudo apt-get install libgmp3-dev libmpfr-dev -y
mkdir -p ~/installations/cgal && cd ~/installations/cgal
wget https://github.com/CGAL/cgal/releases/download/releases%2FCGAL-4.14.2/CGAL-4.14.2.tar.xz
tar -xf CGAL-4.14.2.tar.xz && cd CGAL-4.14.2/ && cmake . -DCMAKE_BUILD_TYPE=Release && sudo make install
```

#### CasADi and IPOPT

Install CasADi from source (see [this](https://github.com/casadi/casadi/wiki/InstallationLinux) for more details) and the solver IPOPT:
```bash
sudo apt-get install gcc g++ gfortran git cmake liblapack-dev pkg-config --install-recommends
sudo apt-get install swig
sudo apt-get install coinor-libipopt-dev
cd ~/installations #Or any other folder of your choice
git clone https://github.com/casadi/casadi.git -b master casadi
cd casadi && mkdir build && cd build
cmake . -DCMAKE_BUILD_TYPE=Release -DWITH_PYTHON=ON -DWITH_IPOPT=ON .. 
sudo make install
``` 

#### Linear Solvers

Go to [http://www.hsl.rl.ac.uk/ipopt/](http://www.hsl.rl.ac.uk/ipopt/), click on `Personal Licence, Source` to install the solver `MA27` (free for everyone), and fill and submit the form. Once you receive the corresponding email, download the compressed file, uncompress it, and place it in the folder `~/installations` (for example). Then execute the following commands:

> Note: the instructions below follow [this](https://github.com/casadi/casadi/wiki/Obtaining-HSL) closely

```bash
cd ~/installations/coinhsl-2015.06.23
wget http://glaros.dtc.umn.edu/gkhome/fetch/sw/metis/OLD/metis-4.0.3.tar.gz #This is the metis version used in the configure file of coinhsl
tar xvzf metis-4.0.3.tar.gz
#sudo make uninstall && sudo make clean #Only needed if you have installed it before
./configure LIBS="-llapack" --with-blas="-L/usr/lib -lblas" CXXFLAGS="-g -O3 -fopenmp" FCFLAGS="-g -O3 -fopenmp" CFLAGS="-g -O3 -fopenmp" #the output should say `checking for metis to compile... yes`
sudo make install #(the files will go to /usr/local/lib)
cd /usr/local/lib
sudo ln -s libcoinhsl.so libhsl.so #(This creates a symbolic link `libhsl.so` pointing to `libcoinhsl.so`). See https://github.com/casadi/casadi/issues/1437
echo "export LD_LIBRARY_PATH='\${LD_LIBRARY_PATH}:/usr/local/lib'" >> ~/.bashrc
```

<details>
  <summary> <b>Note</b></summary>

We recommend to use `MA27`. Alternatively, you can install both `MA27` and `MA57` by clicking on `Coin-HSL Full (Stable) Source` (free for academia) in [http://www.hsl.rl.ac.uk/ipopt/](http://www.hsl.rl.ac.uk/ipopt/) and then following the instructions above. Other alternative is to use the default `mumps` solver (no additional installation required), but its much slower than `MA27` or `MA57`.

</details>


#### Other dependencies
```bash
sudo apt-get install ros-"${ROS_DISTRO}"-rviz-visual-tools  ros-"${ROS_DISTRO}"-tf2-sensor-msgs
sudo apt-get install git-lfs ccache 
```
To be able to use `catkin build`, run:
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
```
And then
* If you have Ubuntu 18.04, run `sudo apt-get install python-catkin-tools -y`
* If you have Ubuntu 20.04 run `sudo apt-get install python3-osrf-pycommon python3-catkin-tools -y`

Additionally, if you have Ubuntu 20.04, you'll need `sudo apt-get install python-is-python3 -y`

<details>
  <summary> <b>Optional (only if you want to modify the optimization problem, or use a different solver than MA27. MATLAB needed)</b></summary>

The easiest way to do this is to install casadi from binaries by simply following these commands:

````bash
cd ~/installations
mkdir casadi
wget https://github.com/casadi/casadi/releases/download/3.5.5/casadi-linux-matlabR2014b-v3.5.5.tar.gz
tar xvzf casadi-linux-matlabR2014b-v3.5.5.tar.gz -C ./casadi
````

Open Matlab, execute the command `edit(fullfile(userpath,'startup.m'))`, and add the line `addpath(genpath('/home/YOUR_USERNAME/installations/casadi'))` in that file (changing `YOUR_USERNAME` with your username). This file `startup.m` is executed every time Matlab starts.

Then you can restart Matlab (or run the file `startup.m`), and make sure this works: 

```bash
import casadi.*
x = MX.sym('x')
disp(jacobian(sin(x),x))

```

Then, to use a specific linear solver, you simply need to change the name of `linear_solver_name` in the file `main.m`. You can also introduce more changes in the optimization problem in that file. After these changes, you need to run `main.m` twice: first with `pos_is_fixed=false` and then with `pos_is_fixed=true`. This will generate all the necessary files in the `panther/matlab/casadi_generated_files` folder. These files will be read by C++.

> Note: When using a linear solver different from `mumps`, you need to start Matlab from the terminal (typing `matlab`).More info [in this issue](https://github.com/casadi/casadi/issues/2032).

> Note: Instead of the binary installation explained in this section, another (but not so straightforward) way would be to use the installation `from source` done above, but it requires some patches to swig, see [this](https://github.com/casadi/casadi/wiki/matlab).
</details>


### <ins>Compilation<ins>
```bash
cd ~/Desktop && mkdir ws && cd ws && mkdir src && cd src
git clone https://github.com/mit-acl/panther.git
cd panther
git lfs install
git submodule init && git submodule update
cd ../../ && catkin build
echo "source ~/Desktop/ws/devel/setup.bash" >> ~/.bashrc 
```

### Running Simulations

Simply execute

```
roslaunch panther simulation.launch quad:=SQ01s
```

Now you can click `Start` on the GUI, and then press `G` (or click the option `2D Nav Goal` on the top bar of RVIZ) and click any goal for the drone. 


You can also change the following arguments when executing `roslaunch`

| Argument      | Description |
| ----------- | ----------- |
| `quad`      | Name of the drone        |
| `perfect_controller`      | If true, the drone will track perfectly the trajectories, and the controller and physics engine of the drone will not be launched. If false, you will need to clone and compile [snap_sim](https://gitlab.com/mit-acl/fsw/snap-stack/snap_sim), [snap](https://gitlab.com/mit-acl/fsw/snap-stack/snap) and [outer_loop](https://gitlab.com/mit-acl/fsw/snap-stack/outer_loop)       |
| `perfect_prediction`      | If true, the drone will have access to the ground truth of the trajectories of the obstacles. If false, the drone will estimate their trajectories (it needs `gazebo=true` in this case).       |
| `gui_mission`      | If true, a gui will be launched to start the experiment       |
| `rviz`      | If true, Rviz will be launched for visualization       |
| `gazebo`      | If true, Gazebo will be launched  |
| `gzclient`      | If true, the gui of Gazebo will be launched. If false, (and if `gazebo=true`) only gzserver will be launched. Note: right now there is some delay in the visualization of the drone the gui of Gazebo. But this doesn't affect the point clouds generated. |

You can see the default values of these arguments in `simulation.launch`.

> **_NOTE:_**  (TODO) Right now the radius of the drone plotted in Gazebo (which comes from the `scale` field of `quadrotor_base_urdf.xacro`) does not correspond with the radius specified in `mader.yaml`. 

> **_NOTE:_**  (TODO)  The case `gazebo=true` has not been fully tested in Ubuntu 20.04.

## Credits:
This package uses some the [hungarian-algorithm-cpp](https://github.com/mcximing/hungarian-algorithm-cpp) and some C++ classes from the [DecompROS](https://github.com/sikang/DecompROS) and  repos (both included in the `thirdparty` folder), so credit to them as well. 

---------

> **Approval for release**: This code was approved for release by The Boeing Company in March 2021. 
