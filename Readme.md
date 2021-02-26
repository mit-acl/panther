# PANTHER: Perception-Aware Trajectory Planner in Dynamic Environments #

## Citation

When using PANTHER, please cite PANTHER:

```bibtex
@article{tordesillas2020panther,
  title={{PANTHER}: Perception-Aware Trajectory Planner in Dynamic Environments},
  author={Tordesillas, Jesus and How, Jonathan P},
  journal={arXiv preprint},
  year={2021}
}
```

## General Setup

PANTHER has been tested with Ubuntu 18.04/ROS Melodic 

### <ins>Dependencies<ins>

#### CGAL
These commands will install [CGAL v4.12.4](https://www.cgal.org/):
```bash
sudo apt-get install libgmp3-dev libmpfr-dev -y
mkdir -p ~/installations/cgal
cd ~/installations/cgal
wget https://github.com/CGAL/cgal/releases/download/releases%2FCGAL-4.14.2/CGAL-4.14.2.tar.xz
tar -xf CGAL-4.14.2.tar.xz
cd CGAL-4.14.2/
cmake . -DCMAKE_BUILD_TYPE=Release
sudo make install
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

<details>
  <summary> <b>Optional (recommended for better performance)</b></summary>

To achieve better performance, you can use other linear solvers for Ipopt (instead of the default `mumps` solver). Specifically, we found that `MA27` and `MA57` are usually faster than the default `mumps` solver.

Go to [http://www.hsl.rl.ac.uk/ipopt/](http://www.hsl.rl.ac.uk/ipopt/), and then 

* If you want the solver `MA57` (or `MA27`, or both), click on `Coin-HSL Full (Stable) Source`. This is free for academia. 
* If you only want the solver `MA27`, click on `Personal Licence, Source`. This is free for everyone

And fill and submit the form. Then download the compressed file from the link of the email you receive. Uncompress that file, and place it in a folder `~/installations` (for example). Then execute the following commands:

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
</details>


<details>
  <summary> <b>Optional (only if you want to modify the optimization problem)</b></summary>

The easiest way to do this is to install casadi from binaries by simply following these commands:

````bash
cd ~/installations
wget https://github.com/casadi/casadi/releases/download/3.5.5/casadi-linux-matlabR2014b-v3.5.5.tar.gz
tar xvzf casadi-linux-matlabR2014b-v3.5.5.tar.gz
````

Open Matlab, execute the command `edit(fullfile(userpath,'startup.m'))`, and add the line `addpath('/usr/local/lib')` in that file. (This file is executed every time Matlab starts)

Then you can restart Matlab (or run the file above), and make sure this works: 

```bash
import casadi.*
x = MX.sym('x')
disp(jacobian(sin(x),x))

```

Then, to use a specific linear solver, you simply need to change the name of `linear_solver_name` in the file `main.m`, and then run that file.

> Note: Instead of the binary installation explained in this section, another (but not so straightforward) way would be to use the installation `from source` done above, but it requires some patches to swig, see [this](https://github.com/casadi/casadi/wiki/matlab).
</details>


### <ins>Compilation<ins>
```bash
cd ~/Desktop && mkdir ws && cd ws && mkdir src && cd src
git clone https://github.com/mit-acl/panther.git
cd panther
git submodule init && git submodule update
catkin build
echo "source /home/YOUR_USER/Desktop/ws/devel/setup.bash" >> ~/.bashrc 
```

### Running Simulations

`roslaunch panther single_agent_simulation.launch use_gui_mission:=true`. You can also the z value of `panther_specific.launch` to set the initial position of the drone

Now you can press `G` (or click the option `2D Nav Goal` on the top bar of RVIZ) and click any goal for the drone. 

## Credits:
This package uses some the [hungarian-algorithm-cpp](https://github.com/mcximing/hungarian-algorithm-cpp) and some C++ classes from the [DecompROS](https://github.com/sikang/DecompROS) and  repos (both included in the `thirdparty` folder), so credit to them as well. 
