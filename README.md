## LineCoverage-library
Library for Line Coverage with single and multiple robots:  
`slc` stands for single robot line coverage  
`mlc` stands for multiple robot line coverage

The algorithms are described in the following papers. Please cite our publications when using the library.

> Approximation algorithms for the single robot line coverage problem.  
> Agarwal S and Akella S (2020)  
> In: Algorithmic Foundations of Robotics XIV. Oulu, Finland, pp. 534–550.

> Line coverage with multiple robots.  
> Agarwal S and Akella S (2020)  
> In: IEEE International Conference on Robotics and Automation (ICRA). Paris, France, pp. 3248–3254.


## Installation
The library has been tested on Ubuntu and should work with any GNU/Linux system provided the following dependencies have been installed.
These packages are usually already installed.

`sudo apt-get install git build-essential cmake unzip`

The library uses `YAML-CPP` for parsing yaml configuration files:  
`sudo apt-get install libyaml-cpp-dev`

The library uses GLPK by default for solving ILP and LP. GLPK 5.0 has better performance than the older 4.x versions, and it can be downloaded from https://www.gnu.org/software/glpk/. On Debain-based systems you can install GLPK  using the following command:  
`sudo apt-get install libglpk-dev` (will probably install an older GLPK 4.x)

The library requires `gnuplot` for plotting. On Debian based systems `gnuplot` can be installed using the following command:  
`sudo apt-get install gnuplot`

If you are on a server:  
`sudo apt-get install gnuplot-nox`

#### Installation of LineCoverage-library (lclibrary):

Create a new directory where we will be cloning the repository and creating builds:

1. `mkdir ~/coverage && cd ~/coverage`   
2. `mkdir build`  
3. `mkdir install`  
4. `git clone https://github.com/UNCCharlotte-CS-Robotics/LineCoverage-library.git`    
5. `cmake -S LineCoverage-library/ -B ./build/ -DCMAKE_INSTALL_PREFIX=install/`  
6. `cmake --build build/`  
7. `cmake --install build/`  

You should be able to see the binary files `slc` and `mlc` in the install directory after this process.

#### Get the LineCoverage-dataset
`cd  ~/coverage`  
`git clone https://github.com/UNCCharlotte-CS-Robotics/LineCoverage-dataset.git`  

#### Run programs
`cd ~/coverage`  
`./install/bin/mlc install/bin/config/default_config.yaml`

Check the folder `LineCoverage-dataset/most_pop_50cities/paris/`. You should find the input data and the results.  
Make a copy of the file `install/bin/config/default_config.yaml` and change according to your preference.  
The file is also located at `LineCoverage-library/config/default_config.yaml`.

For details on usage see wiki: https://github.com/UNCCharlotte-CS-Robotics/LineCoverage-library/wiki/Usage

#### Install with Gurobi:
Obtain Gurobi from https://www.gurobi.com/  
License and installation instructions are available at https://www.gurobi.com/documentation/9.5/quickstart_linux/index.html. Gurobi provides free academic license.  

A helpful installation script is provided for convenient installation.  
Please set the installation directory in the script.  
`cd LineCoverage-library/external/gurobi_setup.sh`  
Change installation directory for `Gurobi` in the file `gurobi_setup.sh`  
`bash gurobi_setup.sh`  
`source ~/.bashrc`  
Activate Gurobi license

To install LineCoverage-library with Gurobi enabled, add the following flags to `cmake` flag in step 5:  
`-DLCLIBRARY_USE_GUROBI=ON`  
Execute installation steps 6 and 7.

#### Install with LKH and GLKH:

LKH and GLKH are very efficient solvers for the ATSP and the GTSP, respectively. The solvers for single robot line coverage problem (slc) can optionally use LKH and GLKH.  
Please visit the links below for information and terms of use:  
LKH: http://webhotel4.ruc.dk/~keld/research/LKH/  
GLKH: http://webhotel4.ruc.dk/~keld/research/GLKH/  

`cd LineCoverage-library/external`  
For LKH:  
`bash lkh_glkh_setup.sh -alkh`
For GLKH:  
`bash lkh_glkh_setup.sh -alkh -aglkh` (needs LKH)

`cd ~/coverage`  
To install LineCoverage-library with LKH and GLKH support, add the following flags to `cmake` command in step 5.
For LKH:  
`-DLCLIBRARY_USE_LKH=ON`  
For GLKH:  
`-DLCLIBRARY_USE_LKH=ON -DLCLIBRARY_USE_GLKH=ON`  
Execute installation steps 6 and 7.

Folders `glkh` and `lkh` inside `LineCoverage-library/external can be removed. You may also delete the `build` folder.

Note: LKH and GLKH are not part of this library. The programs makes system calls to the binaries.
In your `~/.bashrc` file add the following line so the GLKH can find the executables (required for only GLKH).  
`export PATH="${PATH}:${HOME}/coverage/install/bin/glkh"`  
`source ~/.bashrc`

## Contact
Users are requested to raise bugs to help improve the library. We are working on improving the documentation. Please use the GitHub issues, pull requests, and discussions.  
The program is authored by Saurav Agarwal during his PhD (advised by Srinivas Akella) at the University of North Carolina at Charlotte.  
Saurav Agarwal: https://webpages.uncc.edu/sagarw10/  
Srinivas Akella: https://webpages.uncc.edu/sakella/

## License and Acknowledgments
Copyright (C) 2020--2022 University of North Carolina at Charlotte.  
The LineCoverage-library is owned by the University of North Carolina at Charlotte and is protected by United States copyright laws and applicable international treaties and/or conventions.

The source code is licensed under GPLv3. Please see the LICENSE file.

DISCLAIMER OF WARRANTIES: THE SOFTWARE IS PROVIDED "AS-IS" WITHOUT WARRANTY OF ANY KIND INCLUDING ANY WARRANTIES OF PERFORMANCE OR MERCHANTABILITY OR FITNESS FOR A PARTICULAR USE OR PURPOSE OR OF NON-INFRINGEMENT. YOU BEAR ALL RISK RELATING TO QUALITY AND PERFORMANCE OF THE SOFTWARE OR HARDWARE.

The library uses the program from the following repository to solve Minimum-Cost Perfect-Matching and is covered by MIT license:  
https://github.com/dilsonpereira/Minimum-Cost-Perfect-Matching  
Modifications to the program are released in the following repository:  
https://github.com/AgarwalSaurav/mcpm

The following repository is used for parsing JSON:  
https://github.com/nlohmann/json

YAML-CPP is used to parse yaml configuration files:  
https://github.com/jbeder/yaml-cpp

Please see Gurobi and LKH license if you intend to use them.

Gurobi: https://www.gurobi.com/  
LKH: http://webhotel4.ruc.dk/~keld/research/LKH/  
GLKH: http://webhotel4.ruc.dk/~keld/research/GLKH/  
