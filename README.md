# Soft Body Gait Discovery
In this work, we investigate the question of gait discovery for deformable characters without skeletons. Without skeletons, creatures must rely on volume preservation and muscle contractions for control. We model muscle contractions using combination of both isotropic and non-isotropic expansions of deformable voxels. Each muscle voxel expands and contracts following a sinusoidal schedule which are optimized using CMA-ES to maximize forward movement.

[Project webpage]()

## Basic Usage
As of currently, only windows machines are supported. Please open the project with Visual Studio 2017 and run the project.

Simulator settings which includes material properties, body plans, muscle oscillation schedules, and more are `prop.vxl.json`. The read version is a modified version of the [original JSON format](https://github.com/jonhiller/Voxelyze/wiki/Voxelyze-JSON-format) with several additions. Please refer below for more information.

The program have three modes of operation: `Display`, `Replay`, `Optimize`. Controls are mostly set near the top of `main.cpp` after the imports.

#### Display
`Display` runs the simulation according to the JSON configuration. It can be set to record voxel configurations for replay. The specified recording rate can be set using `frameTime` and the default time at which a recording is generated can be set with `recordTime` and the save location is set using `saveFile`.

#### Replay
`Replay` replays the voxel configurations read in from the specified file. The playback speed can be set using `msec_replay`. The read location is set using `loadFile`.

##### Controls for Display/Replay
* AD to move camera along X axis
* WS to move camera along Y axis
* QE to move camera along Z axis
* R to reset to beginning
* P to pause simulation/replay
* U to instantly save replay.

#### Optimize
Additional optimization controls is set in `cmaesNewLibTest.cpp` for the number of parameters. The initial parameters and hyperparameters for CMA-ES are also set in `cmaesNewLibTest.cpp`. By default, every time a new record is achieved, a JSON file containing that configuration is saved in the `data` directory. One can change the name of the saved file on line 85.

### Body Plan Design

A utility called `convertCoords.py` in `softbody_util` allows users to design body plans in [VoxCAD](https://sites.google.com/site/voxcadproject/) and export them into the simulation.

Usage: `python convertCoords.py inFile.txt outFile.txt`

To obtain `inFile.txt`, create your design in VoxCAD. Set Lattice Dim under the Workspace tab to 1000mm and export the file as Voxel Coordinates from `File -> Export -> Voxel Coordinates`. Then, put the export file into the `softbody_util` directory and input its name in place of `inFile.txt`.

## Dependencies
* [libcmaes](https://github.com/beniz/libcmaes) compiled for Windows using Cygwin
* Eigen
* Glew and FreeGlut
* Forked version of [Voxelyze](https://github.com/chunloklo/Voxelyze) with non-isotropic expansions/contractions

One have to manually set additional include/linker/library directories as needed for the VS2017 project.

### Modification to JSON format

The following are additions to the JSON format
#### optConfig
Controls how parameters are loaded into the simulation.

| Properties        | Description|
| ------------- |-------------|
| numMaterials     | Number of muscle activations to load parameters for. |
| optAmp      | Whether to load amplitude information      |
| optFreq | Whether to load frequency information      |
| optOff |Whether to load offset information      |

#### Muscle Axis
Under materials, controls which axis the material will mainly expand towards. Axis number of {-1, 0, 1, 2} correspond with {uniform, X, Y, Z} axis expansions.

#### Material Oscillation
Controls how muscles are activated according to the sin function. Controls temperature per material to obtain expansions/contractions. One can set the amplitude, frequency, and offset for the activation.

`materialNames` sets the correspondence between materials and the oscillation. It is possible to set multiple materials to be controlled by the same oscillation/activation. To use activations, one MUST set the names for materials correspondingly.
