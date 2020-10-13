# Set Up the NVIDIA Isaac Simulator

We used the [Isaac Sim 1.2](https://docs.nvidia.com/isaac/isaac_sim/index.html) for the photo-realistic simulation. We mainly used the provided environment and the Unreal Engine used by the simulator. Therefore, it should be easy to use other Unreal Engine-based simulators instead.

> Note that ISAAC 2019.2 is used, as it is the version available when we set up the experiments. Since then, there seems to be significant changes to the simulation setup from NVIDIA ([old UE4 simulator discontinued](https://forums.developer.nvidia.com/t/is-isaac-sim-no-longer-supporting-unreal-4/109324/2) and [the switch to the new simulator](https://forums.developer.nvidia.com/t/isaac-sim-2020-1-early-access-release-is-now-available-on-devzone-and-ngc/127587)). Unfortunately, the simulator we used is not supported by NVIDIA anymore. Therefore there might be issues with new versions of drivers and CUDA.

* [Install](#install)
  * [Step 0 Dependencies](#step-0-dependencies)
  * [Step 1 Install Isaac Sim 1.2](#step-1-install-isaac-sim-1.2)
  * [Step 2 Install UnrealCV from source](#step-2-install-unrealcv-from-source)
  * [Step 3 The modified warehouse environment](#step-3-the-modified-warehouse-environment)
* [Using the Simulator](#using-the-simulator)
* [Changing Camera Intrinsics for UnrealCV](#changing-camera-intrinsics-for-unrealcv)
* [Packaging](#packaging)

## Install

For the first two steps, we simply set up the Isaac Sim 1.2 and UnrealCV. And in the last step we replace the shipped [warehouse](https://docs.nvidia.com/isaac/isaac_sim/content/maps/warehouse.html) environment with our modified one and rebuild the map for rendering. Make sure that your machine satisfies [the requirement](https://docs.nvidia.com/isaac/archive/1.2/setup.html#machine-configuration).

### Step 0 Dependencies

The simulator is tested on Ubuntu 18.04 with the following setup:

* NVIDIA driver `440.33` (this is the version bundled with CUDA 10.2): install with` sudo apt install nvidia-driver-440`
* CUDA 10.2: install according to the instruction [here](https://developer.nvidia.com/cuda-10.2-download-archive?target_os=Linux&target_arch=x86_64&target_distro=Ubuntu&target_version=1804&target_type=deblocal)

* Vulkan 1.1.108.0: please download [here](https://vulkan.lunarg.com/sdk/home#sdk/downloadConfirm/1.1.108.0/linux/vulkansdk-linux-x86_64-1.1.108.0.tar.gz) and follow the instruction [here](https://docs.nvidia.com/isaac/isaac_sim/setup.html#install-the-vulkan-sdk) to setup

### Step 1 Install Isaac Sim 1.2

Below is a summary of [the official instruction](https://docs.nvidia.com/isaac/isaac_sim/setup.html). 

First, download the Isaac Sim description file from [here](https://developer.nvidia.com/isaacsimproject-121238533) (i.e., the Isaac 2019.2 release) and extract contained `xml` file.

Then run the following to setup the simulator

```sh
git clone --single-branch --branch IsaacSim_1.2 git@github.com:NvPhysX/UnrealEngine.git
cd UnrealEngine
rm Engine/Build/IsaacSimProject_1.2_Core.gitdeps.xml
cp <extracted_xml> ./Engine/Build
# this may take a while
./Setup.sh
./GenerateProjectFiles.sh
./GenerateTestRobotPaths.sh
# this will again take a while
make && make IsaacSimProjectEditor
```

**Verify**

Start the simulator

```sh
# this will take a while for the first time
./Engine/Binaries/Linux/UE4Editor IsaacSimProject CarterWarehouse_P
```

You should see a warehouse-like environment.

> The official instruction uses Vulkan (`-vulkan`). This however makes the editor crash constantly due to `VK_ERROR_DEVICE_LOST` error in our test setup. This error seems to be related to driver versions, and we could not identify the exact cause. Without `-vulkan`, OpenGL will be used and is sufficient for our purpose. You can try with `-vulkan` on your setup.



### Step 2 Install UnrealCV from source

Please following [the official instruction](http://docs.unrealcv.org/en/master/plugin/install.html#compile-from-source-code) to install from the source. Remember to replace the `--UE4` argument with where you place the simulator.

**Verify**

Start the simulator as before

```sh
./Engine/Binaries/Linux/UE4Editor IsaacSimProject CarterWarehouse_P
```

Start a python interpreter

```python
import unrealcv
unrealcv.client.connect()
```

and you should see a message about successful connection.

### Step 3 The modified warehouse environment
Download the modified warehouse environment from [here](http://rpg.ifi.uzh.ch/datasets/FIF/Warehouse_mod_ue4.zip) and replace the folder in `IsaacSimProject/Content/Maps/Warehouse` with the one in the downloaded zip file. Then start the editor as before, you should see a modified warehouse environment that looks like

![warehouse_env](./doc/isaac_env.gif)

> If you want to rebuild the environment, e.g., after making some changes, you should increase the number of allowed open file (before starting `UE4Editor`, in the same terminal):
> ```sh
> ulimit -n 10000
> ```



## Using the Simulator

As mentioned before, you can start the simulator from the command-line using 

```sh
./Engine/Binaries/Linux/UE4Editor IsaacSimProject CarterWarehouse_P
```

We will mostly use the simulator as a server to provide rendering functionalities. For this you need to click the Play button (on top right of the view port) and then leave the editor running in the background. The interaction will be done via the `UnrealCV` plugin and the `unrealcv_bridge` package.



## Changing Camera Intrinsics for UnrealCV

**You need to change the configuration file before you start the simulator**

Note that the visualization you see is using the settings from the editor, which is **not** the same as the rendering settings. Since we use the `UnrealCV`, the only reliable way of changing the camera parameters seems to be via the configuration file `Engine/Binaries/Linux/unrealcv.ini`, which looks like:

```ini
[UnrealCV.Core]                                                           
Port=9000                                                                 
Width=640                                                                 
Height=480                                                                
FOV=90.000000                                                             
EnableInput=True                                                          
EnableRightEye=False  
```

Please read [this thread](https://github.com/unrealcv/unrealcv/issues/11) for some more details. 

>  Note that UnrealCV also provides command line interface to change the [http://docs.unrealcv.org/en/latest/plugin/config.html](http://docs.unrealcv.org/en/latest/plugin/config.html) and [horizontal FoV](http://docs.unrealcv.org/en/latest/reference/commands.html#camera-operation). We could change the FoV but not the resolution via its command system. Therefore we resort to the configuration file.



## Packaging

We can also package the map to be a standalone application, without relying on the Unreal Engine Editor.

First, add the following line to `Engine\Config\ConsoleVariables.ini`

```
r.ForceDebugViewModes = 1
```

for [UnrealCV to work properly](http://docs.unrealcv.org/en/latest/plugin/package.html#modify-an-ue4-config-file).

Second, in the Editor, change the project's default map to `CarterWarehouse_P` according to the instruction [here](https://docs.unrealengine.com/en-US/Engine/Basics/Projects/Packaging/index.html).

Third, only select the map you want to package (here `CarterWarehouse_P`) under **Project Settings -> Packaging** (i.e., delete the other maps).

Finally, use **File -> Package Project -> Linux** to start package - and this is how we created the [provided standalone simulator](http://rpg.ifi.uzh.ch/datasets/FIF/warehouse_bin.zip).

Please refer to the [official documentation](https://docs.unrealengine.com/en-US/Engine/Basics/Projects/Packaging/index.html) and [UnrealCV documentation](http://docs.unrealcv.org/en/latest/plugin/package.html) for more details.



