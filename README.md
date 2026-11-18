# Universal Manipulation Interface

[[Project page]](https://umi-gripper.github.io/)
[[Paper]](https://umi-gripper.github.io/#paper)
[[Hardware Guide]](https://docs.google.com/document/d/1TPYwV9sNVPAi0ZlAupDMkXZ4CA1hsZx7YDMSmcEy6EU/edit?usp=sharing)
[[Data Collection Instruction]](https://swanky-sphere-ad1.notion.site/UMI-Data-Collection-Tutorial-4db1a1f0f2aa4a2e84d9742720428b4c?pvs=4)
[[SLAM repo]](https://github.com/cheng-chi/ORB_SLAM3)
[[SLAM docker]](https://hub.docker.com/r/chicheng/orb_slam3)

<img width="90%" src="assets/umi_teaser.png">

[Cheng Chi](http://cheng-chi.github.io/)<sup>1,2</sup>,
[Zhenjia Xu](https://www.zhenjiaxu.com/)<sup>1,2</sup>,
[Chuer Pan](https://chuerpan.com/)<sup>1</sup>,
[Eric Cousineau](https://www.eacousineau.com/)<sup>3</sup>,
[Benjamin Burchfiel](http://www.benburchfiel.com/)<sup>3</sup>,
[Siyuan Feng](https://www.cs.cmu.edu/~sfeng/)<sup>3</sup>,

[Russ Tedrake](https://groups.csail.mit.edu/locomotion/russt.html)<sup>3</sup>,
[Shuran Song](https://www.cs.columbia.edu/~shurans/)<sup>1,2</sup>

<sup>1</sup>Stanford University,
<sup>2</sup>Columbia University,
<sup>3</sup>Toyota Research Institute

## 🛠️ Installation
Only tested on Ubuntu 22.04

Install docker following the [official documentation](https://docs.docker.com/engine/install/ubuntu/) and finish [linux-postinstall](https://docs.docker.com/engine/install/linux-postinstall/).

Install system-level dependencies:
```console
$ sudo apt install -y libosmesa6-dev libgl1-mesa-glx libglfw3 patchelf
```

We recommend [Miniforge](https://github.com/conda-forge/miniforge?tab=readme-ov-file#miniforge3) instead of the standard anaconda distribution for faster installation: 
```console
$ mamba env create -f conda_environment.yaml
```

Activate environment
```console
$ conda activate umi
(umi)$ 
```

## Running UMI SLAM pipeline
Download example data
```console
(umi)$ wget --recursive --no-parent --no-host-directories --cut-dirs=2 --relative --reject="index.html*" https://real.stanford.edu/umi/data/example_demo_session/
```

Run SLAM pipeline
```console
(umi)$ python run_slam_pipeline.py example_demo_session

...
Found following cameras:
camera_serial
C3441328164125    5
Name: count, dtype: int64
Assigned camera_idx: right=0; left=1; non_gripper=2,3...
             camera_serial  gripper_hw_idx                                     example_vid
camera_idx                                                                                
0           C3441328164125               0  demo_C3441328164125_2024.01.10_10.57.34.882133
99% of raw data are used.
defaultdict(<function main.<locals>.<lambda> at 0x7f471feb2310>, {})
n_dropped_demos 0
````
For this dataset, 99% of the data are useable (successful SLAM), with 0 demonstrations dropped. If your dataset has a low SLAM success rate, double check if you carefully followed our [data collection instruction](https://swanky-sphere-ad1.notion.site/UMI-Data-Collection-Instruction-4db1a1f0f2aa4a2e84d9742720428b4c). 

Despite our significant effort on robustness improvement, OBR_SLAM3 is still the most fragile part of UMI pipeline. If you are an expert in SLAM, please consider contributing to our fork of [OBR_SLAM3](https://github.com/cheng-chi/ORB_SLAM3) which is specifically optimized for UMI workflow.

Generate dataset for training.
```console
(umi)$ python scripts_slam_pipeline/07_generate_replay_buffer.py -o example_demo_session/dataset.zarr.zip example_demo_session
```

### Using an Insta360 camera instead of a GoPro
The pipeline also accepts takes from an Insta360 X3 in either of its two recording layouts:
* **Packed.** One `.insv` per take with both lenses side by side in one stream, e.g. 3840x1920. Takes started over USB with `insta360_record.py` use this layout.
* **Split.** One `.insv` per lens (`_00_`, `_10_`), e.g. 5.7K at 2880x2880 per lens. Takes started with the camera's own button in 5.7K video mode use this layout. The IMU and lens calibration of a split take are read from its LRV proxy, so a split take needs the lens file and the LRV.

Each take carries 1000 Hz IMU data and Insta360's factory lens calibration. The pipeline keeps one lens, lens 1 by default, which faces the gripper on our mount. Both layouts are the same sensor crop at different sizes, so one calibration dir serves both.

Build a calibration dir once per camera. Give it any file of a take where the camera is rotated briskly about all three axes for 20 to 60 seconds, so the camera-IMU rotation can be estimated from gyro against visual rotation:
```console
(umi)$ python scripts/insta360_calibration.py -i VID_xxx.insv -o data/insta360_calib --lens 1
```
Check `cam_imu_report.json`. If its `source` says `default`, the clip did not rotate enough to estimate from. For lens 1 the default rotation is the Basalt calibration of X3 `IAQEB2606EEWG9` from RoboManipBaselines. Lens 1 always takes its translation from that same calibration. For lens 0, or for another camera, the default is an unverified guess with zero translation. Set your printed tag sizes in the dir's `aruco_config.yaml`; rerunning the script keeps that file.

The camera scripts need the Python that has pyinsta360 installed (Python 3.10+) and the camera on USB in Android mode.

**Recording over USB.** Each take records until Enter, or for `--seconds N`, and downloads into the session:
```console
$ python scripts/insta360_record.py <session> --kind mapping
$ python scripts/insta360_record.py <session> --kind gripper_calibration
$ python scripts/insta360_record.py <session>
```

**Recording without a PC.** Select Video mode on the camera, record with its button, then connect it, list the takes on the card and import them by index and role. Only the files the pipeline needs are downloaded, and nothing is deleted from the camera:
```console
$ python scripts/insta360_import.py <session> --list
$ python scripts/insta360_import.py <session> --mapping 13 --gripper-calibration 11 --demo 12 14-20
```

Both tools put a mapping take in `raw_videos/mapping/`, a gripper calibration take in `raw_videos/gripper_calibration/`, and demos in `raw_videos/`. Then run the pipeline with the calibration dir:
```console
(umi)$ python run_slam_pipeline.py <session> -c data/insta360_calib
```
`00_convert_insv.py` turns each take into a single-lens `.mp4` and an `imu_data.json` with the same metadata GoPro files carry, then moves the take's camera files to `<session>/raw_insv/`. Start times come from the camera's file names, which matched the PC clock to 1 s; the files' `creation_time` runs about 10 s early and is UTC or local time depending on how the take was started. Steps 02 and 03 read `slam_setting.yaml` and `slam_mask.png` from the calibration dir. The mask covers the area outside the lens field of view and the gripper on our mount (`GRIPPER_POLYGON` in `scripts/insta360_calibration.py`). Adjust it for yours (255 = ignored) before mapping.

## Training Diffusion Policy
Single-GPU training. Tested to work on RTX3090 24GB.
```console
(umi)$ python train.py --config-name=train_diffusion_unet_timm_umi_workspace task.dataset_path=example_demo_session/dataset.zarr.zip
```

Multi-GPU training.
```console
(umi)$ accelerate --num_processes <ngpus> train.py --config-name=train_diffusion_unet_timm_umi_workspace task.dataset_path=example_demo_session/dataset.zarr.zip
```

Downloading in-the-wild cup arrangement dataset (processed).
```console
(umi)$ wget https://real.stanford.edu/umi/data/zarr_datasets/cup_in_the_wild.zarr.zip
```

Multi-GPU training.
```console
(umi)$ accelerate --num_processes <ngpus> train.py --config-name=train_diffusion_unet_timm_umi_workspace task.dataset_path=cup_in_the_wild.zarr.zip
```

## 🦾 Real-world Deployment
In this section, we will demonstrate our real-world deployment/evaluation system with the cup arrangement policy. While this policy setup only requires a single arm and camera, the our system supports up to 2 arms and unlimited number of cameras.

### ⚙️ Hardware Setup
1. Build deployment hardware according to our [Hardware Guide](https://docs.google.com/document/d/1TPYwV9sNVPAi0ZlAupDMkXZ4CA1hsZx7YDMSmcEy6EU).
2. Setup UR5 with teach pendant:
    * Obtain IP address and update [eval_robots_config.yaml](example/eval_robots_config.yaml)/robots/robot_ip.
    * In Installation > Payload
        * Set mass to 1.81 kg
        * Set center of gravity to (2, -6, 37)mm, CX/CY/CZ.
    * TCP will be set automatically by the eval script.
    * On UR5e, switch control mode to remote.

    If you are using Franka, follow this [instruction](franka_instruction.md).
3. Setup WSG50 gripper with web interface:
    * Obtain IP address and update [eval_robots_config.yaml](example/eval_robots_config.yaml)/grippers/gripper_ip.
    * In Settings > Command Interface
        * Disable "Use text based Interface"
        * Enable CRC
    * In Scripting > File Manager
        * Upload [umi/real_world/cmd_measure.lua](umi/real_world/cmd_measure.lua)
    * In Settings > System
        * Enable Startup Script
        * Select `/user/cmd_measure.lua` you just uploaded.
4. Setup GoPro:
    * Install GoPro Labs [firmware](https://gopro.com/en/us/info/gopro-labs).
    * Set date and time.
    * Scan the following QR code for clean HDMI output 
    <br><img width="50%" src="assets/QR-MHDMI1mV0r27Tp60fWe0hS0sLcFg1dV.png">
5. Setup [3Dconnexion SpaceMouse](https://www.amazon.com/3Dconnexion-SpaceMouse-Wireless-universal-receiver/dp/B079V367MM):
    * Install libspnav `sudo apt install libspnav-dev spacenavd`
    * Start spnavd `sudo systemctl start spacenavd`

### 🤗 Reproducing the Cup Arrangement Policy ☕
Our in-the-wild cup arragement policy is trained with the distribution of ["espresso cup with saucer"](https://www.amazon.com/s?k=espresso+cup+with+saucer) on Amazon across 30 different locations around Stanford. We created a [Amazon shopping list](https://www.amazon.com/hz/wishlist/ls/Q0T8U2N5U3IU?ref_=wl_share) for all cups used for training. We published the processed [Zarr dataset and](https://real.stanford.edu/umi/data/zarr_datasets) pre-trained [checkpoint](https://real.stanford.edu/umi/data/pretrained_models/) (finetuned CLIP ViT-L backbone).

<img width="90%" src="assets/umi_cup.gif">

Download pre-trained checkpoint.
```console
(umi)$ wget https://real.stanford.edu/umi/data/pretrained_models/cup_wild_vit_l_1img.ckpt
```

Grant permission to the HDMI capture card.
```console
(umi)$ sudo chmod -R 777 /dev/bus/usb
```

Launch eval script.
```console
(umi)$ python eval_real.py --robot_config=example/eval_robots_config.yaml -i cup_wild_vit_l.ckpt -o data/eval_cup_wild_example
```
After the script started, use your spacemouse to control the robot and the gripper (spacemouse buttons). Press `C` to start the policy. Press `S` to stop.

If everything are setup correctly, your robot should be able to rotate the cup and placing it onto the saucer, anywhere 🎉

Known issue ⚠️: The policy doesn't work well under direct sunlight, since the dataset was collected during a rainiy week at Stanford.

### 🤗 Reproducing Policies on ARX X5 Robot Arms
Please follow [umi-on-legs](https://github.com/real-stanford/umi-on-legs) for hardware modification and [umi-arx](https://github.com/real-stanford/umi-arx) for detailed policy deployment instructions. 

<img width="90%" src="assets/umi_cup_arx.gif">

## 🏷️ License
This repository is released under the MIT license. See [LICENSE](LICENSE) for additional details.

## 🙏 Acknowledgement
* Our GoPro SLAM pipeline is adapted from [Steffen Urban](https://github.com/urbste)'s [fork](https://github.com/urbste/ORB_SLAM3) of [OBR_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3).
* We used [Steffen Urban](https://github.com/urbste)'s [OpenImuCameraCalibrator](https://github.com/urbste/OpenImuCameraCalibrator/) for camera and IMU calibration.
* The UMI gripper's core mechanism is adpated from [Push/Pull Gripper](https://www.thingiverse.com/thing:2204113) by [John Mulac](https://www.thingiverse.com/3dprintingworld/designs).
* UMI's soft finger is adapted from [Alex Alspach](http://alexalspach.com/)'s original design at TRI.
