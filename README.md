# PROS AI Image

The folder `ros` refers to the [GitHub](https://github.com/dusty-nv/jetson-containers/tree/master/packages/ros) repository.

This project aims to build an image containing `pytorch` and `ros2` for the Jetson Orin Nano Developer Kit (arm64) and x86_64 platform PC (amd64).

**Warning**: The function `torch.cuda.is_available()` returns False in Jetson Orin.



## Colcon

The `bash` shell will always do `colcon build` every time.

You can mount your `src` folder to `/workspaces/src` in the container.



## Note

- This image has removed the source code of

  - `csm`
  - `ros2_laser_scan_matcher`
  - `rplidar_ros`
  - `slam_toolbox`
  
- Manually build the image

  ```
  docker buildx create --use --platform=linux/arm64,linux/amd64 --name multi-platform-builder
  docker buildx inspect --bootstrap
  docker buildx build --platform=linux/arm64,linux/amd64 --push --tag ghcr.io/otischung/pros_ai_image:latest -f ./Dockerfile.ros2 .
  ```

  Reference: https://unix.stackexchange.com/questions/748633/error-multiple-platforms-feature-is-currently-not-supported-for-docker-driver



## Note for the Base Image: ros2-humble

- There is the file `/ros_entrypoint.sh`

  ```bash
  #!/bin/bash
  set -e
  # setup ros2 environment
  # source "$ROS2_WS/install/setup.bash"
  source /.bashrc
  ```

  

- The content in `/.bashrc` is shown below

  ```bash
  # Enable the subsequent settings only in interactive sessions
  case $- in
    *i*) ;;
      *) return;;
  esac
  
  # Path to your oh-my-bash installation.
  export OSH='/root/.oh-my-bash'
  
  # Set name of the theme to load. Optionally, if you set this to "random"
  # it'll load a random theme each time that oh-my-bash is loaded.
  #OSH_THEME="font"
  OSH_THEME='agnoster'
  # Uncomment the following line to use case-sensitive completion.
  # OMB_CASE_SENSITIVE="true"
  
  # Uncomment the following line to use hyphen-insensitive completion. Case
  # sensitive completion must be off. _ and - will be interchangeable.
  # OMB_HYPHEN_SENSITIVE="false"
  
  # Uncomment the following line to disable bi-weekly auto-update checks.
  # DISABLE_AUTO_UPDATE="true"
  
  # Uncomment the following line to change how often to auto-update (in days).
  # export UPDATE_OSH_DAYS=13
  
  # Uncomment the following line to disable colors in ls.
  # DISABLE_LS_COLORS="true"
  
  # Uncomment the following line to disable auto-setting terminal title.
  # DISABLE_AUTO_TITLE="true"
  
  # Uncomment the following line to enable command auto-correction.
  # ENABLE_CORRECTION="true"
  
  # Uncomment the following line to display red dots whilst waiting for completion.
  # COMPLETION_WAITING_DOTS="true"
  
  # Uncomment the following line if you want to disable marking untracked files
  # under VCS as dirty. This makes repository status check for large repositories
  # much, much faster.
  # DISABLE_UNTRACKED_FILES_DIRTY="true"
  
  # Uncomment the following line if you don't want the repository to be considered dirty
  # if there are untracked files.
  # SCM_GIT_DISABLE_UNTRACKED_DIRTY="true"
  
  # Uncomment the following line if you want to completely ignore the presence
  # of untracked files in the repository.
  # SCM_GIT_IGNORE_UNTRACKED="true"
  
  # Uncomment the following line if you want to change the command execution time
  # stamp shown in the history command output.  One of the following values can
  # be used to specify the timestamp format.
  # * 'mm/dd/yyyy'     # mm/dd/yyyy + time
  # * 'dd.mm.yyyy'     # dd.mm.yyyy + time
  # * 'yyyy-mm-dd'     # yyyy-mm-dd + time
  # * '[mm/dd/yyyy]'   # [mm/dd/yyyy] + [time] with colors
  # * '[dd.mm.yyyy]'   # [dd.mm.yyyy] + [time] with colors
  # * '[yyyy-mm-dd]'   # [yyyy-mm-dd] + [time] with colors
  # If not set, the default value is 'yyyy-mm-dd'.
  # HIST_STAMPS='yyyy-mm-dd'
  
  # Uncomment the following line if you do not want OMB to overwrite the existing
  # aliases by the default OMB aliases defined in lib/*.sh
  # OMB_DEFAULT_ALIASES="check"
  
  # Would you like to use another custom folder than $OSH/custom?
  # OSH_CUSTOM=/path/to/new-custom-folder
  
  # To disable the uses of "sudo" by oh-my-bash, please set "false" to
  # this variable.  The default behavior for the empty value is "true".
  OMB_USE_SUDO=true
  
  # To enable/disable display of Python virtualenv and condaenv
  # OMB_PROMPT_SHOW_PYTHON_VENV=true  # enable
  # OMB_PROMPT_SHOW_PYTHON_VENV=false # disable
  
  # Which completions would you like to load? (completions can be found in ~/.oh-my-bash/completions/*)
  # Custom completions may be added to ~/.oh-my-bash/custom/completions/
  # Example format: completions=(ssh git bundler gem pip pip3)
  # Add wisely, as too many completions slow down shell startup.
  completions=(
    git
    git_flow
    composer
    ssh
    pip3
  )
  
  # Which aliases would you like to load? (aliases can be found in ~/.oh-my-bash/aliases/*)
  # Custom aliases may be added to ~/.oh-my-bash/custom/aliases/
  # Example format: aliases=(vagrant composer git-avh)
  # Add wisely, as too many aliases slow down shell startup.
  aliases=(
    general
  )
  
  # Which plugins would you like to load? (plugins can be found in ~/.oh-my-bash/plugins/*)
  # Custom plugins may be added to ~/.oh-my-bash/custom/plugins/
  # Example format: plugins=(rails git textmate ruby lighthouse)
  # Add wisely, as too many plugins slow down shell startup.
  plugins=(
    git
    pyenv
    sudo
    bashmarks
  )
  
  # Which plugins would you like to conditionally load? (plugins can be found in ~/.oh-my-bash/plugins/*)
  # Custom plugins may be added to ~/.oh-my-bash/custom/plugins/
  # Example format:
  #  if [ "$DISPLAY" ] || [ "$SSH" ]; then
  #      plugins+=(tmux-autoattach)
  #  fi
  
  source "$OSH"/oh-my-bash.sh
  
  # User configuration
  # export MANPATH="/usr/local/man:$MANPATH"
  
  # You may need to manually set your language environment
  # export LANG=en_US.UTF-8
  
  # Preferred editor for local and remote sessions
  # if [[ -n $SSH_CONNECTION ]]; then
  #   export EDITOR='vim'
  # else
  #   export EDITOR='mvim'
  # fi
  
  # Compilation flags
  # export ARCHFLAGS="-arch x86_64"
  
  # ssh
  # export SSH_KEY_PATH="~/.ssh/rsa_id"
  
  # Set personal aliases, overriding those provided by oh-my-bash libs,
  # plugins, and themes. Aliases can be placed here, though oh-my-bash
  # users are encouraged to define aliases within the OSH_CUSTOM folder.
  # For a full list of active aliases, run `alias`.
  #
  # Example aliases
  # alias bashconfig="mate ~/.bashrc"
  # alias ohmybash="mate ~/.oh-my-bash"
  
  
  source /opt/ros/humble/setup.bash
  source /etc/profile.d/bash_completion.sh
  export PATH="/usr/lib/ccache:$PATH"
  source /opt/ros/humble/setup.bash
  source /etc/profile.d/bash_completion.sh
  source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
  ```

- The content in `/root/.bashrc` is shown below

  ```bash
  # Enable the subsequent settings only in interactive sessions
  case $- in
    *i*) ;;
      *) return;;
  esac
  
  # Path to your oh-my-bash installation.
  export OSH='/root/.oh-my-bash'
  
  # Set name of the theme to load. Optionally, if you set this to "random"
  # it'll load a random theme each time that oh-my-bash is loaded.
  OSH_THEME="font"
  
  # Uncomment the following line to use case-sensitive completion.
  # OMB_CASE_SENSITIVE="true"
  
  # Uncomment the following line to use hyphen-insensitive completion. Case
  # sensitive completion must be off. _ and - will be interchangeable.
  # OMB_HYPHEN_SENSITIVE="false"
  
  # Uncomment the following line to disable bi-weekly auto-update checks.
  # DISABLE_AUTO_UPDATE="true"
  
  # Uncomment the following line to change how often to auto-update (in days).
  # export UPDATE_OSH_DAYS=13
  
  # Uncomment the following line to disable colors in ls.
  # DISABLE_LS_COLORS="true"
  
  # Uncomment the following line to disable auto-setting terminal title.
  # DISABLE_AUTO_TITLE="true"
  
  # Uncomment the following line to enable command auto-correction.
  # ENABLE_CORRECTION="true"
  
  # Uncomment the following line to display red dots whilst waiting for completion.
  # COMPLETION_WAITING_DOTS="true"
  
  # Uncomment the following line if you want to disable marking untracked files
  # under VCS as dirty. This makes repository status check for large repositories
  # much, much faster.
  # DISABLE_UNTRACKED_FILES_DIRTY="true"
  
  # Uncomment the following line if you don't want the repository to be considered dirty
  # if there are untracked files.
  # SCM_GIT_DISABLE_UNTRACKED_DIRTY="true"
  
  # Uncomment the following line if you want to completely ignore the presence
  # of untracked files in the repository.
  # SCM_GIT_IGNORE_UNTRACKED="true"
  
  # Uncomment the following line if you want to change the command execution time
  # stamp shown in the history command output.  One of the following values can
  # be used to specify the timestamp format.
  # * 'mm/dd/yyyy'     # mm/dd/yyyy + time
  # * 'dd.mm.yyyy'     # dd.mm.yyyy + time
  # * 'yyyy-mm-dd'     # yyyy-mm-dd + time
  # * '[mm/dd/yyyy]'   # [mm/dd/yyyy] + [time] with colors
  # * '[dd.mm.yyyy]'   # [dd.mm.yyyy] + [time] with colors
  # * '[yyyy-mm-dd]'   # [yyyy-mm-dd] + [time] with colors
  # If not set, the default value is 'yyyy-mm-dd'.
  # HIST_STAMPS='yyyy-mm-dd'
  
  # Uncomment the following line if you do not want OMB to overwrite the existing
  # aliases by the default OMB aliases defined in lib/*.sh
  # OMB_DEFAULT_ALIASES="check"
  
  # Would you like to use another custom folder than $OSH/custom?
  # OSH_CUSTOM=/path/to/new-custom-folder
  
  # To disable the uses of "sudo" by oh-my-bash, please set "false" to
  # this variable.  The default behavior for the empty value is "true".
  OMB_USE_SUDO=true
  
  # To enable/disable display of Python virtualenv and condaenv
  # OMB_PROMPT_SHOW_PYTHON_VENV=true  # enable
  # OMB_PROMPT_SHOW_PYTHON_VENV=false # disable
  
  # Which completions would you like to load? (completions can be found in ~/.oh-my-bash/completions/*)
  # Custom completions may be added to ~/.oh-my-bash/custom/completions/
  # Example format: completions=(ssh git bundler gem pip pip3)
  # Add wisely, as too many completions slow down shell startup.
  completions=(
    git
    composer
    ssh
  )
  
  # Which aliases would you like to load? (aliases can be found in ~/.oh-my-bash/aliases/*)
  # Custom aliases may be added to ~/.oh-my-bash/custom/aliases/
  # Example format: aliases=(vagrant composer git-avh)
  # Add wisely, as too many aliases slow down shell startup.
  aliases=(
    general
  )
  
  # Which plugins would you like to load? (plugins can be found in ~/.oh-my-bash/plugins/*)
  # Custom plugins may be added to ~/.oh-my-bash/custom/plugins/
  # Example format: plugins=(rails git textmate ruby lighthouse)
  # Add wisely, as too many plugins slow down shell startup.
  plugins=(
    git
    bashmarks
  )
  
  # Which plugins would you like to conditionally load? (plugins can be found in ~/.oh-my-bash/plugins/*)
  # Custom plugins may be added to ~/.oh-my-bash/custom/plugins/
  # Example format:
  #  if [ "$DISPLAY" ] || [ "$SSH" ]; then
  #      plugins+=(tmux-autoattach)
  #  fi
  
  source "$OSH"/oh-my-bash.sh
  
  # User configuration
  # export MANPATH="/usr/local/man:$MANPATH"
  
  # You may need to manually set your language environment
  # export LANG=en_US.UTF-8
  
  # Preferred editor for local and remote sessions
  # if [[ -n $SSH_CONNECTION ]]; then
  #   export EDITOR='vim'
  # else
  #   export EDITOR='mvim'
  # fi
  
  # Compilation flags
  # export ARCHFLAGS="-arch x86_64"
  
  # ssh
  # export SSH_KEY_PATH="~/.ssh/rsa_id"
  
  # Set personal aliases, overriding those provided by oh-my-bash libs,
  # plugins, and themes. Aliases can be placed here, though oh-my-bash
  # users are encouraged to define aliases within the OSH_CUSTOM folder.
  # For a full list of active aliases, run `alias`.
  #
  # Example aliases
  # alias bashconfig="mate ~/.bashrc"
  # alias ohmybash="mate ~/.oh-my-bash"
  ```



- We need the content in `/.bashrc`, but the initial run command is `/root/.bashrc`. Therefore, we move `/.bashrc` into `/root/`.



# Dockerfile.ros2

> [`CONTAINERS`](#user-content-containers) [`IMAGES`](#user-content-images) [`RUN`](#user-content-run) [`BUILD`](#user-content-build)

> [!NOTE]  
> <a href="https://github.com/NVIDIA-ISAAC-ROS"><b>NVIDIA Isaac ROS</b></a><br/>
> See [`github.com/NVIDIA-ISAAC-ROS`](https://github.com/NVIDIA-ISAAC-ROS) for optimized CV/AI packages and [NITROS](https://nvidia-isaac-ros.github.io/concepts/nitros/index.html) zero-copy transport.<br/>
> For Isaac ROS containers, use the [Isaac ROS Docker Development Environment](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_common/index.html#isaac-ros-docker-development-environment).

Below are ROS/ROS2 base containers for JetPack.  These build ROS from source to run them on the needed versions of Ubuntu.

Supported ROS distros: `melodic` `noetic` `foxy` `galactic` `humble` `iron` <br/>
Supported ROS packages: `ros_base` `ros_core` `desktop`
<details open>
<summary><b><a id="containers">CONTAINERS</a></b></summary>
<br>

| **`ros:melodic-ros-base`** | |
| :-- | :-- |
| &nbsp;&nbsp;&nbsp;Builds | [![`ros-melodic-ros-base_jp46`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-melodic-ros-base_jp46.yml?label=ros-melodic-ros-base:jp46)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-melodic-ros-base_jp46.yml) |
| &nbsp;&nbsp;&nbsp;Requires | `L4T <34` |
| &nbsp;&nbsp;&nbsp;Dependencies | [`build-essential`](/packages/build-essential) [`cmake:apt`](/packages/cmake/cmake_apt) [`python`](/packages/python) |
| &nbsp;&nbsp;&nbsp;Dockerfile | [`Dockerfile.ros.melodic`](Dockerfile.ros.melodic) |
| &nbsp;&nbsp;&nbsp;Images | [`dustynv/ros:melodic-ros-base-l4t-r32.4.4`](https://hub.docker.com/r/dustynv/ros/tags) `(2021-08-06, 0.5GB)`<br>[`dustynv/ros:melodic-ros-base-l4t-r32.5.0`](https://hub.docker.com/r/dustynv/ros/tags) `(2021-09-23, 0.5GB)`<br>[`dustynv/ros:melodic-ros-base-l4t-r32.6.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2022-03-02, 0.5GB)`<br>[`dustynv/ros:melodic-ros-base-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 0.5GB)` |
| &nbsp;&nbsp;&nbsp;Notes | ROS Melodic is for JetPack 4 only |

| **`ros:melodic-ros-core`** | |
| :-- | :-- |
| &nbsp;&nbsp;&nbsp;Builds | [![`ros-melodic-ros-core_jp46`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-melodic-ros-core_jp46.yml?label=ros-melodic-ros-core:jp46)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-melodic-ros-core_jp46.yml) |
| &nbsp;&nbsp;&nbsp;Requires | `L4T <34` |
| &nbsp;&nbsp;&nbsp;Dependencies | [`build-essential`](/packages/build-essential) [`cmake:apt`](/packages/cmake/cmake_apt) [`python`](/packages/python) |
| &nbsp;&nbsp;&nbsp;Dockerfile | [`Dockerfile.ros.melodic`](Dockerfile.ros.melodic) |
| &nbsp;&nbsp;&nbsp;Images | [`dustynv/ros:melodic-ros-core-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 0.5GB)` |
| &nbsp;&nbsp;&nbsp;Notes | ROS Melodic is for JetPack 4 only |

| **`ros:melodic-desktop`** | |
| :-- | :-- |
| &nbsp;&nbsp;&nbsp;Builds | [![`ros-melodic-desktop_jp46`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-melodic-desktop_jp46.yml?label=ros-melodic-desktop:jp46)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-melodic-desktop_jp46.yml) |
| &nbsp;&nbsp;&nbsp;Requires | `L4T <34` |
| &nbsp;&nbsp;&nbsp;Dependencies | [`build-essential`](/packages/build-essential) [`cmake:apt`](/packages/cmake/cmake_apt) [`python`](/packages/python) |
| &nbsp;&nbsp;&nbsp;Dockerfile | [`Dockerfile.ros.melodic`](Dockerfile.ros.melodic) |
| &nbsp;&nbsp;&nbsp;Images | [`dustynv/ros:melodic-desktop-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 0.7GB)` |
| &nbsp;&nbsp;&nbsp;Notes | ROS Melodic is for JetPack 4 only |

| **`ros:noetic-ros-base`** | |
| :-- | :-- |
| &nbsp;&nbsp;&nbsp;Builds | [![`ros-noetic-ros-base_jp46`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-noetic-ros-base_jp46.yml?label=ros-noetic-ros-base:jp46)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-noetic-ros-base_jp46.yml) [![`ros-noetic-ros-base_jp51`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-noetic-ros-base_jp51.yml?label=ros-noetic-ros-base:jp51)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-noetic-ros-base_jp51.yml) |
| &nbsp;&nbsp;&nbsp;Requires | `L4T >=32.6` |
| &nbsp;&nbsp;&nbsp;Dependencies | [`build-essential`](/packages/build-essential) [`python`](/packages/python) [`cmake`](/packages/cmake/cmake_pip) [`numpy`](/packages/numpy) [`opencv`](/packages/opencv) |
| &nbsp;&nbsp;&nbsp;Dockerfile | [`Dockerfile.ros.noetic`](Dockerfile.ros.noetic) |
| &nbsp;&nbsp;&nbsp;Images | [`dustynv/ros:noetic-ros-base-l4t-r32.4.4`](https://hub.docker.com/r/dustynv/ros/tags) `(2021-08-06, 0.5GB)`<br>[`dustynv/ros:noetic-ros-base-l4t-r32.5.0`](https://hub.docker.com/r/dustynv/ros/tags) `(2021-09-23, 0.5GB)`<br>[`dustynv/ros:noetic-ros-base-l4t-r32.6.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2022-03-02, 0.5GB)`<br>[`dustynv/ros:noetic-ros-base-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 0.6GB)`<br>[`dustynv/ros:noetic-ros-base-l4t-r34.1.0`](https://hub.docker.com/r/dustynv/ros/tags) `(2022-04-18, 5.6GB)`<br>[`dustynv/ros:noetic-ros-base-l4t-r34.1.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2022-09-23, 5.6GB)`<br>[`dustynv/ros:noetic-ros-base-l4t-r35.1.0`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-04-29, 5.6GB)`<br>[`dustynv/ros:noetic-ros-base-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-08-29, 5.2GB)`<br>[`dustynv/ros:noetic-ros-base-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 5.2GB)`<br>[`dustynv/ros:noetic-ros-base-l4t-r35.4.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-07, 5.2GB)` |

| **`ros:noetic-ros-core`** | |
| :-- | :-- |
| &nbsp;&nbsp;&nbsp;Builds | [![`ros-noetic-ros-core_jp51`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-noetic-ros-core_jp51.yml?label=ros-noetic-ros-core:jp51)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-noetic-ros-core_jp51.yml) [![`ros-noetic-ros-core_jp46`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-noetic-ros-core_jp46.yml?label=ros-noetic-ros-core:jp46)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-noetic-ros-core_jp46.yml) |
| &nbsp;&nbsp;&nbsp;Requires | `L4T >=32.6` |
| &nbsp;&nbsp;&nbsp;Dependencies | [`build-essential`](/packages/build-essential) [`python`](/packages/python) [`cmake`](/packages/cmake/cmake_pip) [`numpy`](/packages/numpy) [`opencv`](/packages/opencv) |
| &nbsp;&nbsp;&nbsp;Dockerfile | [`Dockerfile.ros.noetic`](Dockerfile.ros.noetic) |
| &nbsp;&nbsp;&nbsp;Images | [`dustynv/ros:noetic-ros-core-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 0.6GB)`<br>[`dustynv/ros:noetic-ros-core-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-09-07, 5.2GB)`<br>[`dustynv/ros:noetic-ros-core-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 5.2GB)`<br>[`dustynv/ros:noetic-ros-core-l4t-r35.4.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-07, 5.2GB)` |

| **`ros:noetic-desktop`** | |
| :-- | :-- |
| &nbsp;&nbsp;&nbsp;Builds | [![`ros-noetic-desktop_jp51`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-noetic-desktop_jp51.yml?label=ros-noetic-desktop:jp51)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-noetic-desktop_jp51.yml) [![`ros-noetic-desktop_jp46`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-noetic-desktop_jp46.yml?label=ros-noetic-desktop:jp46)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-noetic-desktop_jp46.yml) |
| &nbsp;&nbsp;&nbsp;Requires | `L4T >=32.6` |
| &nbsp;&nbsp;&nbsp;Dependencies | [`build-essential`](/packages/build-essential) [`python`](/packages/python) [`cmake`](/packages/cmake/cmake_pip) [`numpy`](/packages/numpy) [`opencv`](/packages/opencv) |
| &nbsp;&nbsp;&nbsp;Dockerfile | [`Dockerfile.ros.noetic`](Dockerfile.ros.noetic) |
| &nbsp;&nbsp;&nbsp;Images | [`dustynv/ros:noetic-desktop-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 0.6GB)`<br>[`dustynv/ros:noetic-desktop-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 5.2GB)`<br>[`dustynv/ros:noetic-desktop-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-08-29, 5.2GB)`<br>[`dustynv/ros:noetic-desktop-l4t-r35.4.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-07, 5.2GB)` |

| **`ros:foxy-ros-base`** | |
| :-- | :-- |
| &nbsp;&nbsp;&nbsp;Builds | [![`ros-foxy-ros-base_jp51`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-foxy-ros-base_jp51.yml?label=ros-foxy-ros-base:jp51)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-foxy-ros-base_jp51.yml) [![`ros-foxy-ros-base_jp46`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-foxy-ros-base_jp46.yml?label=ros-foxy-ros-base:jp46)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-foxy-ros-base_jp46.yml) |
| &nbsp;&nbsp;&nbsp;Requires | `L4T >=32.6` |
| &nbsp;&nbsp;&nbsp;Dependencies | [`build-essential`](/packages/build-essential) [`python`](/packages/python) [`cmake`](/packages/cmake/cmake_pip) [`numpy`](/packages/numpy) [`opencv`](/packages/opencv) |
| &nbsp;&nbsp;&nbsp;Dockerfile | [`Dockerfile.ros2`](Dockerfile.ros2) |
| &nbsp;&nbsp;&nbsp;Images | [`dustynv/ros:foxy-ros-base-l4t-r32.4.4`](https://hub.docker.com/r/dustynv/ros/tags) `(2021-08-06, 1.1GB)`<br>[`dustynv/ros:foxy-ros-base-l4t-r32.5.0`](https://hub.docker.com/r/dustynv/ros/tags) `(2021-09-23, 1.1GB)`<br>[`dustynv/ros:foxy-ros-base-l4t-r32.6.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2022-03-02, 1.1GB)`<br>[`dustynv/ros:foxy-ros-base-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 0.8GB)`<br>[`dustynv/ros:foxy-ros-base-l4t-r34.1.0`](https://hub.docker.com/r/dustynv/ros/tags) `(2022-04-18, 5.9GB)`<br>[`dustynv/ros:foxy-ros-base-l4t-r34.1.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2022-09-23, 5.9GB)`<br>[`dustynv/ros:foxy-ros-base-l4t-r35.1.0`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-04-29, 5.9GB)`<br>[`dustynv/ros:foxy-ros-base-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-09-07, 5.3GB)`<br>[`dustynv/ros:foxy-ros-base-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 5.3GB)`<br>[`dustynv/ros:foxy-ros-base-l4t-r35.4.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-07, 5.3GB)` |

| **`ros:foxy-ros-core`** | |
| :-- | :-- |
| &nbsp;&nbsp;&nbsp;Builds | [![`ros-foxy-ros-core_jp51`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-foxy-ros-core_jp51.yml?label=ros-foxy-ros-core:jp51)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-foxy-ros-core_jp51.yml) [![`ros-foxy-ros-core_jp46`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-foxy-ros-core_jp46.yml?label=ros-foxy-ros-core:jp46)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-foxy-ros-core_jp46.yml) |
| &nbsp;&nbsp;&nbsp;Requires | `L4T >=32.6` |
| &nbsp;&nbsp;&nbsp;Dependencies | [`build-essential`](/packages/build-essential) [`python`](/packages/python) [`cmake`](/packages/cmake/cmake_pip) [`numpy`](/packages/numpy) [`opencv`](/packages/opencv) |
| &nbsp;&nbsp;&nbsp;Dockerfile | [`Dockerfile.ros2`](Dockerfile.ros2) |
| &nbsp;&nbsp;&nbsp;Images | [`dustynv/ros:foxy-ros-core-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 0.8GB)`<br>[`dustynv/ros:foxy-ros-core-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-08-29, 5.3GB)`<br>[`dustynv/ros:foxy-ros-core-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 5.3GB)`<br>[`dustynv/ros:foxy-ros-core-l4t-r35.4.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-07, 5.3GB)` |

| **`ros:foxy-desktop`** | |
| :-- | :-- |
| &nbsp;&nbsp;&nbsp;Builds | [![`ros-foxy-desktop_jp51`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-foxy-desktop_jp51.yml?label=ros-foxy-desktop:jp51)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-foxy-desktop_jp51.yml) [![`ros-foxy-desktop_jp46`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-foxy-desktop_jp46.yml?label=ros-foxy-desktop:jp46)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-foxy-desktop_jp46.yml) |
| &nbsp;&nbsp;&nbsp;Requires | `L4T >=32.6` |
| &nbsp;&nbsp;&nbsp;Dependencies | [`build-essential`](/packages/build-essential) [`python`](/packages/python) [`cmake`](/packages/cmake/cmake_pip) [`numpy`](/packages/numpy) [`opencv`](/packages/opencv) |
| &nbsp;&nbsp;&nbsp;Dockerfile | [`Dockerfile.ros2`](Dockerfile.ros2) |
| &nbsp;&nbsp;&nbsp;Images | [`dustynv/ros:foxy-desktop-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 1.1GB)`<br>[`dustynv/ros:foxy-desktop-l4t-r34.1.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2022-09-23, 6.5GB)`<br>[`dustynv/ros:foxy-desktop-l4t-r35.1.0`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-04-29, 6.4GB)`<br>[`dustynv/ros:foxy-desktop-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-09-07, 5.9GB)`<br>[`dustynv/ros:foxy-desktop-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 5.9GB)`<br>[`dustynv/ros:foxy-desktop-l4t-r35.4.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-07, 5.9GB)` |

| **`ros:galactic-ros-base`** | |
| :-- | :-- |
| &nbsp;&nbsp;&nbsp;Builds | [![`ros-galactic-ros-base_jp46`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-galactic-ros-base_jp46.yml?label=ros-galactic-ros-base:jp46)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-galactic-ros-base_jp46.yml) [![`ros-galactic-ros-base_jp51`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-galactic-ros-base_jp51.yml?label=ros-galactic-ros-base:jp51)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-galactic-ros-base_jp51.yml) |
| &nbsp;&nbsp;&nbsp;Requires | `L4T >=32.6` |
| &nbsp;&nbsp;&nbsp;Dependencies | [`build-essential`](/packages/build-essential) [`python`](/packages/python) [`cmake`](/packages/cmake/cmake_pip) [`numpy`](/packages/numpy) [`opencv`](/packages/opencv) |
| &nbsp;&nbsp;&nbsp;Dockerfile | [`Dockerfile.ros2`](Dockerfile.ros2) |
| &nbsp;&nbsp;&nbsp;Images | [`dustynv/ros:galactic-ros-base-l4t-r32.4.4`](https://hub.docker.com/r/dustynv/ros/tags) `(2021-08-06, 0.8GB)`<br>[`dustynv/ros:galactic-ros-base-l4t-r32.5.0`](https://hub.docker.com/r/dustynv/ros/tags) `(2021-09-23, 0.8GB)`<br>[`dustynv/ros:galactic-ros-base-l4t-r32.6.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2022-03-02, 0.8GB)`<br>[`dustynv/ros:galactic-ros-base-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 0.6GB)`<br>[`dustynv/ros:galactic-ros-base-l4t-r34.1.0`](https://hub.docker.com/r/dustynv/ros/tags) `(2022-04-18, 5.6GB)`<br>[`dustynv/ros:galactic-ros-base-l4t-r34.1.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2022-09-23, 5.6GB)`<br>[`dustynv/ros:galactic-ros-base-l4t-r35.1.0`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-04-29, 5.6GB)`<br>[`dustynv/ros:galactic-ros-base-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-08-29, 5.2GB)`<br>[`dustynv/ros:galactic-ros-base-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 5.2GB)`<br>[`dustynv/ros:galactic-ros-base-l4t-r35.4.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-07, 5.2GB)` |

| **`ros:galactic-ros-core`** | |
| :-- | :-- |
| &nbsp;&nbsp;&nbsp;Builds | [![`ros-galactic-ros-core_jp51`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-galactic-ros-core_jp51.yml?label=ros-galactic-ros-core:jp51)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-galactic-ros-core_jp51.yml) [![`ros-galactic-ros-core_jp46`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-galactic-ros-core_jp46.yml?label=ros-galactic-ros-core:jp46)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-galactic-ros-core_jp46.yml) |
| &nbsp;&nbsp;&nbsp;Requires | `L4T >=32.6` |
| &nbsp;&nbsp;&nbsp;Dependencies | [`build-essential`](/packages/build-essential) [`python`](/packages/python) [`cmake`](/packages/cmake/cmake_pip) [`numpy`](/packages/numpy) [`opencv`](/packages/opencv) |
| &nbsp;&nbsp;&nbsp;Dockerfile | [`Dockerfile.ros2`](Dockerfile.ros2) |
| &nbsp;&nbsp;&nbsp;Images | [`dustynv/ros:galactic-ros-core-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 0.6GB)`<br>[`dustynv/ros:galactic-ros-core-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-08-29, 5.1GB)`<br>[`dustynv/ros:galactic-ros-core-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 5.2GB)`<br>[`dustynv/ros:galactic-ros-core-l4t-r35.4.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-07, 5.2GB)` |

| **`ros:galactic-desktop`** | |
| :-- | :-- |
| &nbsp;&nbsp;&nbsp;Builds | [![`ros-galactic-desktop_jp46`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-galactic-desktop_jp46.yml?label=ros-galactic-desktop:jp46)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-galactic-desktop_jp46.yml) [![`ros-galactic-desktop_jp51`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-galactic-desktop_jp51.yml?label=ros-galactic-desktop:jp51)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-galactic-desktop_jp51.yml) |
| &nbsp;&nbsp;&nbsp;Requires | `L4T >=32.6` |
| &nbsp;&nbsp;&nbsp;Dependencies | [`build-essential`](/packages/build-essential) [`python`](/packages/python) [`cmake`](/packages/cmake/cmake_pip) [`numpy`](/packages/numpy) [`opencv`](/packages/opencv) |
| &nbsp;&nbsp;&nbsp;Dockerfile | [`Dockerfile.ros2`](Dockerfile.ros2) |
| &nbsp;&nbsp;&nbsp;Images | [`dustynv/ros:galactic-desktop-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 1.0GB)`<br>[`dustynv/ros:galactic-desktop-l4t-r34.1.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2022-09-23, 6.2GB)`<br>[`dustynv/ros:galactic-desktop-l4t-r35.1.0`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-04-29, 6.1GB)`<br>[`dustynv/ros:galactic-desktop-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-08-29, 5.7GB)`<br>[`dustynv/ros:galactic-desktop-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 5.8GB)`<br>[`dustynv/ros:galactic-desktop-l4t-r35.4.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-07, 5.7GB)` |

| **`ros:humble-ros-base`** | |
| :-- | :-- |
| &nbsp;&nbsp;&nbsp;Builds | [![`ros-humble-ros-base_jp46`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-humble-ros-base_jp46.yml?label=ros-humble-ros-base:jp46)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-humble-ros-base_jp46.yml) [![`ros-humble-ros-base_jp51`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-humble-ros-base_jp51.yml?label=ros-humble-ros-base:jp51)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-humble-ros-base_jp51.yml) |
| &nbsp;&nbsp;&nbsp;Requires | `L4T >=32.6` |
| &nbsp;&nbsp;&nbsp;Dependencies | [`build-essential`](/packages/build-essential) [`python`](/packages/python) [`cmake`](/packages/cmake/cmake_pip) [`numpy`](/packages/numpy) [`opencv`](/packages/opencv) |
| &nbsp;&nbsp;&nbsp;Dockerfile | [`Dockerfile.ros2`](Dockerfile.ros2) |
| &nbsp;&nbsp;&nbsp;Images | [`dustynv/ros:humble-ros-base-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 0.7GB)`<br>[`dustynv/ros:humble-ros-base-l4t-r34.1.0`](https://hub.docker.com/r/dustynv/ros/tags) `(2022-05-26, 5.6GB)`<br>[`dustynv/ros:humble-ros-base-l4t-r34.1.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2022-09-23, 5.6GB)`<br>[`dustynv/ros:humble-ros-base-l4t-r35.1.0`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-04-29, 5.6GB)`<br>[`dustynv/ros:humble-ros-base-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-08-29, 5.2GB)`<br>[`dustynv/ros:humble-ros-base-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 5.2GB)`<br>[`dustynv/ros:humble-ros-base-l4t-r35.4.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-07, 5.2GB)` |

| **`ros:humble-ros-core`** | |
| :-- | :-- |
| &nbsp;&nbsp;&nbsp;Builds | [![`ros-humble-ros-core_jp46`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-humble-ros-core_jp46.yml?label=ros-humble-ros-core:jp46)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-humble-ros-core_jp46.yml) [![`ros-humble-ros-core_jp51`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-humble-ros-core_jp51.yml?label=ros-humble-ros-core:jp51)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-humble-ros-core_jp51.yml) |
| &nbsp;&nbsp;&nbsp;Requires | `L4T >=32.6` |
| &nbsp;&nbsp;&nbsp;Dependencies | [`build-essential`](/packages/build-essential) [`python`](/packages/python) [`cmake`](/packages/cmake/cmake_pip) [`numpy`](/packages/numpy) [`opencv`](/packages/opencv) |
| &nbsp;&nbsp;&nbsp;Dockerfile | [`Dockerfile.ros2`](Dockerfile.ros2) |
| &nbsp;&nbsp;&nbsp;Images | [`dustynv/ros:humble-ros-core-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 0.6GB)`<br>[`dustynv/ros:humble-ros-core-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-09-07, 5.2GB)`<br>[`dustynv/ros:humble-ros-core-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 5.2GB)`<br>[`dustynv/ros:humble-ros-core-l4t-r35.4.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-07, 5.2GB)` |

| **`ros:humble-desktop`** | |
| :-- | :-- |
| &nbsp;&nbsp;&nbsp;Builds | [![`ros-humble-desktop_jp46`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-humble-desktop_jp46.yml?label=ros-humble-desktop:jp46)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-humble-desktop_jp46.yml) [![`ros-humble-desktop_jp51`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-humble-desktop_jp51.yml?label=ros-humble-desktop:jp51)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-humble-desktop_jp51.yml) |
| &nbsp;&nbsp;&nbsp;Requires | `L4T >=32.6` |
| &nbsp;&nbsp;&nbsp;Dependencies | [`build-essential`](/packages/build-essential) [`python`](/packages/python) [`cmake`](/packages/cmake/cmake_pip) [`numpy`](/packages/numpy) [`opencv`](/packages/opencv) |
| &nbsp;&nbsp;&nbsp;Dockerfile | [`Dockerfile.ros2`](Dockerfile.ros2) |
| &nbsp;&nbsp;&nbsp;Images | [`dustynv/ros:humble-desktop-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 1.0GB)`<br>[`dustynv/ros:humble-desktop-l4t-r34.1.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2022-09-23, 6.2GB)`<br>[`dustynv/ros:humble-desktop-l4t-r35.1.0`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-04-29, 6.2GB)`<br>[`dustynv/ros:humble-desktop-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-09-07, 5.8GB)`<br>[`dustynv/ros:humble-desktop-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 5.8GB)`<br>[`dustynv/ros:humble-desktop-l4t-r35.4.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-07, 5.8GB)` |

| **`ros:iron-ros-base`** | |
| :-- | :-- |
| &nbsp;&nbsp;&nbsp;Builds | [![`ros-iron-ros-base_jp51`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-iron-ros-base_jp51.yml?label=ros-iron-ros-base:jp51)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-iron-ros-base_jp51.yml) [![`ros-iron-ros-base_jp46`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-iron-ros-base_jp46.yml?label=ros-iron-ros-base:jp46)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-iron-ros-base_jp46.yml) |
| &nbsp;&nbsp;&nbsp;Requires | `L4T >=32.6` |
| &nbsp;&nbsp;&nbsp;Dependencies | [`build-essential`](/packages/build-essential) [`python`](/packages/python) [`cmake`](/packages/cmake/cmake_pip) [`numpy`](/packages/numpy) [`opencv`](/packages/opencv) |
| &nbsp;&nbsp;&nbsp;Dockerfile | [`Dockerfile.ros2`](Dockerfile.ros2) |
| &nbsp;&nbsp;&nbsp;Images | [`dustynv/ros:iron-ros-base-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 0.7GB)`<br>[`dustynv/ros:iron-ros-base-l4t-r35.1.0`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-05-26, 5.6GB)`<br>[`dustynv/ros:iron-ros-base-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-09-07, 5.2GB)`<br>[`dustynv/ros:iron-ros-base-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 5.2GB)`<br>[`dustynv/ros:iron-ros-base-l4t-r35.4.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-07, 5.2GB)` |

| **`ros:iron-ros-core`** | |
| :-- | :-- |
| &nbsp;&nbsp;&nbsp;Builds | [![`ros-iron-ros-core_jp46`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-iron-ros-core_jp46.yml?label=ros-iron-ros-core:jp46)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-iron-ros-core_jp46.yml) [![`ros-iron-ros-core_jp51`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-iron-ros-core_jp51.yml?label=ros-iron-ros-core:jp51)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-iron-ros-core_jp51.yml) |
| &nbsp;&nbsp;&nbsp;Requires | `L4T >=32.6` |
| &nbsp;&nbsp;&nbsp;Dependencies | [`build-essential`](/packages/build-essential) [`python`](/packages/python) [`cmake`](/packages/cmake/cmake_pip) [`numpy`](/packages/numpy) [`opencv`](/packages/opencv) |
| &nbsp;&nbsp;&nbsp;Dockerfile | [`Dockerfile.ros2`](Dockerfile.ros2) |
| &nbsp;&nbsp;&nbsp;Images | [`dustynv/ros:iron-ros-core-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 0.7GB)`<br>[`dustynv/ros:iron-ros-core-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-08-29, 5.2GB)`<br>[`dustynv/ros:iron-ros-core-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 5.2GB)`<br>[`dustynv/ros:iron-ros-core-l4t-r35.4.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-07, 5.2GB)` |

| **`ros:iron-desktop`** | |
| :-- | :-- |
| &nbsp;&nbsp;&nbsp;Builds | [![`ros-iron-desktop_jp46`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-iron-desktop_jp46.yml?label=ros-iron-desktop:jp46)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-iron-desktop_jp46.yml) [![`ros-iron-desktop_jp51`](https://img.shields.io/github/actions/workflow/status/dusty-nv/jetson-containers/ros-iron-desktop_jp51.yml?label=ros-iron-desktop:jp51)](https://github.com/dusty-nv/jetson-containers/actions/workflows/ros-iron-desktop_jp51.yml) |
| &nbsp;&nbsp;&nbsp;Requires | `L4T >=32.6` |
| &nbsp;&nbsp;&nbsp;Dependencies | [`build-essential`](/packages/build-essential) [`python`](/packages/python) [`cmake`](/packages/cmake/cmake_pip) [`numpy`](/packages/numpy) [`opencv`](/packages/opencv) |
| &nbsp;&nbsp;&nbsp;Dockerfile | [`Dockerfile.ros2`](Dockerfile.ros2) |
| &nbsp;&nbsp;&nbsp;Images | [`dustynv/ros:iron-desktop-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 1.0GB)`<br>[`dustynv/ros:iron-desktop-l4t-r35.1.0`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-05-26, 6.2GB)`<br>[`dustynv/ros:iron-desktop-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-09-07, 5.8GB)`<br>[`dustynv/ros:iron-desktop-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-24, 5.8GB)`<br>[`dustynv/ros:iron-desktop-l4t-r35.4.1`](https://hub.docker.com/r/dustynv/ros/tags) `(2023-10-07, 5.8GB)` |

</details>

<details open>
<summary><b><a id="images">CONTAINER IMAGES</a></b></summary>
<br>

| Repository/Tag | Date | Arch | Size |
| :-- | :--: | :--: | :--: |
| &nbsp;&nbsp;[`dustynv/ros:eloquent-ros-base-l4t-r32.4.4`](https://hub.docker.com/r/dustynv/ros/tags) | `2021-08-06` | `arm64` | `0.8GB` |
| &nbsp;&nbsp;[`dustynv/ros:eloquent-ros-base-l4t-r32.5.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2021-09-23` | `arm64` | `0.8GB` |
| &nbsp;&nbsp;[`dustynv/ros:eloquent-ros-base-l4t-r32.6.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2022-03-02` | `arm64` | `0.8GB` |
| &nbsp;&nbsp;[`dustynv/ros:eloquent-ros-base-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-05-19` | `arm64` | `0.8GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-desktop-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `1.1GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-desktop-l4t-r34.1.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2022-09-23` | `arm64` | `6.5GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-desktop-l4t-r35.1.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-04-29` | `arm64` | `6.4GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-desktop-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-09-07` | `arm64` | `5.9GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-desktop-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `5.9GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-desktop-l4t-r35.4.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-07` | `arm64` | `5.9GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-pytorch-l4t-r32.4.4`](https://hub.docker.com/r/dustynv/ros/tags) | `2021-08-06` | `arm64` | `1.6GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-pytorch-l4t-r32.5.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2021-09-23` | `arm64` | `1.6GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-pytorch-l4t-r32.6.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2022-03-02` | `arm64` | `1.6GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-pytorch-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-05-19` | `arm64` | `1.7GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-pytorch-l4t-r34.1.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2022-04-18` | `arm64` | `6.4GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-pytorch-l4t-r34.1.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2022-09-23` | `arm64` | `6.4GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-pytorch-l4t-r35.1.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-04-29` | `arm64` | `6.6GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-pytorch-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-03-28` | `arm64` | `6.5GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-pytorch-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-05-02` | `arm64` | `6.1GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-ros-base-l4t-r32.4.4`](https://hub.docker.com/r/dustynv/ros/tags) | `2021-08-06` | `arm64` | `1.1GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-ros-base-l4t-r32.5.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2021-09-23` | `arm64` | `1.1GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-ros-base-l4t-r32.6.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2022-03-02` | `arm64` | `1.1GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-ros-base-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `0.8GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-ros-base-l4t-r34.1.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2022-04-18` | `arm64` | `5.9GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-ros-base-l4t-r34.1.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2022-09-23` | `arm64` | `5.9GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-ros-base-l4t-r35.1.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-04-29` | `arm64` | `5.9GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-ros-base-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-09-07` | `arm64` | `5.3GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-ros-base-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `5.3GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-ros-base-l4t-r35.4.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-07` | `arm64` | `5.3GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-ros-core-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `0.8GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-ros-core-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-08-29` | `arm64` | `5.3GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-ros-core-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `5.3GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-ros-core-l4t-r35.4.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-07` | `arm64` | `5.3GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-slam-l4t-r32.4.4`](https://hub.docker.com/r/dustynv/ros/tags) | `2021-08-06` | `arm64` | `2.3GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-slam-l4t-r32.5.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2021-09-23` | `arm64` | `2.3GB` |
| &nbsp;&nbsp;[`dustynv/ros:foxy-slam-l4t-r32.6.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2022-03-02` | `arm64` | `2.3GB` |
| &nbsp;&nbsp;[`dustynv/ros:galactic-desktop-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `1.0GB` |
| &nbsp;&nbsp;[`dustynv/ros:galactic-desktop-l4t-r34.1.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2022-09-23` | `arm64` | `6.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:galactic-desktop-l4t-r35.1.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-04-29` | `arm64` | `6.1GB` |
| &nbsp;&nbsp;[`dustynv/ros:galactic-desktop-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-08-29` | `arm64` | `5.7GB` |
| &nbsp;&nbsp;[`dustynv/ros:galactic-desktop-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `5.8GB` |
| &nbsp;&nbsp;[`dustynv/ros:galactic-desktop-l4t-r35.4.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-07` | `arm64` | `5.7GB` |
| &nbsp;&nbsp;[`dustynv/ros:galactic-pytorch-l4t-r32.4.4`](https://hub.docker.com/r/dustynv/ros/tags) | `2021-08-06` | `arm64` | `1.3GB` |
| &nbsp;&nbsp;[`dustynv/ros:galactic-pytorch-l4t-r32.5.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2021-09-23` | `arm64` | `1.3GB` |
| &nbsp;&nbsp;[`dustynv/ros:galactic-pytorch-l4t-r32.6.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2022-03-02` | `arm64` | `1.3GB` |
| &nbsp;&nbsp;[`dustynv/ros:galactic-pytorch-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-05-19` | `arm64` | `1.4GB` |
| &nbsp;&nbsp;[`dustynv/ros:galactic-pytorch-l4t-r34.1.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2022-04-18` | `arm64` | `6.1GB` |
| &nbsp;&nbsp;[`dustynv/ros:galactic-pytorch-l4t-r34.1.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2022-09-23` | `arm64` | `6.1GB` |
| &nbsp;&nbsp;[`dustynv/ros:galactic-pytorch-l4t-r35.1.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-04-29` | `arm64` | `6.3GB` |
| &nbsp;&nbsp;[`dustynv/ros:galactic-pytorch-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-03-28` | `arm64` | `6.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:galactic-pytorch-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-05-02` | `arm64` | `5.8GB` |
| &nbsp;&nbsp;[`dustynv/ros:galactic-ros-base-l4t-r32.4.4`](https://hub.docker.com/r/dustynv/ros/tags) | `2021-08-06` | `arm64` | `0.8GB` |
| &nbsp;&nbsp;[`dustynv/ros:galactic-ros-base-l4t-r32.5.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2021-09-23` | `arm64` | `0.8GB` |
| &nbsp;&nbsp;[`dustynv/ros:galactic-ros-base-l4t-r32.6.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2022-03-02` | `arm64` | `0.8GB` |
| &nbsp;&nbsp;[`dustynv/ros:galactic-ros-base-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `0.6GB` |
| &nbsp;&nbsp;[`dustynv/ros:galactic-ros-base-l4t-r34.1.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2022-04-18` | `arm64` | `5.6GB` |
| &nbsp;&nbsp;[`dustynv/ros:galactic-ros-base-l4t-r34.1.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2022-09-23` | `arm64` | `5.6GB` |
| &nbsp;&nbsp;[`dustynv/ros:galactic-ros-base-l4t-r35.1.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-04-29` | `arm64` | `5.6GB` |
| &nbsp;&nbsp;[`dustynv/ros:galactic-ros-base-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-08-29` | `arm64` | `5.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:galactic-ros-base-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `5.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:galactic-ros-base-l4t-r35.4.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-07` | `arm64` | `5.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:galactic-ros-core-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `0.6GB` |
| &nbsp;&nbsp;[`dustynv/ros:galactic-ros-core-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-08-29` | `arm64` | `5.1GB` |
| &nbsp;&nbsp;[`dustynv/ros:galactic-ros-core-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `5.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:galactic-ros-core-l4t-r35.4.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-07` | `arm64` | `5.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:humble-desktop-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `1.0GB` |
| &nbsp;&nbsp;[`dustynv/ros:humble-desktop-l4t-r34.1.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2022-09-23` | `arm64` | `6.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:humble-desktop-l4t-r35.1.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-04-29` | `arm64` | `6.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:humble-desktop-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-09-07` | `arm64` | `5.8GB` |
| &nbsp;&nbsp;[`dustynv/ros:humble-desktop-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `5.8GB` |
| &nbsp;&nbsp;[`dustynv/ros:humble-desktop-l4t-r35.4.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-07` | `arm64` | `5.8GB` |
| &nbsp;&nbsp;[`dustynv/ros:humble-pytorch-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-05-19` | `arm64` | `1.4GB` |
| &nbsp;&nbsp;[`dustynv/ros:humble-pytorch-l4t-r34.1.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2022-05-26` | `arm64` | `6.1GB` |
| &nbsp;&nbsp;[`dustynv/ros:humble-pytorch-l4t-r34.1.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2022-09-23` | `arm64` | `6.1GB` |
| &nbsp;&nbsp;[`dustynv/ros:humble-pytorch-l4t-r35.1.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-04-29` | `arm64` | `6.3GB` |
| &nbsp;&nbsp;[`dustynv/ros:humble-pytorch-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-03-28` | `arm64` | `6.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:humble-pytorch-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-05-02` | `arm64` | `5.8GB` |
| &nbsp;&nbsp;[`dustynv/ros:humble-ros-base-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `0.7GB` |
| &nbsp;&nbsp;[`dustynv/ros:humble-ros-base-l4t-r34.1.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2022-05-26` | `arm64` | `5.6GB` |
| &nbsp;&nbsp;[`dustynv/ros:humble-ros-base-l4t-r34.1.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2022-09-23` | `arm64` | `5.6GB` |
| &nbsp;&nbsp;[`dustynv/ros:humble-ros-base-l4t-r35.1.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-04-29` | `arm64` | `5.6GB` |
| &nbsp;&nbsp;[`dustynv/ros:humble-ros-base-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-08-29` | `arm64` | `5.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:humble-ros-base-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `5.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:humble-ros-base-l4t-r35.4.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-07` | `arm64` | `5.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:humble-ros-core-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `0.6GB` |
| &nbsp;&nbsp;[`dustynv/ros:humble-ros-core-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-09-07` | `arm64` | `5.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:humble-ros-core-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `5.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:humble-ros-core-l4t-r35.4.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-07` | `arm64` | `5.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:iron-desktop-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `1.0GB` |
| &nbsp;&nbsp;[`dustynv/ros:iron-desktop-l4t-r35.1.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-05-26` | `arm64` | `6.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:iron-desktop-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-09-07` | `arm64` | `5.8GB` |
| &nbsp;&nbsp;[`dustynv/ros:iron-desktop-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `5.8GB` |
| &nbsp;&nbsp;[`dustynv/ros:iron-desktop-l4t-r35.4.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-07` | `arm64` | `5.8GB` |
| &nbsp;&nbsp;[`dustynv/ros:iron-pytorch-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-05-26` | `arm64` | `1.4GB` |
| &nbsp;&nbsp;[`dustynv/ros:iron-pytorch-l4t-r35.1.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-05-26` | `arm64` | `6.3GB` |
| &nbsp;&nbsp;[`dustynv/ros:iron-pytorch-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-05-25` | `arm64` | `6.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:iron-pytorch-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-05-25` | `arm64` | `5.8GB` |
| &nbsp;&nbsp;[`dustynv/ros:iron-ros-base-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `0.7GB` |
| &nbsp;&nbsp;[`dustynv/ros:iron-ros-base-l4t-r35.1.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-05-26` | `arm64` | `5.6GB` |
| &nbsp;&nbsp;[`dustynv/ros:iron-ros-base-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-09-07` | `arm64` | `5.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:iron-ros-base-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `5.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:iron-ros-base-l4t-r35.4.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-07` | `arm64` | `5.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:iron-ros-core-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `0.7GB` |
| &nbsp;&nbsp;[`dustynv/ros:iron-ros-core-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-08-29` | `arm64` | `5.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:iron-ros-core-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `5.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:iron-ros-core-l4t-r35.4.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-07` | `arm64` | `5.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:melodic-desktop-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `0.7GB` |
| &nbsp;&nbsp;[`dustynv/ros:melodic-ros-base-l4t-r32.4.4`](https://hub.docker.com/r/dustynv/ros/tags) | `2021-08-06` | `arm64` | `0.5GB` |
| &nbsp;&nbsp;[`dustynv/ros:melodic-ros-base-l4t-r32.5.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2021-09-23` | `arm64` | `0.5GB` |
| &nbsp;&nbsp;[`dustynv/ros:melodic-ros-base-l4t-r32.6.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2022-03-02` | `arm64` | `0.5GB` |
| &nbsp;&nbsp;[`dustynv/ros:melodic-ros-base-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `0.5GB` |
| &nbsp;&nbsp;[`dustynv/ros:melodic-ros-core-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `0.5GB` |
| &nbsp;&nbsp;[`dustynv/ros:noetic-desktop-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `0.6GB` |
| &nbsp;&nbsp;[`dustynv/ros:noetic-desktop-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `5.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:noetic-desktop-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-08-29` | `arm64` | `5.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:noetic-desktop-l4t-r35.4.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-07` | `arm64` | `5.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:noetic-pytorch-l4t-r32.4.4`](https://hub.docker.com/r/dustynv/ros/tags) | `2021-08-06` | `arm64` | `1.1GB` |
| &nbsp;&nbsp;[`dustynv/ros:noetic-pytorch-l4t-r32.5.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2021-09-23` | `arm64` | `1.1GB` |
| &nbsp;&nbsp;[`dustynv/ros:noetic-pytorch-l4t-r32.6.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2022-03-02` | `arm64` | `1.0GB` |
| &nbsp;&nbsp;[`dustynv/ros:noetic-pytorch-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-05-19` | `arm64` | `1.3GB` |
| &nbsp;&nbsp;[`dustynv/ros:noetic-pytorch-l4t-r34.1.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2022-04-18` | `arm64` | `6.1GB` |
| &nbsp;&nbsp;[`dustynv/ros:noetic-pytorch-l4t-r34.1.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2022-09-23` | `arm64` | `6.0GB` |
| &nbsp;&nbsp;[`dustynv/ros:noetic-pytorch-l4t-r35.1.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-04-29` | `arm64` | `6.3GB` |
| &nbsp;&nbsp;[`dustynv/ros:noetic-pytorch-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-03-28` | `arm64` | `6.1GB` |
| &nbsp;&nbsp;[`dustynv/ros:noetic-pytorch-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-05-02` | `arm64` | `5.8GB` |
| &nbsp;&nbsp;[`dustynv/ros:noetic-ros-base-l4t-r32.4.4`](https://hub.docker.com/r/dustynv/ros/tags) | `2021-08-06` | `arm64` | `0.5GB` |
| &nbsp;&nbsp;[`dustynv/ros:noetic-ros-base-l4t-r32.5.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2021-09-23` | `arm64` | `0.5GB` |
| &nbsp;&nbsp;[`dustynv/ros:noetic-ros-base-l4t-r32.6.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2022-03-02` | `arm64` | `0.5GB` |
| &nbsp;&nbsp;[`dustynv/ros:noetic-ros-base-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `0.6GB` |
| &nbsp;&nbsp;[`dustynv/ros:noetic-ros-base-l4t-r34.1.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2022-04-18` | `arm64` | `5.6GB` |
| &nbsp;&nbsp;[`dustynv/ros:noetic-ros-base-l4t-r34.1.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2022-09-23` | `arm64` | `5.6GB` |
| &nbsp;&nbsp;[`dustynv/ros:noetic-ros-base-l4t-r35.1.0`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-04-29` | `arm64` | `5.6GB` |
| &nbsp;&nbsp;[`dustynv/ros:noetic-ros-base-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-08-29` | `arm64` | `5.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:noetic-ros-base-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `5.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:noetic-ros-base-l4t-r35.4.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-07` | `arm64` | `5.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:noetic-ros-core-l4t-r32.7.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `0.6GB` |
| &nbsp;&nbsp;[`dustynv/ros:noetic-ros-core-l4t-r35.2.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-09-07` | `arm64` | `5.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:noetic-ros-core-l4t-r35.3.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-24` | `arm64` | `5.2GB` |
| &nbsp;&nbsp;[`dustynv/ros:noetic-ros-core-l4t-r35.4.1`](https://hub.docker.com/r/dustynv/ros/tags) | `2023-10-07` | `arm64` | `5.2GB` |

> <sub>Container images are compatible with other minor versions of JetPack/L4T:</sub><br>
> <sub>&nbsp;&nbsp;&nbsp;&nbsp;• L4T R32.7 containers can run on other versions of L4T R32.7 (JetPack 4.6+)</sub><br>
> <sub>&nbsp;&nbsp;&nbsp;&nbsp;• L4T R35.x containers can run on other versions of L4T R35.x (JetPack 5.1+)</sub><br>
</details>

<details open>
<summary><b><a id="run">RUN CONTAINER</a></b></summary>
<br>

To start the container, you can use the [`run.sh`](/docs/run.md)/[`autotag`](/docs/run.md#autotag) helpers or manually put together a [`docker run`](https://docs.docker.com/engine/reference/commandline/run/) command:
```bash
# automatically pull or build a compatible container image
./run.sh $(./autotag ros)

# or explicitly specify one of the container images above
./run.sh dustynv/ros:foxy-ros-base-l4t-r32.7.1

# or if using 'docker run' (specify image and mounts/ect)
sudo docker run --runtime nvidia -it --rm --network=host dustynv/ros:foxy-ros-base-l4t-r32.7.1
```
> <sup>[`run.sh`](/docs/run.md) forwards arguments to [`docker run`](https://docs.docker.com/engine/reference/commandline/run/) with some defaults added (like `--runtime nvidia`, mounts a `/data` cache, and detects devices)</sup><br>
> <sup>[`autotag`](/docs/run.md#autotag) finds a container image that's compatible with your version of JetPack/L4T - either locally, pulled from a registry, or by building it.</sup>

To mount your own directories into the container, use the [`-v`](https://docs.docker.com/engine/reference/commandline/run/#volume) or [`--volume`](https://docs.docker.com/engine/reference/commandline/run/#volume) flags:
```bash
./run.sh -v /path/on/host:/path/in/container $(./autotag ros)
```
To launch the container running a command, as opposed to an interactive shell:
```bash
./run.sh $(./autotag ros) my_app --abc xyz
```
You can pass any options to [`run.sh`](/docs/run.md) that you would to [`docker run`](https://docs.docker.com/engine/reference/commandline/run/), and it'll print out the full command that it constructs before executing it.
</details>
<details open>
<summary><b><a id="build">BUILD CONTAINER</b></summary>
<br>

If you use [`autotag`](/docs/run.md#autotag) as shown above, it'll ask to build the container for you if needed.  To manually build it, first do the [system setup](/docs/setup.md), then run:
```bash
./build.sh ros
```
The dependencies from above will be built into the container, and it'll be tested during.  See [`./build.sh --help`](/jetson_containers/build.py) for build options.
</details>
