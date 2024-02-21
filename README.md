# PROS AI Image

The folder `ros` refers to the [GitHub](https://github.com/dusty-nv/jetson-containers/tree/master/packages/ros) repository.

This project aims to build an image containing `pytorch` and `ros2` for the Jetson Orin Nano Developer Kit (arm64) and x86_64 platform PC (amd64).

**Warning**: The function `torch.cuda.is_available()` returns False in Jetson Orin.

This project contains **2 submodules**, please add the git clone flag `--recursive`.



## Colcon

We've written a run command `rebuild_colcon.rc` in `/workspaces` folder. You can do `colcon build` and `source /workspaces/install/setup.bash` by the following command:

```bash
source /workspaces/rebuild_colcon.rc
```



## Source code

- This image has removed the source code of

  - `csm`
  - `ros2_laser_scan_matcher`
  - `rplidar_ros`, which is a submodule from https://github.com/Slamtec/rplidar_ros.
  - `slam_toolbox`, which is a submodule from https://github.com/SteveMacenski/slam_toolbox.
  - `pros_image`
  - `ros2_astra_camera`
  - `ros2_astra_camera_msgs`



## Manually build the image

### Environments Setup

1. To use buildx, make sure your Docker runtime is at least version 19.03. buildx actually comes bundled with Docker by default, but needs to be enabled by setting the environment variable DOCKER_CLI_EXPERIMENTAL.

    ```bash
    export DOCKER_CLI_EXPERIMENTAL=enabled
    ```

2. If you're on Linux, you need to set up `binfmt_misc`. This is pretty easy in most distributions but is even easier now that you can just run a privileged Docker container to set it up for you.

   ```bash
   docker run --rm --privileged tonistiigi/binfmt:latest
   ```

   or

    ```bash
    docker run --rm --privileged docker/binfmt:latest
    ```

3. Create a new builder which gives access to the new multi-architecture features. This command creates a new builder instance. In this case, it supports both linux/arm64 and linux/amd64 platforms. The --name flag sets a name for the builder- "multi-platform-builder".

    ```bash
    docker buildx create --use --platform=linux/arm64,linux/amd64 --name multi-platform-builder
    ```

4. This command inspects the builder created in the previous step and performs any necessary setup or configuration. The --bootstrap flag indicates that the builder should be initialized if it hasn't been already

    ```bash
    docker buildx inspect --bootstrap
    ```

5. This command builds a Docker image using the builder created earlier.

    ```bash
    docker buildx build --platform=linux/arm64,linux/amd64 --push --tag ghcr.io/otischung/pros_ai_image:latest -f ./Dockerfile .
    ```


Reference: https://stackoverflow.com/questions/70757791/build-linux-arm64-docker-image-on-linux-amd64-host

Reference: https://unix.stackexchange.com/questions/748633/error-multiple-platforms-feature-is-currently-not-supported-for-docker-driver



### Troubleshooting

If you encounter that you can't build Dockerfile for arm64 due to `libc-bin` segmentation fault, try solve by the following instrucitons.

```bash
docker pull tonistiigi/binfmt:latest
docker run --privileged --rm tonistiigi/binfmt --uninstall qemu-*
docker run --privileged --rm tonistiigi/binfmt --install all)
```

Reference: https://askubuntu.com/questions/1339558/cant-build-dockerfile-for-arm64-due-to-libc-bin-segmentation-fault



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


