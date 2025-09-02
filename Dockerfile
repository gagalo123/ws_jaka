# CUDA 12.8.1 runtime + cuDNN + Ubuntu 22.04
FROM nvidia/cuda:12.8.1-cudnn-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# --- 国内源 & 基本系统依赖 & OpenGL/GUI libs ---
RUN echo "deb https://mirrors.aliyun.com/ubuntu/ jammy main restricted universe multiverse" > /etc/apt/sources.list \
 && echo "deb https://mirrors.aliyun.com/ubuntu/ jammy-security main restricted universe multiverse" >> /etc/apt/sources.list \
 && echo "deb https://mirrors.aliyun.com/ubuntu/ jammy-updates main restricted universe multiverse" >> /etc/apt/sources.list \
 && echo "deb https://mirrors.aliyun.com/ubuntu/ jammy-backports main restricted universe multiverse" >> /etc/apt/sources.list
RUN apt-get update 

RUN apt install -y locales sudo software-properties-common curl neovim

# 设置语言环境
RUN locale-gen en_US.UTF-8 \
 && update-locale LANG=en_US.UTF-8 \
 && export LANG=en_US.UTF-8

# --- 安装 ROS 2 Humble (Debian 包) 使用国内源加速 ---
RUN mkdir -p /usr/share/keyrings \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-argcomplete \
 && rm -rf /var/lib/apt/lists/*

# 初始化 rosdep
RUN rosdep init || true
RUN rosdep update || true

# --- 安装 Miniconda 并创建 conda 环境 ---
WORKDIR /opt
RUN curl -fsSL https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -o /tmp/miniconda.sh \
 && bash /tmp/miniconda.sh -b -p /opt/conda \
 && rm /tmp/miniconda.sh
ENV PATH=/opt/conda/bin:$PATH

# 设置 channels 和 always_yes
# 写入 conda 配置文件，使用国内镜像和自定义 channel
RUN mkdir -p /opt/conda/.condarc.d \
 && echo "channels:" > /opt/conda/.condarc \
 && echo "  - defaults" >> /opt/conda/.condarc \
 && echo "show_channel_urls: true" >> /opt/conda/.condarc \
 && echo "default_channels:" >> /opt/conda/.condarc \
 && echo "  - https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main" >> /opt/conda/.condarc \
 && echo "  - https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/r" >> /opt/conda/.condarc \
 && echo "  - https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/msys2" >> /opt/conda/.condarc \
 && echo "custom_channels:" >> /opt/conda/.condarc \
 && echo "  conda-forge: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud" >> /opt/conda/.condarc \
 && echo "  pytorch: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud" >> /opt/conda/.condarc \
 && /opt/conda/bin/conda config --set always_yes yes \
 && /opt/conda/bin/conda config --set changeps1 no \
 && conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/free/ \
 && conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main/ \
 && conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/conda-forge 
#  && conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/r \
#  && conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/msys2 \
#  && conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud \

# 创建 conda 环境 humble（Python 3.10.13），跳过交互式 TOS
RUN conda create -y -n humble python=3.10.13 \
   && conda run -n humble conda install -y pinocchio casadi
RUN conda run -n humble pip config set global.index-url https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple \
   && conda run -n humble pip install "fastapi[standard]" \
   && conda run -n humble pip install meshcat catkin_pkg empy==3.3.4 lark-parser matplotlib


RUN sudo apt update \
   && apt install -y ros-humble-moveit lsb-release gnupg git ros-humble-moveit-visual-tools

RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
   && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
   http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
   && sudo apt-get update \
   && sudo apt-get -y install ignition-fortress ros-humble-ros-gz python-is-python3 python3-pip  ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-ign-ros2-control ros-humble-ign-ros2-control-demos
RUN pip install yapf



# # 切回 root shell
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# # --- ROS2 workspace & 复制 robot_tool 源码 ---
# WORKDIR /home/jaka_ws
# COPY . .

# # WORKDIR /root/ros2_ws

# # 安装 ROS package 依赖
# RUN /bin/bash -lc "source /opt/ros/humble/setup.bash && rosdep install --from-paths src -i -y || true"

# # build
# RUN /bin/bash -lc "source /opt/ros/humble/setup.bash && colcon build --parallel-workers 1 --event-handlers console_direct+"

# # --- 容器启动环境配置（自动 source ROS，激活 conda 环境） ---
# 初始化 conda，但默认不激活 base
RUN echo "# >>> conda initialize >>>" >> /root/.bashrc \
 && echo "__conda_setup=\"\$('/opt/conda/bin/conda' 'shell.bash' 'hook' 2> /dev/null)\"" >> /root/.bashrc \
 && echo "if [ \$? -eq 0 ]; then" >> /root/.bashrc \
 && echo "    eval \"\$__conda_setup\"" >> /root/.bashrc \
 && echo "else" >> /root/.bashrc \
 && echo "    if [ -f \"/opt/conda/etc/profile.d/conda.sh\" ]; then" >> /root/.bashrc \
 && echo "        . \"/opt/conda/etc/profile.d/conda.sh\"" >> /root/.bashrc \
 && echo "    else" >> /root/.bashrc \
 && echo "        export PATH=\"/opt/conda/bin:\$PATH\"" >> /root/.bashrc \
 && echo "    fi" >> /root/.bashrc \
 && echo "fi" >> /root/.bashrc \
 && echo "unset __conda_setup" >> /root/.bashrc \
 && echo "# <<< conda initialize <<<" >> /root/.bashrc \
 && echo "conda deactivate" >> /root/.bashrc


RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
 && echo "export PATH=/usr/bin:$PATH" >> /root/.bashrc

CMD ["/bin/bash", "-l"]
