FROM ros:humble-ros-core-jammy

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    sudo \
    curl \
    wget \
    vim \
    git \
    python3-pip \
    python3-vcstool \
    python3-rosdep \
    xvfb \
    htop \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libglx-mesa0 \
    ros-dev-tools \
    xdg-utils \
    software-properties-common \
    apt-transport-https \
    gpg \
    openjdk-11-jre \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init 

## Install TypeDB
RUN gpg --keyserver hkp://keyserver.ubuntu.com:80 --recv-key 17507562824cfdcc
RUN gpg --export 17507562824cfdcc | sudo tee /etc/apt/trusted.gpg.d/vaticle.gpg > /dev/null
RUN echo "deb https://repo.typedb.com/public/public-release/deb/ubuntu trusty main" | tee /etc/apt/sources.list.d/vaticle.list > /dev/null

RUN apt-get update && apt-get install -y \
    typedb=2.28.3 \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install typedb-driver==2.28.0

## Install TypeDB studio
RUN sudo mkdir /usr/share/desktop-directories/
RUN wget https://repo.typedb.com/public/public-release/raw/names/typedb-studio-linux-x86_64-deb/versions/2.28.0/typedb-studio-linux-x86_64-2.28.0.deb
RUN dpkg -i typedb-studio-linux-x86_64-2.28.0.deb

## Create ubuntu-user user, disable password, and add it to sudo group
RUN groupadd -g 1000 ubuntu-user \
    && adduser --disabled-password --gid 1000 --uid 1000 --gecos '' ubuntu-user \
    && adduser ubuntu-user sudo

RUN echo 'ubuntu-user ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers
ENV HOME=/home/ubuntu-user
USER ubuntu-user
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR $HOME

## Create alias for TypeDB studio
RUN echo 'alias typedb-studio="/opt/typedb-studio/bin/TypeDB\ Studio"' >> ~/.bashrc

## Create, install deps, and build ros_typedb workspace
RUN mkdir -p typedb_ws/src
WORKDIR $HOME/typedb_ws/src
RUN git clone https://github.com/kas-lab/ros_typedb.git
WORKDIR $HOME/typedb_ws/

RUN ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash \
    && sudo apt update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && sudo rm -rf /var/lib/apt/lists/"]

## There is a bug with the default setuptools and packaging versions
RUN USER=ubuntu-user python3 -m pip install setuptools==75.8.2  packaging==24.1 empy==3.3.4 pandas==2.0.2 scipy==1.15.2 numpy==1.26.4
RUN ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install \
    && echo 'source ~/typedb_ws/install/setup.bash' >> ~/.bashrc"]

RUN sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/