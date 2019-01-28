FROM ubuntu:16.04
FROM ubuntu:16.04

RUN apt-get update -qqy && apt-get install -qqy \
   curl \
   && rm -rf /var/lib/apt/lists/*

RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub xenial robotpkg" > /etc/apt/sources.list.d/robotpkg.list
RUN curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | apt-key add -

RUN apt-get update -qqy && apt-get upgrade -qqy
RUN apt-get install -qqy sudo wget
RUN apt-get install -qqy \
      autoconf \
      g++ \
      cmake \
      doxygen \
      libboost-dev \
      liburdfdom-dev \
      libassimp-dev \
      robotpkg-qpoases+doc \
      robotpkg-roboptim-core \
      robotpkg-roboptim-trajectory \
      ros-kinetic-xacro \
      ros-kinetic-kdl-parser \
      ros-kinetic-common-msgs \
      ros-kinetic-tf \
      ros-kinetic-tf-conversions \
      libccd-dev \
      ros-kinetic-octomap \
      ros-kinetic-resource-retriever \
      ros-kinetic-srdfdom \
      ros-kinetic-pr2-description \
      flex \
      bison \
      asciidoc \
      source-highlight \
      git \
      libomniorb4-dev \
      omniorb-nameserver \
      omniidl \
      omniidl-python \
      libltdl-dev \
      python-matplotlib \
      libxml2-dev \
      libtinyxml2-dev \
      liblog4cxx10-dev \
      libltdl-dev \
      qt4-dev-tools \
      libqt4-opengl-dev \
      libqtwebkit-dev \
      libqtgui4 \
      oxygen-icon-theme \
      libopenscenegraph-dev \
      openscenegraph \
      libpcre3-dev \
      libcdd-dev \
      nano \
      cmake-curses-gui \
      && apt-get remove -qqy texlive-latex-base texlive-binaries ghostscript \
      && apt-get autoremove -qqy \
      && rm -rf /var/lib/apt/lists/*

COPY auto-install-hpp.sh /
RUN bash auto-install-hpp.sh
RUN git config --global alias.plog "log --graph --abbrev-commit --decorate --date=relative --format=format:'%C(bold blue)%h%C(reset) - %C(bold green)(%ar)%C(reset) %C(white)%s%C(reset) %C(dim white)- %an%C(reset)%C(bold yellow)%d%C(reset)' --all"
COPY auto-install-locomote.sh /
RUN bash auto-install-locomote.sh 
COPY starthpp
RUN sudo mv /starthpp /usr/bin/.
RUN sudo chmod 755 /usr/bin/starthpp
