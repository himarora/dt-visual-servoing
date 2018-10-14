# Definition of Submission container

# We start from a base ROS image
FROM duckietown/rpi-duckiebot-base:master18

RUN ["cross-build-start"]

# DO NOT MODIFY: your submission won't run if you do
RUN apt-get update -y && apt-get install -y --no-install-recommends \
         gcc \
         libc-dev\
         git \
         bzip2 \
         python-tk \
         python-wheel \
         python-pip && \
     rm -rf /var/lib/apt/lists/*

# let's create our workspace, we don't want to clutter the container
RUN mkdir /workspace

# here, we install the requirements, some requirements come by default
# you can add more if you need to in requirements.txt
COPY requirements.txt /workspace
RUN pip install -r /workspace/requirements.txt

# let's copy all our solution files to our workspace
# if you have more file use the COPY command to move them to the workspace
COPY solution.py /workspace

# For ROS Agent - Additional Files
COPY rosagent.py /workspace
COPY lf_slim.launch /workspace
COPY catkin_ws /workspace/catkin_ws

# we make the workspace our working directory
WORKDIR /workspace

# Build the catkin_ws
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_make -j -C catkin_ws/"

# Source it to add messages to path
RUN /bin/bash -c "echo source catkin_ws/devel/setup.bash >> $HOME/.bashrc"

# DO NOT MODIFY: your submission won't run if you do
ENV DUCKIETOWN_SERVER=evaluator

# For ROS Agent - pulls the default configuration files
ENV HOSTNAME=default

COPY test.py /workspace

RUN ["cross-build-end"]


# let's see what you've got there...
CMD python solution.py
