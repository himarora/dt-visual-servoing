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

WORKDIR /workspace

# here, we install the requirements, some requirements come by default
# you can add more if you need to in requirements.txt
COPY requirements.txt .
RUN pip install -r requirements.txt

# Need to upgrade Pillow for Old ROS stack
RUN pip install pillow --user --upgrade

# let's copy all our solution files to our workspace
# if you have more file use the COPY command to move them to the workspace
COPY solution.py ./

# For ROS Agent - Additional Files
COPY rosagent.py ./
COPY lf_slim.launch ./

# Source it to add messages to path
RUN echo "source /home/software/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN echo "source /opt/ros/kinetic/devel/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "export PYTHONPATH="/usr/local/lib/python2.7/dist-packages:$PYTHONPATH""

# DO NOT MODIFY: your submission won't run if you do
ENV DUCKIETOWN_SERVER=evaluator

# For ROS Agent - pulls the default configuration files
ENV HOSTNAME=default

RUN ["cross-build-end"]
ENTRYPOINT ["qemu3-arm-static"]

# let's see what you've got there...
# CMD ["/bin/bash", "-c", "source /home/software/docker/env.sh && . /home/software/set_vehicle_name.sh && ./solution2.py"]

CMD ["/bin/bash", "-ci", "./solution.py"]