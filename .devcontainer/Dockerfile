FROM yyss2028/legged_state_estimation:gtsam

# add libtorch
# https://github.com/pytorch/pytorch/issues/17492#issuecomment-524692441
# need to use a version with flag _GLIBCXX_USE_CXX11_ABI = 1
# otherwise it will break ROS installation 
WORKDIR $SUPPORT_WS
RUN wget https://download.pytorch.org/libtorch/nightly/cpu/libtorch-cxx11-abi-shared-with-deps-latest.zip
RUN sudo apt-get install -y unzip gnuplot
RUN unzip libtorch-cxx11-abi-shared-with-deps-latest.zip


RUN sudo apt-get install -y ros-melodic-plotjuggler-ros