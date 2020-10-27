ARG TAG="task-dispatcher"
# FROM osrf/rmf_core:$TAG
FROM tanyouliang95/rmf_core:$TAG

ENV HOME /home/ws_rmf/

RUN apt-get update && apt-get install -y \
    libbson-1.0-0 libbson-dev \
    libmongoc-1.0-0 libmongoc-dev \
    libicu-dev \
    curl

WORKDIR  /home/
RUN curl -OL https://github.com/mongodb/mongo-cxx-driver/archive/r3.4.2.tar.gz
RUN ls && tar -xzf r3.4.2.tar.gz
RUN cd /home/mongo-cxx-driver-r3.4.2/build && \
    cmake ..                                \
    -DCMAKE_BUILD_TYPE=Release          \
    -DCMAKE_INSTALL_PREFIX=/usr/local &&\
    make && make install

RUN rm /usr/local/lib/lib*.so._noabi
RUN ln -s /usr/local/lib/libmongocxx.so.3.4.2 /usr/local/lib/libmongocxx.so._noabi
RUN ln -s /usr/local/lib/libbsoncxx.so.3.4.2 /usr/local/lib/libbsoncxx.so._noabi
RUN ldconfig
RUN rm r3.4.2.tar.gz
RUN rm -rf mongo-c-driver-3.4.2

WORKDIR  /home/ws_rmf/
COPY . src
RUN /ros_entrypoint.sh \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=RELEASE && \
    rm -rf build devel src

# todo: should have a multistage build

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
