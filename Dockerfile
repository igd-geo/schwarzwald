FROM gcc:7.5 AS build

RUN apt-get update && apt-get -y install cmake

RUN mkdir /data
WORKDIR /data

RUN git clone https://github.com/m-schuetz/LAStools.git
WORKDIR /data/LAStools/LASzip
RUN mkdir build
RUN cd build && cmake -DCMAKE_BUILD_TYPE=Release ..
RUN cd build && make

WORKDIR /data
RUN mkdir Schwarzwald
WORKDIR /data/Schwarzwald
ADD . /data/Schwarzwald
RUN mkdir build
RUN cd build && cmake -DCMAKE_BUILD_TYPE=Release -DLASZIP_INCLUDE_DIRS=/data/LAStools/LASzip/dll -DLASZIP_LIBRARY=/data/LAStools/LASzip/build/src/liblaszip.so .. 
RUN cd build && make

# copy libproj.so dependency to a temporary directory
RUN ldd /data/Schwarzwald/build/Release/Schwarzwald | grep 'libproj.so' | awk '{print $3}' | xargs -I '{}' cp -v '{}' /tmp/

# after building, create smaller image that only contains the binary
# and its dependencies
FROM debian:buster

# copy dependencies
COPY --from=build /root/.hunter/_Base/0b8c31b/8d6d629/adeda0f/Install/lib/ /root/.hunter/_Base/0b8c31b/8d6d629/adeda0f/Install/lib/
COPY --from=build /data/LAStools/LASzip/build/src/liblaszip.so /usr/lib/liblaszip.so
COPY --from=build /tmp/libproj.so* /usr/lib/

# copy binary
COPY --from=build /data/Schwarzwald/build/Release/Schwarzwald /pointcloud-tiler/Schwarzwald/build/Release/Schwarzwald
COPY --from=build /data/Schwarzwald/build/Release/LASBenchmark /pointcloud-tiler/Schwarzwald/build/Release/LASBenchmark

ENTRYPOINT ["/pointcloud-tiler/Schwarzwald/build/Release/Schwarzwald"]
