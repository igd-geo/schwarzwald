FROM gcc:14.2 AS build

RUN apt-get update && apt-get -y install cmake sqlite3 libboost-system-dev libboost-iostreams-dev libboost-program-options-dev

RUN mkdir /data
WORKDIR /data

RUN git clone https://github.com/m-schuetz/LAStools.git
WORKDIR /data/LAStools/LASzip
RUN mkdir build
RUN cd build && cmake -DCMAKE_BUILD_TYPE=Release ..
RUN cd build && make -j`nproc`

WORKDIR /data
RUN mkdir Schwarzwald
WORKDIR /data/Schwarzwald
ADD . /data/Schwarzwald
RUN rm -rf build && mkdir build
RUN cd build && cmake -DCMAKE_BUILD_TYPE=Release -DLASZIP_INCLUDE_DIRS=/data/LAStools/LASzip/dll -DLASZIP_LIBRARY=/data/LAStools/LASzip/build/src/liblaszip.so ..
RUN cd build && make -j`nproc`

# copy libproj.so dependency to a temporary directory
RUN ldd /data/Schwarzwald/build/Release/Schwarzwald | grep 'libproj.so' | awk '{print $3}' | xargs -I '{}' cp -v '{}' /tmp/

# after building, create smaller image that only contains the binary
# and its dependencies
FROM debian:bookworm

RUN apt-get update && apt-get -y install sqlite3 curl

# copy dependencies
COPY --from=build /data/LAStools/LASzip/build/src/liblaszip.so /usr/lib/liblaszip.so
COPY --from=build /tmp/libproj.so* /usr/lib/
COPY --from=build /data/Schwarzwald/build/_deps/proj-build/data/proj.db /proj.db
COPY --from=build /usr/local/lib64/libstdc++.so* /usr/local/lib64/
ENV LD_LIBRARY_PATH=/usr/local/lib64

# copy binary
COPY --from=build /data/Schwarzwald/build/Release/Schwarzwald /pointcloud-tiler/Schwarzwald/build/Release/Schwarzwald
COPY --from=build /data/Schwarzwald/build/Release/LASBenchmark /pointcloud-tiler/Schwarzwald/build/Release/LASBenchmark

ENTRYPOINT ["/pointcloud-tiler/Schwarzwald/build/Release/Schwarzwald"]
