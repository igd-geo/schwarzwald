FROM conanio/gcc7 AS build

RUN whoami

RUN pip install conan --upgrade
RUN conan remote add bincrafters https://api.bintray.com/conan/bincrafters/public-conan
#RUN apt-get update && apt-get install -y git cmake libboost-all-dev

RUN sudo mkdir /data
RUN sudo chown -c conan /data 
WORKDIR /data

RUN git clone https://github.com/m-schuetz/LAStools.git
WORKDIR /data/LAStools/LASzip
RUN mkdir build
RUN cd build && cmake -DCMAKE_BUILD_TYPE=Release ..
RUN cd build && make

WORKDIR /data

# Check out and build terminalpp. Checking out a specific commit prior to the change to using fmt library because it does not link with fmt. Fixing this is a TODO
RUN git clone https://github.com/KazDragon/terminalpp.git && cd ./terminalpp && git checkout 68b497734c57239061b36aee0abf947db0f51c9f

#TODO Set C++ library version to libstdc++11
#RUN sudo mkdir ~/.conan/profiles
#RUN sudo sh -c 'echo "compiler.libcxx=libstdc++11" > ~/.conan/profiles/default'

WORKDIR /data/terminalpp

RUN conan install .
RUN cmake -DCMAKE_BUILD_TYPE=Release .
RUN make

WORKDIR /data
RUN mkdir PotreeConverter
WORKDIR /data/PotreeConverter
ADD . /data/PotreeConverter
RUN mkdir build
RUN cd build && cmake -DCMAKE_BUILD_TYPE=Release -DLASZIP_INCLUDE_DIRS=/data/LAStools/LASzip/dll -DLASZIP_LIBRARY=/data/LAStools/LASzip/build/src/liblaszip.so -DTERMINALPP_INCLUDE_DIRS=/data/terminalpp/include -DTERMINALPP_LIBRARY=/data/terminalpp/libterminalpp.a .. 
RUN cd build && make
RUN cp -R /data/PotreeConverter/PotreeConverter/resources/ /data

# copy libproj.so dependency to a temporary directory
RUN ldd /data/PotreeConverter/build/Release/PotreeConverter | grep 'libproj.so' | awk '{print $3}' | xargs -I '{}' cp -v '{}' /tmp/

# after building, create smaller image that only contains the binary
# and its dependencies
FROM ubuntu:artful

# copy dependencies
COPY --from=build /home/conan/.hunter/_Base/0b8c31b/2af030b/adeda0f/Install/lib/ /home/conan/.hunter/_Base/0b8c31b/2af030b/adeda0f/Install/lib/
COPY --from=build /data/LAStools/LASzip/build/src/liblaszip.so /usr/lib/liblaszip.so
COPY --from=build /tmp/libproj.so* /usr/lib/

# copy binary
COPY --from=build /data/PotreeConverter/build/Release/PotreeConverter /potree-3d-tiles/PotreeConverter/build/Release/PotreeConverter

ENTRYPOINT ["/potree-3d-tiles/PotreeConverter/build/Release/PotreeConverter"]
