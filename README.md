# Schwarzwald - A fast point cloud tiling tool

Schwarzwald is a command line tool to generate acceleration structures for point cloud data, supporting the [3D Tiles format](https://github.com/AnalyticalGraphicsInc/3d-tiles) and [Potree format](https://github.com/potree/potree/). 
Schwarzwald will convert your raw point cloud data into data that can be visualized with [CesiumJS](https://cesium.com/cesiumjs/) and [Potree](https://github.com/potree/potree/).
Schwarzwald is very fast, supporting machines with dozens of logical cores. 

**Supported input formats**
*  LAS/LAZ

**Supported output formats**
*  LAS/LAZ
*  3D Tiles
*  Binary dump

## Build

**Windows build is not tested!**

Requirements:
*  [CMake >= 3.11](https://cmake.org/)
*  [gcc >= 7.5](https://gcc.gnu.org/) or another C++17-compliant compiler
*  (Optional) [Docker >= 18](https://www.docker.com/) 

### Dependencies

Schwarzwald uses `LASzip` from the [lastools](https://github.com/m-schuetz/LAStools.git). Before building Schwarzwald, build `LASzip` like this:

```
cd ~/dev/lastools
git clone https://github.com/m-schuetz/LAStools.git
cd LAStools/LASzip
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release .. && make
```

This should produce `liblaszip.so` in `~/dev/lastools/LASzip/build/src`. 

### Building on Linux

```
cd ~/dev/path-to-this-repository
mkdir -p build/release
cd build/release
cmake -DCMAKE_BUILD_TYPE=Release -DLASZIP_INCLUDE_DIRS=~/dev/lastools/LASzip/dll -DLASZIP_LIBRARY=~/dev/lastools/LASzip/build/src/liblaszip.so ../../
make
```

This should produce the `Schwarzwald` executable in `~/dev/path-to-this-repository/build/Release`.

### Building with Docker

Run `sudo docker build -t schwarzwald:latest .` from the root folder. 

## Usage

Schwarzwald supports two different modes:
*  `tiler` for creating an optimized point cloud structure
*  (**Currently unsupported, might be removed in future versions**) `converter` to convert the result of the `tiler` mode into different output formats

To get information about the possible arguments, call `Schwarzwald` without any arguments. 

### Generating 3D Tiles from LAS/LAZ

The easiest way to generate 3D Tiles is to call Schwarzwald like this:

```
Schwarzwald --tiler -i /path/to/your/LAS/files -o /output/path/for/3D/tiles --output-format 3DTILES
```

This starts a tiling process that generates 3D Tiles and stores them in the specified output folder. 

### Generating Potree tiles from LAS/LAZ

Potree-compatible tiles can be generated like this:

```
Schwarzwald --tiler -i /path/to/your/LAS/files -o /output/path/for/3D/tiles --output-format ENTWINE_LAZ
```

As the name suggests, this generates data in the same format as the [Entwine tool](https://entwine.io/), which is fully compatible with Potree. 

### Tiling parameters

There are several parameters that control the structure of the tiles. They are very similar to the ones that [PotreeConverter](https://github.com/potree/PotreeConverter) supports:

```
  -s [ --spacing ] arg (=0)             Distance between points at root level. 
                                        Distance halves each level.
  -d [ --spacing-by-diagonal-fraction ] arg (=0)
                                        Maximum number of points on the 
                                        diagonal in the first level (sets 
                                        spacing). spacing = diagonal / value
  --sampling arg (=MIN_DISTANCE)        Sampling strategy to use. Possible 
                                        values are RANDOM_GRID, GRID_CENTER, 
                                        MIN_DISTANCE. The quality of the 
                                        resulting point cloud can be adjusted 
                                        with this parameter, with RANDOM_GRID 
                                        corresponding to the lowest quality and
                                        MIN_DISTANCE to the highest quality.
  --tiling-strategy arg (=FAST)         The tiling strategy to use. Valid 
                                        options are FAST or ACCURATE, where 
                                        FAST will yield better performance but 
                                        larger data.
```

There should be little reason to manually set the `tiling-strategy` parameter unless you have strict space and quality requirements. The `FAST` strategy, which is the default, has much better performance but will produce slightly larger data. 

### Handling errors during processing

Depending on your environment, you might want to ignore some errors that can occur during processing, such as unreadable files or unsupported file formats. To do some, use can use the following option: 

```
  --ignore arg (=NONE)                  If provided, all recoverable errors for
                                        the given categories are ignored and 
                                        processing proceeds normally instead of
                                        terminating the program. Specify one or
                                        more of the following possible values:
                                        MISSING_FILES (= ignore missing files)
                                        INACCESSIBLE_FILES (= ignore 
                                        inaccessible files)
                                        UNSUPPORTED_FILE_FORMAT (= ignore files
                                        with unsupported file formats)
                                        CORRUPTED_FILES (= ignore 
                                        corrupted/broken files)
                                        MISSING_POINT_ATTRIBUTES (= ignore 
                                        files that don't have the point 
                                        attributes specified by the -a option. 
                                        Default values will be used for these 
                                        attributes)
                                        ALL_FILE_ERRORS (= ignore all 
                                        recoverable file-related errors)
                                        ALL_ERRORS (= ignore all recoverable 
                                        errors)
                                        NONE (= terminate program on every 
                                        error)
```

### Point attributes

Schwarzwald supports all regular point attributes defined by the [LAS standard (v1.2)](https://www.asprs.org/a/society/committees/standards/asprs_las_format_v12.pdf). For the output formats `ENTWINE_LAS` and `ENTWINE_LAZ`, all attributes are retained in the output LAS/LAZ files. For the `3D_TILES` output format, this is not possible as `3D_TILES` does not natively support all the different attribute types that `LAS` does. Currently, Schwarzwald only stores the positions, RGB colors and intensities with `3D_TILES`. 

It is possible to calculate RGB colors from other attributes when writing `3D_TILES` using the following option:

```
  --calculate-rgb-from arg              Calculate RGB values from one of the 
                                        other point attributes. Accepted values
                                        are: INTENSITY_LINEAR (convert 
                                        intensity values to RGB using a linear 
                                        mapping), INTENSITY_LOG (convert 
                                        intensity values to RGB using a 
                                        logarithmic mapping), NONE (do not 
                                        calculate RGB values, same as not 
                                        specifying this option). This feature 
                                        is only supported when output-format is
                                        3DTILES
```

### Transforming the points into EPSG:4978 (the world coordinate system used by 3D Tiles)

By default, Schwarzwald writes point positions in the same coordinate system as they were in the input files.
If you want to convert the positions into EPSG:4978 (the world coordinate system of 3D Tiles and Cesium), you have to specify the spatial reference system of your input files in [proj format](https://en.wikipedia.org/wiki/PROJ) like so:

```
Schwarzwald --tiler -i /path/to/your/LAS/files -o /output/path/for/3D/tiles --output-format 3DTILES --source-projection "+proj=utm +zone=32 +ellps=WGS84 +datum=WGS84 +units=m +no_defs"
```

This example uses the proj string for the UTM32N spatial reference system. 

## License

Licensed under the Apache License, Version 2.0. 
Loosely based on the source code of PotreeConverter v1.7 by Markus Schütz, licensed under BSD 2-Clause. Source code available at https://github.com/potree/PotreeConverter 
