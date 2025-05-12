# SpectralUpsamplingImpls

Small program to convert RGB to spectum

## How to build

Simply run 
~~~
./build.sh
~~~
 to build program (Ninja required).

## Usage
Run
~~~
./build/converter
~~~
Parameters: 
* -f \<path\> : input file;
* -c \<hex\> : input color in RGB (upsampling only);
* -m \<method\> : upsampling method;
* -D \<path\> : directory to output (upsampling only);
* -n \<name\> : name for output (upsampling only);
* --downsample : downsample instead of upsampling (requires -f).
