# Aqua-Sim-NG + ndnSIM

-------------------------------------

## About this branch

This Aqua-Sim branch is used for connecting ndnSIM with aqua-sim stack of L3-L1 protocols. General information about aqua-sim can be found in the master branch.

--------------------------------------

## Installation

1. Install ndnSIM using the official documentation, provided here:
https://ndnsim.net/current/

2. Clone "ndnSIM" branch of aqua-sim-ng repository and place it under /src folder inside ndnSIM:

<pre><code>
cd <ndnSIM_working_directory>/src/
git clone url
</code></pre>

3. Re-build the ndnSIM

<pre><code>
./waf build
</code></pre>

--------------------------------------

## Example

A simple example-script for running NDN application over aqua-sim stack is provided under:

<pre><code>
cd <ndnSIM_working_directory>/src/aqua-sim-ng/examples/ndn-simple-aqua-broadcast.cc
git clone url
</code></pre>

To run it, please copy this script into `scratch` folder and execute it:

<pre><code>
cp <ndnSIM_working_directory>/src/aqua-sim-ng/examples/ndn-simple-aqua-broadcast.cc <ndnSIM_working_directory>/scratch/
./waf --run ndn-simple-aqua-broadcast
</code></pre>

You might want to run the script above with the standard debuggin options ndnSIM use (see the official documentation: https://ndnsim.net/current/intro.html). For example, showing NDNConsumer debug-information:

<pre><code>
NS_LOG=ndn.Consumer ./waf --run ndn-simple-aqua-broadcast</code></pre>


--------------------------------------

#### Contact

Dmitrii Dugaev : <ddugaev@gradcenter.cuny.edu>

Zheng Peng, Ph.D : <zheng@cs.ccny.cuny.edu>
