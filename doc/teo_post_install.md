## Simulation and Basic Control: Now what can I do?

Now that you have installed the basic TEO repository, you're probably wondering what to do.

###  Initializing the communication server

Our current implementation uses <a class="el" href="http://eris.liralab.it/yarpdoc/what_is_yarp.html">YARP</a> for communication. Basic use of YARP requires the use of a centralized server. This server associates the low-level implementation of the communication ports with the names we give them. Before executing any TEO program or application, please launch a yarp server:

```bash
[terminal 1] yarp server
```

### Launching the simulator

Launch with a single command: 

```bash
[terminal 2] teoSim
```

You should get a window similar to the one depicted on Figure 1.

<p align="center">
<img src="http://robots.uc3m.es/dox-teo-main/teoSim.png">
<div align="center">Fig. 1 - An instance of the teoSim program.</div>
</p>


### Changing parameters

Each layer of TEO programs has parameters that can be modified three different ways. They are parsed in the following order (the second parsing overwrites the first, and the third one overwrites both):

  - Default parameters defined in the class header files (*.h or *.hpp extension). You must recompile the project if you change any of these parameters.
  - Configuration files (*.ini or *.xml extension). These files are downloaded from teo-main/share and installed to teo-main/build/share/teo with the <i>cmake</i> command we issued. Within this folder, separate folders are maintained for programs and program layers.
  - Command line (on program execution). Parameters may be modified executing a program using the following format: <i>./program \--parameter new_value</i>.

Let's say, for example, our graphic card supports offscreen rendering and we want [teoSim](http://robots.uc3m.es/dox-teo-main/group__teoSim.html) to load an enviroment that has cameras in the simulated environment. We can execute the program touching the parameters at the command line level which, as we have said, are parsed last:

```bash
[terminal 2] teoSim --env teo_kitchen_cameras.env.xml
```

This specific environment contains simulated cameras that Figure 2. You can learn how to make the connections in the  <a class="el" href="group__testRaveBot.html#testRaveBot_interfacing">testRaveBot</a> documentation.

\image html ravebotCompare.png

We can actually see the default parameters, and the final selected ones, with the <b>\--help</b> parameter:

```bash
[terminal 2] teoSim --env teo_kitchen_cameras.env.xml --help
```

If we want to affect this parameter at a more persistent level, we can change the configuration file. For this specific case, the [teoSim](http://robots.uc3m.es/dox-teo-main/group__teoSim.html) configuration file is located at <i>$TEO_ROOT/app/teoSim/conf/teoSim.ini</i>. In this file, we can see that most parameters are commented out (the <b>//</b> characters). This is a common convention to indicate these are the default parameters set in the headers (the first parsed, as explained previously). Here, we would search to subsitute the line:

```bash
// env teo_kitchen.env.xml           /// env [xml] environment name in abs or rel
```

With a new, uncommented line:

```bash
env teo_kitchen_cameras.env.xml           /// env [xml] environment name in abs or rel
```

### Interfacing

We can interact with this program through port commands as described in
<a class="el" href="group__testRaveBot.html#testRaveBot_interfacing">testRaveBot</a>
and <a class="el" href="group__teoSim.html#teoSim_interfacing">teoSim</a>,
or through the different language APIs as can be seen in the different
<a class="el" href="group__teo__examples.html">TEO examples</a>.

### Even more!

Done! You are now probably interested in one of the following links:

  - install_vision_on_ubuntu
  - You can now compile and install any of the <a class="el" href="programs.html">other programs</a>.
  - These programs are actually packed up and ready to go in our [teo-main Applications (Collections of Programs)!](http://robots.uc3m.es/dox-teo-main/group__teo__applications.html)
