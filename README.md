# VrepFetch
This is an independent simulation of Fetch using vrep simulator. 
Only navigation and manipulation modules are implemented.
I also integrate Kinect sensor and activate motors for head.

The Vrep scene can be found inside the scene folder. 
I use external editor for lua scripts. 
For instance, to use __OMPL.ttt__ scene, you need to change the package path from following scripts 
* ConcretBlock 
* fetch
* sholder_pan_joint

```
package.path = package.path .. ";/Users/redwannewaz/Projects/Fetch/?.lua"
```
You need to change only 
/Users/redwannewaz/Projects/Fetch/
Currently the robot does not pick up the glass but gets close to the glass. I don't need this task to accomplish. I integrate OMPL lib to see the control of manipulator. Well it works well. __The manipulator (sholder_pan_joint)__ in OMPL.ttt is set to static.  
If you are interested in pick and place task, you may use fetchModle.ttt where you can see griper actuation as well. 
Note that CustomBlock is a custom script which shows the yellow grid. If you don't need grid, you can delete this script.

## Things to do 
* Torso control 
* Head control 
