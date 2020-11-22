### Readme
Merge MTS to CARLA 0.9.10 

### Test Environment
* OS: Ubuntu 18.04
* CPU: Intel(R) Core(TM) i5-10500 CPU @ 3.10GHz
* GPU: GeForce GTX 1070
* RAM: 16GB

### Get Start 
1. ```git clone``` whole repository to local file system
2. Go to the root directory of CARLA: ```user_path/carla```
3. Under root carla folder, you can find the source code in following path:  ```user_path/carla/LibCarla/source/carla```.
4. Replace the original folders found in the path at previous step ```client```, ```road``` and ```trafficmanager``` with the ones you have cloned in first step. 
(**Note**: you may backup the original folders before you replace them)
5. Go back to the root directory  ```user_path/carla``` and type the command in terminal to make new package:
```bash= 
user_path/carla$ make package
```
6. The result of ```make package``` will show up in ```user_path/carla/Dist```. In general, there will be a ```.tar``` file and a folder named with ```CARLA_new_version_number``` and may have suffix ```-dirty```.
7. Execute the CARLA simulator with the lateset package.
