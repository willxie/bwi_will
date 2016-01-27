# bwi_will

Please clone this using `git clone --recursive`
To run the code, catkin_make with the rest of the BWI repositories. 

First run:
```
roslaunch bwi_object_search object_search.launch
```

This is similar to segbot_v2.launch.


Then for training(collect data), run these in two separate terminals:

```
rosrun bwi_object_search ar_object_transform
rosrun bwi_object_search object_search
```

For testing (finding object), run:

```
rosrun bwi_object_search object_search_find
```
