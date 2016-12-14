The hratc2017_entry_template package will help the HRATC 2017 competitors jump start their entry!
To know more about HRATC 2017 visit http://inf.ufrgs.br/hratc2017/HRATC2017/Welcome.html

### Installation
To learn how to install the complete HRATC 2017 framework visit http://inf.ufrgs.br/hratc2017/HRATC2017/Simulator.html

### Examples in the repository
The examples present on the hratc2017_entry_template package are available under the examples folder. 
These are divided into two sub-folders, cpp and py.

The cpp examples are:
* **get_data** - In this example we show how to read odometry, gps, metal detection information, among other stuff.
* **random_walk_simple_node** - In this example a random walk algorithm is implemented. The goal is to demonstrate how to subscribe to the odometry and laser data and publish velocity commands to the robot.
* **set_mine_node** - In this example we show how to tell the HRATC 2017 Judge that we found a mine.

The py example is:
* **simple_control** - This example is designed to guide you through the system capabilities. Thus it was made as plain as possible. This is not meant to be a complete and exhaustive tutorial. We just illustrate some of the basic features of the system using a sample code in python. There are snippets of code for most of the system capabilities in this package.

### How to run a node?
With the simulator running:

`$ rosrun hratc2017_entry_template name_of_the_node`

Example:

`$ rosrun hratc2017_entry_template simple_control`

### Contributions

 - cpp examples by Gonçalo Cabrita and Renan Maffei;
 - python examples by Guilherme Franco and Renan Maffei;
 - Readmes edits by Gonçalo Cabrita and Renan Maffei.

