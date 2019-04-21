# ECEN 5823 Bluetooth Mesh Skeleton Project

This project contains skeleton code used for coursework in University of Colorado [ECEN 5823 IoT Embedded Firmware](https://sites.google.com/colorado.edu/ecen5823/home).

Below is an overview of the sequence used to generate this repository:
* The project was generated starting with the new project Wizard built into [Simplicity Studio 4](https://www.silabs.com/products/development-tools/software/simplicity-studio).  
* The AppBuilder project was used with application type "Bluetooth Mesh SDK" with version 2.11.2.0 and Bluetooth SDK 2.9.3.0, application stack "Bluetooth Mesh SDK 1.4.1.0"
* The SOC- BT Mesh Empty project application was used.
* Board and part were configured for BRD4104A Rev 00 and EFR32BG13P632F512GM48 respectively
* Configurations were setup to target GNU ARM 7.2.1.
* Simplicity project Workspace paths were setup to pull in emlib functions needed for display support, including middleware/glib directory and glib/glib glib/dmd directories
* Relevant emlib project files were copied from SiliconLabs\SimplicityStudio\v4\developer\sdks\gecko_sdk_suite\v2.5\platform as needed and added into the respective directories at the root.
* The main.c file in the root folder was renamed [gecko_main.c](gecko_main.c).  Contents of the main while loop were moved into functions and the main() function was #ifdef'd out.
* The [src](src) subfolder was added to contain code specific to the ECEN 5823 course and source files were added to support ECEN 5823 and the simplicity studio exercise assignment.


## Bluetooth Mesh Final Project Update 1 [April 13, 2019]

* In gecko_main.c, lpn_init() and lpn_deinit() are added.
* In gecko_main.c, after the device is provisioned, lpn_init() is called that configures the node as a low power node with minimum friend queue length as 2 and poll 
timeout of 5seconds and initiate friendship request using gecko_cmd_mesh_lpn_establish_friendship.
* As of April 13, Low power node is able to establish friendship with the friend node and relay button press/release info to the friend node using a generic On Off 
Model
* [Group Project Document Folder Link](https://drive.google.com/drive/u/0/folders/19pP2BnPFkOunKWceiNoMzz06ycwfbmVY)
* [Individual Project Document Folder Link](https://drive.google.com/drive/u/0/folders/13ImMufIetzgHsnXX1L5a2Kr49Aut80EE) 

## Bluetooth Mesh Final Project Update 1 [April 20, 2019]

* In gecko_main.c, routines for adding data to / retrieving data from persistent storage are added
* In the project, i2c.c and i2c.h has the required routines for interfacing APDS 9301 sensor
* CLIENT_LEVEL_MODEL AND GENERIC_CLIENT_ON_OFF models have been added and accordingly the .isc file is modified to add these models
* In gecko_main.c, publish_data() is added which takes arguments related to the respective models and publishes the data to the friend node
* As of April 20, my node is able to publish data from the above mentioned 2 models to the friend node and friend node is able to receive the data properly
* [Group Project Document Folder Link](https://drive.google.com/drive/u/0/folders/19pP2BnPFkOunKWceiNoMzz06ycwfbmVY)
* [Individual Project Document Folder Link](https://drive.google.com/drive/u/0/folders/13ImMufIetzgHsnXX1L5a2Kr49Aut80EE)

