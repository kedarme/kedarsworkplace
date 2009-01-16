QuickStart:

1) Bring up VEHICLES.MDL
2) Hit UPDATE SIM button on FrontEnd GUI
3) After the update has finished (see Matlab Command line), Hit RUN SIM button.


The following is a brief description of the files in this ZIP file.

DOTHIS.M
AIAA-2002-4495.pdf
FRONTEND.FIG
FRONTEND.M
INITTHIS.M
MAKE_UAVMODES.M
MAKE_UAVVR.M
MYFISH.WRL
SF_UAVMODE_TEMPLATE.MDL
STATEIC.FIG
STATEIC.M
TEMP.WRL
VEHICLEDATA.M
VEHICLES.MDL
VR_UAV.WRL

DOTHIS.M - 
 this m-file is used to update the simulation.  During simulation update the
 outline in this m file is followed.  Namely, making the VR world file TEMP.WRL,
 making the Stateflow machine, update the Simulink diagram to update ports and
 signal widths, reattach the VR sink.  This m-file calls MAKE_UAVVR and MAKE_UAVMODES
 
AIAA-2002-4495.pdf -
 PDF file of the AIAA paper that was presented on this demo.
 
FRONTEND.FIG - 
 FIG file associated with FRONTEND.M GUI
 
FRONTEND.M -
 Frontend GUI which is used as a front end to control the simulation.
 This GUI allows the following to be changed
   number of vehicles, 
   number size and location of obstacles,
   number size and location of targets,
   vehicle ICs
   
INITTHIS.M -
 This m-file is called in various blocks of the simulation.  Use Simulink Finder to find them.
 It will correctly populate constants and ICs with the correct width for a given
 simulation run.  See file for list of what it initializes.
 
MAKE_UAVMODES.M - 
 This file uses the Stateflow API to rebuild the stateflow chart based on number of vehicles.
 
MAKE_UAVVR.M - 
 This file rebuilds the VR world file (temp.wrl) using the VR API based on number of
 vehicles, obstacles, and targets.
 
MYFISH.WRL - 
 This is the VR representation of the vehicle.
 
SF_UAVMODE_TEMPLATE.MDL - 
 This is a simulink model that houses a Stateflow machine that is used as a template for
 each vehicle when the full state machine is constructed.  This was done using the Stateflow
 clipboard such that it would be easy to modify each vehicle since it can be done graphically.
 
STATEIC.FIG - 
 FIG file associated with STATEIC.M GUI.
 
STATEIC.M - 
 State IC GUI used to set initial conditions for each vehicle.
 
TEMP.WRL - 
 Temporary WRL file used by the simulation.  This WRL file gets rewritten to take into
 account changing vehicles, obstacles, and targets.
 
VEHICLEDATA.M -
 File that loads in vehicle specific data.  Also has collaborative control specific data.
 
VEHICLES.MDL -
 Main Simulink model of multiple uav collabrative control simulation.
 
VR_UAV.WRL - 
 Base WRL file that is used to build TEMP.WRL from.  This is the least common denominator for
 all TEMP.WRL files.
