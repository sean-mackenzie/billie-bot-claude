MSOSA MACRO README

NOTES:
	* This assumes you already have all the blocks in the model
		(see CAMEO_IMPORT_README.txt if you need to import blocks)


1. Create a .csv like: 
	BlockName,PropertyName,Type,DefaultValue,Documentation
	RPLidar_A1,outerRadius_m,Real,0.0493,"Outer radius of the lidar in meters"
	RPLidar_A1,height_m,Real,0.060,"Physical height of the lidar"
	RPLidar_A1,frameName,String,"laser_frame","TF frame name"

2. If you need to create the Macro in MSOSA:
	1. Tools > Macro > Create Macro
	2. Copy the Groovy code and click Save
	3. Give the macro a name, description, and click "Add macro to model"
		(you don't need to do anything else)
	4. Press ok

3. In MSOSA:
	1. Move all the blocks you want to import Value Properties onto a single diagram.
	2. Select all the blocks. 
	3. Tools > Macros > Select the macro you created > Run
	4. You will be prompted to select a .csv file
	5. Click run


	