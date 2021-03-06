# INTRODUCTION

The software is a user-friendly evaluation tool for point cloud classification and segmentation. The tool was realized with the aid of QT and PCL. The interface contains two QVTK widgets to visualize ground truth and test sample and also an information panel. Point cloud manipulation operations were performed using PCL. The tool can accept many commonly used PCL types as well as files with .txt extension while loading ground truth and test sample since classification or segmentation methods can produce results in different types of files. Many basic functionalities such as undo, redo, saving screen with .pcd and .png extensions were implemented. Besides, the tool provides user-friendly functionalities such as changing the background color of the QVTK widgets, synchronizing ground truth and test sample screens, colorizing points that belong to a class or a segment, producing an excel file for metric results, and taking a snapshot of ground truth, test sample, or from a specific area.

In order to calculate metric and visual results, we handled classification and segmentation problems separately. In the point cloud classification problem, there are a predetermined number of classes, such as tables, chairs, and walls. On the other hand, the number of segments is undetermined in the point cloud segmentation problem. For that reason, corresponding segment pairs between ground truth and test sample are also undetermined in the point cloud segmentation problem. The proposed tool offers two options (automatic and manual) to decide paired segments. The automatic pair-making option considers the number of points and bounding boxes of segments, while the manual option allows choosing points by clicking desired segments in the ground truth and test sample. After the paired segments are specified, the same procedure with the classification problem is applied. The commonly used metrics such as precision, recall, F1, IoU, MIoU, and accuracy were selected for metric results.  

Apart from the traditional approaches that used the K-Nearest Neighbor (KNN) search to retrieve a point via point coordinates, the indices of points were store in the set data structure to speed up the point cloud operations and use the memory efficiently.  A preprocessing step is required to place points in the same order for both ground truth and test sample. After that step, the points can be retrieved in a constant time through the indices. 

# CITING PCCSET

If you use PCCSET in a scientific publication, we would appreciate citations to the following paper:

Kaleci, B. and Erdo??an, M. F. "A USER-FRIENDLY EVALUATION TOOL FOR POINT CLOUD CLASSIFICATION AND SEGMENTATION", 2nd International Hazar Scientific Researches Conference, pp. 835-852, Baku, Azerbaijan, 2021. 

@inproceedings{Kaleci_Erdogan_2021,

  title={A USER-FRIENDLY EVALUATION TOOL FOR POINT CLOUD CLASSIFICATION AND SEGMENTATION},

  author={Kaleci, B. and Erdo??an, M. F.},

  booktitle={2nd International Hazar Scientific Researches Conference},

  year={2021},

  month={April},

  pages={835-852},

}

# INSTALLATION

System Requirements:

- Cmake >= 3.16
- Point Cloud Library (PCL) >= 1.8
- VTK >= 7.1
- XLNT Excel Library(xlnt-duplicate-string-phonetic version)
- QT5 

How to run the tool:

Firstly, you have to compile the XLNT library. To compile it:

Download the files from https://github.com/tfussell/xlnt/tree/duplicate-string-phonetic.
In XLNT files open a terminal and use these commands:

- `mkdir build`
- `cd build`
- `cmake ..`
- `make`


Download the files for PCCSET and use these commands in the project file:

- `mkdir build`
- `cd build`
- `cmake ..`
- `make`
- Run executable.

**Note:** To compile PCCSET, please use CMakeList.txt in project folder.

Change the following paths according to location of XInt in project file's CMakeList.txt.

- `include_directories({FILE_DIRECTORY}/xlnt-duplicate-string-phonetic/include)`
- `include_directories({FILE_DIRECTORY}/xlnt-duplicate-string-phonetic/source)`
- `include_directories({FILE_DIRECTORY}/xlnt-duplicate-string-phonetic/build/source)`
- `link_directories({FILE_DIRECTORY}/xlnt-duplicate-string-phonetic/include)`
- `link_directories({FILE_DIRECTORY}/xlnt-duplicate-string-phonetic/source)`
- `link_directories({FILE_DIRECTORY}/xlnt-duplicate-string-phonetic/build/source)`

**Note:**  
If you dont build the XLNT library, you can't find the "libxlnt.so".

**Note:Unfortunately the tool is vtk dependent. Please modify vtk-x.x with your vtk version version** 
target_link_libraries(${PROJECT_NAME} ${VTK_LIBRARIES} -lvtkGUISupportQt-x.x)

**Note: if the following commands do not work**

find_package(VTK REQUIRED)

include(${VTK_USE_FILE})

Explicitly set the header directory of QVTKWidget.h

Explicitly set the library directory of libvtkGUISupportQt-x.x.so

# How it works?

[![SC2 Video](https://img.youtube.com/vi/NNydPugppnA/0.jpg)](http://www.youtube.com/watch?v=NNydPugppnA)

### File Menu:

There are 5 functions in the File menu. Open,save, save as, export as and exit. 
Ground truth and sample scenes can be loaded with the open menu. Pcd and txt type files are accepted. If the pcd file does not have a label field, the user can load labels separately.
Pcd files can be saved with save function. This file will be saved into the current directory. 
Pcd files also can be save with save as function. It allows the user to select a path to save the .pcd file.
The user can save a .png file with export as menu. It allows the user to select a path to save the .png file.
If the user clicks the exit menu, the tool asks if the user wants to save the current file. If user decides to save file before exit, the tool will save the pcd file into the current directory.

### Edit Menu:

The Edit menu has 3 different functions. Clear, undo and redo. 

Scenes can be cleaned by the user with the clear operation. After this menu used, new scenes can be added and new operations can be made.
Changes made to scenes can be undone by the user with undo function. If the user has reverted the changes to the scenes by mistake and wants to do the same operation again, can use the redo function. Colorize and background operations handled for the ground truth screen. However, for the test sample, the user can use undo and redo function for automatic pair, manual pair, calculate metrics, and error map functions. Undo and redo functions were implemented via polymorphic classes.

### View Menu: 

View menu has 3 different functions. Increase point size, decrease point size and reset point size. The default point size of the point cloud is 1.
The point size of the point cloud can be increased with increase point size function. After this operation is done, the point size of the point cloud will be increased.
The point size of the cloud can be decreased with decrease point size function. After this operation is done, the point size of the point cloud will be decreased.
If the user changed the point size and wants to return the default point size value, can use reset point size function.

### Tools Menu: 

Tools menu has 5 different functions. Background, snapshot, excel, synchonizer and colorize.
The user can change both screens??? background colors with background function. The tool accepts RGB value or HTML code to use as color. Using this function will change the background color of both screens.
A screenshot can be taken with snapshot function. The user can select which screen will be used when the snapshot action is triggered. User can take screenshots from ground truth screen, test sample screen, and specify an area to take screenshots from. To take a screenshot of a specific area, the user must hit the ???X??? button on the desired screen and define the area boundaries. A saving path can be specified with snapshot function.
After automatic pair, manual pair, or calculate metrics operations are applied; excel operation will be available. User can enter sample name, method name, elapsed time value and can select a target folder. Precision, recall, F1, IoU, MIoU, and accuracy values will be saved in the excel file and the user can check the calculation results easily.
User can change the color of points that belong to the same class or segment with select colorize function. Color of points can be changed in ground truth and test sample separately. A screen will pop up and new color can be selected by user in this pop-up when the function is used. After a color is selected, the user must hit the ???SHIFT??? button and mouse left click at the same time on the desired screen to select which segment will recolor. 

### Evaluate Menu:

Classification and segmentation operations can be done with evaluate menu. 
User can use classification menu to obtain metric and visual results for classification problems. Calculate metrics operation calculates the precision, recall, F1, IoU for each class. Also, MIoU and accuracy values are calculated for the scene. After calculate metrics operation done; the program asks the user to colorize the test sample with the same color with ground truth and depending on the user???s answer it performs an action. Lastly, results are displayed in the Info Panel.
The segmentation menu can be used to obtain metric and visual results for segmentation problems. Evaluating segmentation results is problematic since the number of segments and the segmented pairs are undetermined. The tool offers 2 different options (automatic and manual) to determine paired segments. 
The Automatic Pair takes into account the number of points and bounding boxes of segments. The tool finds a paired segment and shows the bounding boxes of these segments when the user runs the Automatic Pair. A pop-up will be shown to the user if a match found. If the user selects the apply button, then another pop-up is shown and asks the user to colorize the test sample with the same colors with ground truth. Lastly, Automatic Pair operation results are shown in Info Panel. These operations continue until each segment is paired. In some cases, Automatic Pair operation can not find all segments to pair. The user can manually pair these unsegmented labels with Manual Pair operation.
Manual pair operation can be used for segments that cannot be paired in Automatic Pair operation. Also, if the user wants to use Manual Pair from the beginning, can use it. An info pop-up will be shown to the user and can starts to manual pair process by clicking a point from ground truth and then pick a point from a test sample. The segments which unsegmented before will be segmented after this operation and the user will see another pop-up to colorize the paired segments.
Both classification and segmentation menus have Error Map operation. After Automatic Pair, Manual Pair and Calculate Metrics operations done; the user can run Error Map operation and checks which points segmented or classified truly. Correctly classified or segmented points will be shown with blue; incorrectly classified or segmented points will be shown with red. 


