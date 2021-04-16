# INTRODUCTION

In this study, we presented a user-friendly evaluation tool for point cloud classification and segmentation. The tool was realized with the aid of QT (QT Library, 2021) and Point Cloud Library (PCL) (Rusu and Cousins, 2011). The interface contains two QVTK widgets to visualize ground truth and test sample and also an information panel. Point cloud manipulation operations were performed using PCL. The tool can accept many commonly used PCL types as well as files with .txt extension while loading ground truth and test sample since classification or segmentation methods can produce results in different types of files. Many basic functionalities such as undo, redo, saving screen with .pcd and .png extensions were implemented. Besides, the tool provides user-friendly functionalities such as changing the background color of the QVTK widgets, synchronizing ground truth and test sample screens, colorizing points that belong to a class or a segment, producing an excel file for metric results, and taking a snapshot of ground truth, test sample, or from a specific area.

In order to calculate metric and visual results, we handled classification and segmentation problems separately. In the point cloud classification problem, there are a predetermined number of classes, such as tables, chairs, and walls. The classification approaches attempt to determine the class that a point should belong to while considering its attributes. We utilized this information to produce results. First, for each class, the points that belong to a class were determined separately in the ground truth and test sample. Then, the points corresponding to the same class in the ground truth and test sample were evaluated to calculate the metrics. On the other hand, the number of segments is undetermined in the point cloud segmentation problem. The segmentation methods aim to group points into homogenous regions, in other words, segments according to their characteristics such as color, point normals, and curvature. For that reason, corresponding segment pairs between ground truth and test sample are also undetermined in the point cloud segmentation problem. The proposed tool offers two options (automatic and manual) to decide paired segments. The automatic pair-making option considers the number of points and bounding boxes of segments, while the manual option allows choosing points by clicking desired segments in the ground truth and test sample. After the paired segments are specified, the same procedure with the classification problem is applied. The commonly used metrics such as precision, recall, F1, IoU, MIoU, and accuracy were selected for metric results.  

Apart from the traditional approaches that used the K-Nearest Neighbor (KNN) search to retrieve a point via point coordinates, the indices of points were store in the set data structure to speed up the point cloud operations and use the memory efficiently.  A preprocessing step is required to place points in the same order for both ground truth and test sample. After that step, the points can be retrieved in a constant time through the indices. For comparison purposes, the tool functionalities were realized with KNN. The experiments were conducted to examine efficiency of the set data structure with two point clouds, and each consists of approximately half-billion points. The experimental results exhibited that the set data structure significantly decreases the processing time.  

# INSTALLATION

System Requirements:

- Point Cloud Library (PCL)
- XLNT Excel Library(xlnt-duplicate-string-phonetic version)
- QT5 

How to run the tool:

Firstly, you have to compile the XLNT library. To compile it:

- Download the files from https://github.com/tfussell/xlnt/tree/duplicate-string-phonetic.
- In XLNT files open a terminal and use these commands:

`mkdir build`
`cd build`
`cmake ..`
`make`

- After using these commands, change these paths in project file's CMakeList.txt.

`include_directories(/home/fatih/Desktop/xlnt-duplicate-string-phonetic/include)`
`include_directories(/home/fatih/Desktop/xlnt-duplicate-string-phonetic/source)`
`include_directories(/home/fatih/Desktop/xlnt-duplicate-string-phonetic/build/source)`
`link_directories(/home/fatih/Desktop/xlnt-duplicate-string-phonetic/include)`
`link_directories(/home/fatih/Desktop/xlnt-duplicate-string-phonetic/source)`
`link_directories(/home/fatih/Desktop/xlnt-duplicate-string-phonetic/build/source)`

- Also add this command and change the path:

`target_link_libraries(${PROJECT_NAME} /home/fatih/Desktop/xlnt-duplicate-string-phonetic/build/source/libxlnt.so)`

**Note:**  
If you dont build the XLNT library, you can't find the "libxlnt.so".




Download the files and use these commands in the project file:

- mkdir build
- cd build
- cmake ..
- make ..
