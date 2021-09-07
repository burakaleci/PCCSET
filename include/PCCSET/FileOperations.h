
#ifndef FILEOPERATIONS_H
#define FILEOPERATIONS_H

// Standard
#include <iostream>
#include <string>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// User defined
#include "PCCSET.h"
#include "ui_PCCSET.h"

using namespace std;

// Forward declaration
class PCCSET;

class FileOperations
{
   
public:

    // Constructor and Destructor
    FileOperations(PCCSET *);
    ~FileOperations(void);
    
    // Public functions for file operations
    void fileOpen(const string &);
    void fileSave(const string &);
    void fileSaveAs(const string &);
    void fileExportAs(const string &);
    void appClose(void);

private:

    // Utility functions for file operations
    bool fileOpenTXT(void);
    bool fileOpenPCD(void);
    bool readLabelFile(const string &,vector < int > &);
    bool readPointFile(void);
    bool mergePointsAndLabels(void);
    void swapPointClouds(void);
    void arrangeSampleData(void);
    void analyzeAndSeparateData(void);
    void initializeGTAndButtons(void);
    void initializeSampleAndButtons(void);
    void printInitialInfo(void);

    // Private data attributes for file operations
    string gtFileName;
    string sampleFileName;
    string source;
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud; 
    PCCSET *pccset;
};

#endif // FILEOPERATIONS_H
