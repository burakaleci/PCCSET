
#ifndef TOOLOPERATIONS_H
#define TOOLOPERATIONS_H

// Standard
#include <iostream>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// QT
#include <QDialog>
#include <QColor>

// User defined
#include "PCCSET.h"
#include "ui_PCCSET.h"
#include "SnapshotDialog.h"
#include "ExcelDialog.h"

using namespace std;

// Forward declaration
class PCCSET;

class ToolOperations:public QDialog
{
     Q_OBJECT

public:

    // Constructor and Destructor
    ToolOperations(PCCSET *);
    ~ToolOperations(void);
    
    // Public functions for tools operations
    void getBackgroundColor(void);
    void background(QColor);
    void snapshot(void);
    void excel(void);
    void synchoniser(void);
    void getColor(const string &);
    void AreaPicking(const pcl::visualization::AreaPickingEvent &,pcl::visualization::PCLVisualizer::Ptr);
    void Mouse(const pcl::visualization::MouseEvent &,pcl::visualization::PCLVisualizer::Ptr);
    void PointPicking(const pcl::visualization::PointPickingEvent&,pcl::visualization::PCLVisualizer::Ptr);
    void colorize(int,uint32_t,const string &);

public slots:

    void snapshotDialogOpened(void);
    void snapshotBrowseButtonClicked(void);
    void excelDialogOpened(void);
    void excelBrowseButtonClicked(void);

private:

    /// Utility functions for tools operations
    void creatingSheet(string,string,string,string);
    string to_string_excel(double);

    // Private data attributes for tools operations
    PCCSET *pccset;
    SnapshotDialog *sD;
    ExcelDialog *eD;
    string snapshotBrowsePath;
    QString snapshotCurrentPath;
    string excelBrowsePath;
    QString excelCurrentPath;
    bool flagSynchoniser;
    uint32_t gtRGB;
    uint32_t sampleRGB;
    QColor backgroundColor;

};

#endif // TOOLOPERATIONS_H
