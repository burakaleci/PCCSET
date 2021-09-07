#ifndef PCCSET_H
#define PCCSET_H

// Standard
#include <iostream>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// QT
#include <QMainWindow>
#include <QFileDialog>
#include <QPixmap>
#include <QRect>

// VTK
#include <vtkRenderWindow.h>

// User defined
#include "FileOperations.h"
#include "EditOperations.h"
#include "ViewOperations.h"
#include "ToolOperations.h"
#include "EvaluateOperations.h"

using namespace std;

// Forward declaration
class FileOperations;
class EditOperations;
class ViewOperations;
class ToolOperations;
class EvaluateOperations;

QT_BEGIN_NAMESPACE
namespace Ui { class PCCSET; }

class PCCSET : public QMainWindow
{
    Q_OBJECT

public:

    // Constructor and Destructor
    explicit PCCSET(QWidget *parent = nullptr);
    ~PCCSET();

    // Public functions for PCCSET
    void AreaPickingCallback(const pcl::visualization::AreaPickingEvent &, void*);
    void MouseCallback(const pcl::visualization::MouseEvent &, void*);
    void PointPickingCallback(const pcl::visualization::PointPickingEvent& event, void*);

    // Public data attributes for PCCSET
    pcl::visualization::PCLVisualizer::Ptr sampleScreen;
    pcl::visualization::PCLVisualizer::Ptr gtScreen;
    
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_sample; 
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_groundtruth;

    /// Set for ground truth labels
    set<int> gtLabels; 
        
    /// Holds segments in a set for ground truth
    vector < set< int > > gtSetVector;

    /// Set for sample labels
    set<int> sampleLabels; 

    /// Holds segments in a set for sample
    vector < set< int > > sampleSetVector;

    /// Holds indices of True Positive Samples 
    vector< set <int> > TP;
        
    /// Holds indices of False Negative Samples
    vector< set <int> > FN;
        
    /// Holds indices of False Positive Samples
    vector< set <int> > FP;

    vector< vector<double> > metricResults;

    Ui::PCCSET *ui;
    FileOperations *fO;
    EditOperations *gtEO;
    EditOperations *sampleEO;
    ViewOperations  *vO;
    ToolOperations  *tO;
    EvaluateOperations  *eO;

public slots:

    // File Menu Slots
    void gtFileOpenClicked(void);
    void sampleFileOpenClicked(void);
    void gtSaveFileClicked(void);
    void sampleSaveFileClicked(void);
    void gtSaveAsClicked(void);
    void sampleSaveAsClicked(void);
    void gtExportAsClicked(void);
    void sampleExportAsClicked(void);
    void exitClicked(void);
    void closeEvent (QCloseEvent *);

    // Edit Menu Slots
    void gtClearClicked(void);
    void sampleClearClicked(void);
    void gtUndoClicked(void);
    void sampleUndoClicked(void);
    void gtRedoClicked(void);
    void sampleRedoClicked(void);

    // View Menu Slots
    void resetPointSizeClicked(void);
    void increasePointSizeClicked(void);
    void decreasePointSizeClicked(void);
    
    // Tools Menu Slots
    void backgroundClicked(void);
    void snapshotClicked(void);
    void excelClicked(void);
    void synchoniserClicked(void);
    void gtColorizeClicked(void);
    void sampleColorizeClicked(void);
    
    // Evaluate Menu Slots
    void calculateMetricsClicked(void);
    void automaticPairClicked(void);
    void manualPairClicked(void);
    void errorMapClicked(void);
    

private:
    
    // Utility functions for PCCSET
    void connectFileMenuAndButtons(void);
    void connectEditMenuAndButtons(void);
    void connectViewMenuAndButtons(void);
    void connectToolsMenuAndButtons(void);
    void connectEvaluateMenuAndButtons(void);

    // Private data attributes for PCCSET
    bool flagColorize;
    bool flagManualPair;

};

QT_END_NAMESPACE

#endif // PCCSET_H
