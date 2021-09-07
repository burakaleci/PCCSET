
// Standard
#include <iostream>
#include <random>
#include <string>
#include <set>
#include <stack>
#include <vector>
#include <typeinfo>
#include <fstream>
#include <algorithm>   
#include <chrono>

// PCL
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/area_picking_event.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/file_io.h>
#include <pcl/common/io.h>

// QT
#include <QFileDialog>
#include <QMessageBox>
#include <QColorDialog>
#include <QColor>
#include <QLabel>
#include <QCloseEvent>

// VTK
#include <vtkRenderWindow.h>

// User defined
#include "PCCSET.h"
#include "ui_PCCSET.h"

using namespace std;

PCCSET::PCCSET(QWidget *parent)
  : QMainWindow(parent),
    ui(new Ui::PCCSET),
    gtScreen (new pcl::visualization::PCLVisualizer("GT",false)),
    sampleScreen (new pcl::visualization::PCLVisualizer("Sample",false))
{
  
    ui->setupUi(this);

    ui->widget_gt->SetRenderWindow (gtScreen->getRenderWindow());
    gtScreen->setupInteractor (ui->widget_gt->GetInteractor (), ui->widget_gt->GetRenderWindow ());
    gtScreen->registerAreaPickingCallback(&PCCSET::AreaPickingCallback, *this,(void*)&gtScreen);
    gtScreen->registerMouseCallback(&PCCSET::MouseCallback, *this,(void*)&gtScreen); 
    gtScreen->registerPointPickingCallback (&PCCSET::PointPickingCallback,*this,(void*)&gtScreen);
           
    ui->widget_sample->SetRenderWindow (sampleScreen->getRenderWindow());
    sampleScreen->setupInteractor (ui->widget_sample->GetInteractor (), ui->widget_sample->GetRenderWindow ());
    sampleScreen->registerAreaPickingCallback(&PCCSET::AreaPickingCallback, *this,(void*)&sampleScreen);
    sampleScreen->registerMouseCallback(&PCCSET::MouseCallback, *this,(void*)&sampleScreen);
    sampleScreen->registerPointPickingCallback (&PCCSET::PointPickingCallback,*this,(void*)&sampleScreen);
    
    cloud_groundtruth = pcl::PointCloud<pcl::PointXYZRGBL>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBL>);
    cloud_sample = pcl::PointCloud<pcl::PointXYZRGBL>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBL>);

    fO= new FileOperations(this);        // create FileOperations object pointer 
    gtEO = new EditOperations(this);
    sampleEO = new EditOperations(this);
    vO= new ViewOperations(this);
    tO= new ToolOperations(this);
    eO= new EvaluateOperations(this);

    /// Connect Menu and buttons
    connectFileMenuAndButtons();
    connectEditMenuAndButtons();
    connectViewMenuAndButtons();
    connectToolsMenuAndButtons();
    connectEvaluateMenuAndButtons();

    flagColorize=false;
    flagManualPair=false;
    
}

PCCSET::~PCCSET(void)
{
    delete ui;
    delete fO;
    delete gtEO;
    delete sampleEO;
    delete vO;
    delete tO;
    delete eO;
}

void PCCSET::PointPickingCallback (const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
    pcl::visualization::PCLVisualizer::Ptr viewer = *static_cast<pcl::visualization::PCLVisualizer::Ptr *> (viewer_void);
  
    if (flagManualPair){
      if (eO->PointPicking(event,viewer))
        flagManualPair=false;
    }
    else if (flagColorize){
      tO->PointPicking(event,viewer);
      flagColorize=false;
    }
}

void PCCSET::MouseCallback(const pcl::visualization::MouseEvent &event, void* viewer_void)
{
    pcl::visualization::PCLVisualizer::Ptr viewer = *static_cast<pcl::visualization::PCLVisualizer::Ptr *> (viewer_void);
    tO->Mouse(event,viewer);
}

void PCCSET::AreaPickingCallback(const pcl::visualization::AreaPickingEvent &event, void* viewer_void)
{
    pcl::visualization::PCLVisualizer::Ptr viewer = *static_cast<pcl::visualization::PCLVisualizer::Ptr *> (viewer_void);
    tO->AreaPicking(event,viewer);
}

void PCCSET::gtFileOpenClicked(void)
{
    fO->fileOpen("GT");
}

void PCCSET::sampleFileOpenClicked(void)
{
    fO->fileOpen("Sample");
}

void PCCSET::gtSaveFileClicked(void)
{
    fO->fileSave("GT");
}

void PCCSET::sampleSaveFileClicked(void)
{
    fO->fileSave("Sample");
}

void PCCSET::gtSaveAsClicked(void)
{
    fO->fileSaveAs("GT");
}

void PCCSET::sampleSaveAsClicked(void)
{
    fO->fileSaveAs("Sample");
}

void PCCSET::gtExportAsClicked(void)
{
    fO->fileExportAs("GT");
}

void PCCSET::sampleExportAsClicked(void)
{
    fO->fileExportAs("Sample");
}

void PCCSET::closeEvent (QCloseEvent *event)
{
    fO->appClose();
}

void PCCSET::exitClicked(void)
{
    fO->appClose();
}

void PCCSET::gtClearClicked(void)
{
    gtEO->gtClear();
    sampleEO->sampleClear();
}

void PCCSET::sampleClearClicked(void)
{
    sampleEO->sampleClear();
}

void PCCSET::gtUndoClicked(void)
{
    gtEO->gtUndo();  
}

void PCCSET::gtRedoClicked(void)
{
    gtEO->gtRedo();
}

void PCCSET::sampleUndoClicked(void)
{
    sampleEO->sampleUndo();
}

void PCCSET::sampleRedoClicked(void)
{
    sampleEO->sampleRedo();
}

void PCCSET::resetPointSizeClicked(void)
{
    vO->resetPointSize();
}

void PCCSET::increasePointSizeClicked(void)
{
    vO->increasePointSize();
}

void PCCSET::decreasePointSizeClicked(void)
{
    vO->decreasePointSize();
}

void PCCSET::backgroundClicked(void)
{
    tO->getBackgroundColor(); 
}

void PCCSET::snapshotClicked(void)
{
    tO->snapshot();
}

void PCCSET::excelClicked(void)
{
    tO->excel();
}

void PCCSET::synchoniserClicked(void)
{
    tO->synchoniser();
}

void PCCSET::gtColorizeClicked(void)
{
    tO->getColor("GT");
    flagColorize=true;
}

void PCCSET::sampleColorizeClicked(void)
{
    tO->getColor("Sample");
    flagColorize=true;
}

void PCCSET::calculateMetricsClicked()
{
    eO->calculateMetrics();
}

void PCCSET::automaticPairClicked()
{
    eO->automaticPair();
}

void PCCSET::manualPairClicked()
{
    eO->manualPair();
    flagManualPair=true;
}

void PCCSET::errorMapClicked(void)
{
    eO->errorMap();  
}

void PCCSET::connectFileMenuAndButtons(void)
{
    connect(ui->b_gtOpen, SIGNAL(clicked()), this, SLOT(gtFileOpenClicked()));
    connect(ui->m_gtOpen, SIGNAL(triggered()), this, SLOT(gtFileOpenClicked()));
    connect(ui->b_sampleOpen, SIGNAL(clicked()), this, SLOT(sampleFileOpenClicked()));
    connect(ui->m_sampleOpen, SIGNAL(triggered()), this, SLOT(sampleFileOpenClicked()));

    connect(ui->b_gtSave, SIGNAL(clicked()), this, SLOT(gtSaveFileClicked()));
    connect(ui->m_gtSave, SIGNAL(triggered()), this, SLOT(gtSaveFileClicked()));
    connect(ui->b_sampleSave, SIGNAL(clicked()), this, SLOT(sampleSaveFileClicked()));
    connect(ui->m_sampleSave, SIGNAL(triggered()), this, SLOT(sampleSaveFileClicked()));
    
    connect(ui->b_gtSaveAs, SIGNAL(clicked()), this, SLOT(gtSaveAsClicked()));
    connect(ui->m_gtSaveAs, SIGNAL(triggered()), this, SLOT(gtSaveAsClicked()));
    connect(ui->b_sampleSaveAs, SIGNAL(clicked()), this, SLOT(sampleSaveAsClicked()));
    connect(ui->m_sampleSaveAs, SIGNAL(triggered()), this, SLOT(sampleSaveAsClicked()));

    connect(ui->b_gtExportAs, SIGNAL(clicked()), this, SLOT(gtExportAsClicked()));
    connect(ui->m_gtExportAs, SIGNAL(triggered()), this, SLOT(gtExportAsClicked()));
    connect(ui->b_sampleExportAs, SIGNAL(clicked()), this, SLOT(sampleExportAsClicked()));
    connect(ui->m_sampleExportAs, SIGNAL(triggered()), this, SLOT(sampleExportAsClicked()));
    
    connect(ui->m_exit, SIGNAL(triggered()), this, SLOT(exitClicked()));
}

void PCCSET::connectEditMenuAndButtons(void)
{
    connect(ui->b_gtClear, SIGNAL(clicked()), this, SLOT(gtClearClicked()));
    connect(ui->m_gtClear, SIGNAL(triggered()), this, SLOT(gtClearClicked()));
    connect(ui->b_sampleClear, SIGNAL(clicked()), this, SLOT(sampleClearClicked()));
    connect(ui->m_sampleClear, SIGNAL(triggered()), this, SLOT(sampleClearClicked()));
    
    connect(ui->b_gtUndo, SIGNAL(clicked()), this, SLOT(gtUndoClicked()));
    connect(ui->m_gtUndo, SIGNAL(triggered()), this, SLOT(gtUndoClicked()));
    connect(ui->b_sampleUndo, SIGNAL(clicked()), this, SLOT(sampleUndoClicked()));  
    connect(ui->m_sampleUndo, SIGNAL(triggered()), this, SLOT(sampleUndoClicked()));  

    connect(ui->b_gtRedo, SIGNAL(clicked()), this, SLOT(gtRedoClicked()));
    connect(ui->m_gtRedo, SIGNAL(triggered()), this, SLOT(gtRedoClicked()));
    connect(ui->b_sampleRedo, SIGNAL(clicked()), this, SLOT(sampleRedoClicked())); 
    connect(ui->m_sampleRedo, SIGNAL(triggered()), this, SLOT(sampleRedoClicked()));
}

void PCCSET::connectViewMenuAndButtons(void)
{
    connect(ui->m_resetPointSize, SIGNAL(triggered()), this, SLOT(resetPointSizeClicked()));
    connect(ui->m_decreasePointSize, SIGNAL(triggered()), this, SLOT(decreasePointSizeClicked())); 
    connect(ui->m_increasePointSize, SIGNAL(triggered()), this, SLOT(increasePointSizeClicked()));
}

void PCCSET::connectToolsMenuAndButtons(void)
{
    connect(ui->b_background, SIGNAL(clicked()), this, SLOT(backgroundClicked()));
    connect(ui->m_background, SIGNAL(triggered()), this, SLOT(backgroundClicked())); 
    
    connect(ui->b_snapshot, SIGNAL(clicked()), this, SLOT(snapshotClicked()));
    connect(ui->m_snapshot, SIGNAL(triggered()), this, SLOT(snapshotClicked()));
    
    connect(ui->b_excel, SIGNAL(clicked()), this, SLOT(excelClicked()));
    connect(ui->m_excel, SIGNAL(triggered()), this, SLOT(excelClicked())); 
    
    connect(ui->b_synchroniser,SIGNAL(clicked()),this,SLOT(synchoniserClicked()));   
    connect(ui->m_synchroniser,SIGNAL(triggered()),this,SLOT(synchoniserClicked())); 
    
    connect(ui->b_gtColorize, SIGNAL(clicked()), this, SLOT(gtColorizeClicked()));  
    connect(ui->m_gtColorize, SIGNAL(triggered()), this, SLOT(gtColorizeClicked()));  
    connect(ui->b_sampleColorize,SIGNAL(clicked()),this,SLOT(sampleColorizeClicked()));
    connect(ui->m_sampleColorize,SIGNAL(triggered()),this,SLOT(sampleColorizeClicked()));
}

void PCCSET::connectEvaluateMenuAndButtons(void)
{
    connect(ui->b_calculateMetrics, SIGNAL(clicked()), this, SLOT(calculateMetricsClicked()));
    connect(ui->m_calculateMetrics, SIGNAL(triggered()), this, SLOT(calculateMetricsClicked()));
    
    connect(ui->b_automaticPair, SIGNAL(clicked()), this, SLOT(automaticPairClicked()));
    connect(ui->m_automaticPair,SIGNAL(triggered()), this, SLOT(automaticPairClicked())); 
    
    connect(ui->b_manualPair, SIGNAL(clicked()), this, SLOT(manualPairClicked()));
    connect(ui->m_manualPair,SIGNAL(triggered()), this, SLOT(manualPairClicked()));
    
    connect(ui->b_errorMap, SIGNAL(clicked()), this, SLOT(errorMapClicked()));
    connect(ui->m_errorMapClassification,SIGNAL(triggered()), this, SLOT(errorMapClicked()));
    connect(ui->m_errorMapSegmentation,SIGNAL(triggered()), this, SLOT(errorMapClicked()));
}