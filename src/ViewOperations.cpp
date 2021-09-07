
// Standard
#include <iostream>

// PCL
#include <pcl/point_types.h>
#include <pcl/common/common.h>

// QT
#include <QMainWindow>
#include <QMessageBox>

// User defined
#include "ViewOperations.h"

using namespace std;

ViewOperations::ViewOperations(PCCSET *p)
    : pccset(p)
{
    pointSize = 1;   
}

ViewOperations::~ViewOperations(void)
{
    
}

void ViewOperations::resetPointSize(void)
{
    pointSize = 1;
    if (!pccset->cloud_groundtruth->points.empty())
        pccset->gtScreen->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, "cloud_groundtruth");
    
    if (!pccset->cloud_sample->points.empty())
        pccset->sampleScreen->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, "cloud_sample");
    
    pccset->ui->widget_gt->update();
    pccset->ui->widget_sample->update();
}

void ViewOperations::increasePointSize(void)
{
    ++pointSize;
    if (!pccset->cloud_groundtruth->points.empty())
        pccset->gtScreen->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, "cloud_groundtruth");
    
    if (!pccset->cloud_sample->points.empty())
        pccset->sampleScreen->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, "cloud_sample");
    
    pccset->ui->widget_gt->update();
    pccset->ui->widget_sample->update();
}

void ViewOperations::decreasePointSize(void)
{
    if(pointSize <= 1)
        QMessageBox::warning(pccset, "Error!", "Point size can't be less than 1.");
    else{
    
        --pointSize;
        if (!pccset->cloud_groundtruth->points.empty())
            pccset->gtScreen->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, "cloud_groundtruth");
     
        if (!pccset->cloud_sample->points.empty())
            pccset->sampleScreen->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, "cloud_sample");
      
        pccset->ui->widget_gt->update();
        pccset->ui->widget_sample->update();
    }
}
    