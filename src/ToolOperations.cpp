
// Standard
#include <iostream>
#include <string>

// PCL
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>

// QT
#include <QMainWindow>
#include <QMessageBox>
#include <QColorDialog>
#include <QColor>
#include <QStandardPaths>

// Excel
#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <xlnt/xlnt.hpp>

// User defined
#include "ToolOperations.h"
#include "ui_SnapshotDialog.h"
#include "SnapshotDialog.h"
#include "ExcelDialog.h"
#include "ui_ExcelDialog.h"

using namespace std;

ToolOperations::ToolOperations(PCCSET *p)
  : pccset(p),
    sD(new SnapshotDialog),
    eD(new ExcelDialog),
    backgroundColor(QColor(0,0,0))
{
    // Snapshot Ui
    connect(sD->ui->CancelOkButtons->button(QDialogButtonBox::Ok), SIGNAL(clicked()), this, SLOT(snapshotDialogOpened()));
    connect(sD->ui->browseButton, SIGNAL(clicked()), this, SLOT(snapshotBrowseButtonClicked())); 

    // Excel Ui
    connect(eD->ui->CancelOkButtons->button(QDialogButtonBox::Ok), SIGNAL(clicked()), this, SLOT(excelDialogOpened()));
    connect(eD->ui->browseButton, SIGNAL(clicked()), this, SLOT(excelBrowseButtonClicked()));

    snapshotCurrentPath = QStandardPaths::HomeLocation;
    snapshotBrowsePath = snapshotCurrentPath.toStdString();
    excelCurrentPath = QStandardPaths::writableLocation(QStandardPaths::HomeLocation);
    excelBrowsePath = excelCurrentPath.toStdString();
    
    flagSynchoniser = false;
  
}

ToolOperations::~ToolOperations(void)
{
    delete sD;
    delete eD;
}

void ToolOperations::getBackgroundColor(void)
{
    QColor newColor = QColorDialog::getColor(Qt::black, pccset, "Pick a color",  QColorDialog::DontUseNativeDialog);
    
    shared_ptr<UndoRedoBase> bAction(new BackgroundAction(pccset,backgroundColor,newColor)); // create command for background
    pccset->gtEO->executeCmd(bAction);
    pccset->ui->b_gtUndo->setEnabled(true);
    pccset->ui->m_gtUndo->setEnabled(true);

    backgroundColor.setRgb(newColor.red(),newColor.green(),newColor.blue());
}

void ToolOperations::background(QColor c)
{
    pccset->gtScreen->setBackgroundColor(c.red()/255.0,c.green()/255.0,c.blue()/255.0);
    pccset->sampleScreen->setBackgroundColor(c.red()/255.0,c.green()/255.0,c.blue()/255.0);
    pccset->ui->widget_gt->update();
    pccset->ui->widget_sample->update();

    backgroundColor.setRgb(c.red(),c.green(),c.blue()); 
}

void ToolOperations::snapshot(void)
{
    sD->show();
}

void ToolOperations::excel(void)
{
    eD->show();
}

void ToolOperations::synchoniser(void)
{
    if(flagSynchoniser==false){
      pccset->ui->b_synchroniser->setChecked(true);
      pccset->ui->m_synchroniser->setChecked(true);
      flagSynchoniser = true;
    }
    else if(flagSynchoniser==true){
      pccset->ui->b_synchroniser->setChecked(false);
      pccset->ui->m_synchroniser->setChecked(false);
      flagSynchoniser=false;
    }
}

void ToolOperations::getColor(const string & source)
{
    QColor selectedColor = QColorDialog::getColor(Qt::black, pccset, "Pick a color",  QColorDialog::DontUseNativeDialog);

    if (source.find("GT")!=string::npos)
      gtRGB = ((uint32_t)selectedColor.red() << 16 | (uint32_t)selectedColor.green() << 8 | (uint32_t)selectedColor.blue()); 
    if (source.find("Sample")!=string::npos)
      sampleRGB = ((uint32_t)selectedColor.red() << 16 | (uint32_t)selectedColor.green() << 8 | (uint32_t)selectedColor.blue()); 

    QMessageBox::StandardButton pickPoint= QMessageBox::information(pccset, "Pick a Point", "Use SHift + Left click when you pick.");
}

void ToolOperations::AreaPicking(const pcl::visualization::AreaPickingEvent &event,pcl::visualization::PCLVisualizer::Ptr viewer)
{
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    const char *s =viewer->getRenderWindow()->GetWindowName();
    string source(s);

    if (event.getPointsIndices(inliers->indices) != false){

      pcl::PointCloud<pcl::PointXYZRGBL>::Ptr selectedCloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
      pcl::PointCloud<pcl::PointXYZRGBL>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
      string cloud_id;

      if (source.find("GT")!=string::npos){
        sourceCloud=pccset->cloud_groundtruth;
        cloud_id="cloud_groundtruth";
      }
          
      if (source.find("Sample")!=string::npos){
        sourceCloud=pccset->cloud_sample;
        cloud_id="cloud_sample";
      }
              
      pcl::ExtractIndices<pcl::PointXYZRGBL> extract;
      extract.setInputCloud(sourceCloud);
      extract.setIndices(inliers);
      extract.filter(*selectedCloud);

      viewer->removeAllPointClouds();
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBL> rgbXYZ(selectedCloud);
      viewer->addPointCloud<pcl::PointXYZRGBL>(selectedCloud,rgbXYZ,"selectedCloud");
          
      pccset->ui->widget_gt->update();
      pccset->ui->widget_gt->GetInteractor()->Render();
      pccset->ui->widget_gt->GetRenderWindow()->Render();
      pccset->ui->widget_sample->update();
      pccset->ui->widget_sample->GetInteractor()->Render();
      pccset->ui->widget_sample->GetRenderWindow()->Render();

      sleep(5);
      viewer->saveScreenshot(snapshotBrowsePath);

      viewer->removeAllPointClouds();
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBL> rgb(sourceCloud);
      viewer->addPointCloud<pcl::PointXYZRGBL>(sourceCloud,rgb,cloud_id);
          
      pccset->ui->widget_gt->update();
      pccset->ui->widget_sample->update();
    }
    QCoreApplication::processEvents();
}

void ToolOperations::Mouse(const pcl::visualization::MouseEvent &event ,pcl::visualization::PCLVisualizer::Ptr viewer)
{
    if(flagSynchoniser==true){
      if ((event.getButton() == pcl::visualization::MouseEvent::LeftButton && event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease) ||
          (event.getButton() == pcl::visualization::MouseEvent::MiddleButton && event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease) || 
          event.getType() == pcl::visualization::MouseEvent::MouseScrollDown || event.getType() == pcl::visualization::MouseEvent::MouseScrollUp){
      
          pcl::visualization::Camera cameraParameters;
          viewer->getCameraParameters (cameraParameters);

          const char *s =viewer->getRenderWindow()->GetWindowName();
          string source(s);

          if (source.find("GT")!=string::npos){
            pccset->sampleScreen->setCameraParameters(cameraParameters);
            pccset->ui->widget_sample->update();
          }
          
          if (source.find("Sample")!=string::npos){
            pccset->gtScreen->setCameraParameters(cameraParameters);
            pccset->ui->widget_gt->update();
          }
      }
    }
}

void ToolOperations::PointPicking(const pcl::visualization::PointPickingEvent& event,pcl::visualization::PCLVisualizer::Ptr viewer)
{
    const char *s =viewer->getRenderWindow()->GetWindowName();
    string source(s);
    int index=event.getPointIndex();

    string info="\n"+source+" point taken\n";
    pccset->ui->infoTextEdit->insertPlainText(QString::fromStdString(info));

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
    set<int> labels; 
    uint32_t sourceRGB;

    if (source.find("GT")!=string::npos){
      sourceCloud=pccset->cloud_groundtruth;
      labels=pccset->gtLabels;
      sourceRGB=gtRGB;
    }
    if (source.find("Sample")!=string::npos){
      sourceCloud=pccset->cloud_sample;
      labels=pccset->sampleLabels;
      sourceRGB=sampleRGB;
    }

    uint32_t currentRGB=*reinterpret_cast<int*>(&(*sourceCloud)[index].rgb);
    int position=distance(labels.begin(), labels.find(sourceCloud->points[index].label));

    shared_ptr<UndoRedoBase> cAction(new ColorizeAction(pccset, position,currentRGB,sourceRGB,source)); // create command for colorize

    if (source.find("GT")!=string::npos){
      pccset->gtEO->executeCmd(cAction);
      pccset->ui->b_gtUndo->setEnabled(true);
      pccset->ui->m_gtUndo->setEnabled(true);
    }
      
    if (source.find("Sample")!=string::npos){
      pccset->sampleEO->executeCmd(cAction);
      pccset->ui->b_sampleUndo->setEnabled(true);
      pccset->ui->m_sampleUndo->setEnabled(true);
    }

}

void ToolOperations::colorize(int pos,uint32_t color,const string & source)
{
    if (source.find("GT")!=string::npos){

      for (auto gtItr = pccset->gtSetVector[pos].begin(); gtItr != pccset->gtSetVector[pos].end(); gtItr++)
        (*pccset->cloud_groundtruth)[ *gtItr ].rgb=*reinterpret_cast<float*>(&color);

      pccset->gtScreen->removeAllPointClouds();
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBL> rgb(pccset->cloud_groundtruth);
      pccset->gtScreen->addPointCloud<pcl::PointXYZRGBL>(pccset->cloud_groundtruth,rgb,"cloud_groundtruth");
      pccset->ui->widget_gt->update();

    }

    if (source.find("Sample")!=string::npos){

      pccset->eO->sColorVector[pos]=color;

      for (auto sampleItr = pccset->sampleSetVector[pos].begin(); sampleItr != pccset->sampleSetVector[pos].end(); sampleItr++)
        (*pccset->cloud_sample)[ *sampleItr ].rgb=*reinterpret_cast<float*>(&color);

      pccset->sampleScreen->removeAllPointClouds();
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBL> rgb(pccset->cloud_sample);
      pccset->sampleScreen->addPointCloud<pcl::PointXYZRGBL>(pccset->cloud_sample,rgb,"cloud_sample");
      pccset->ui->widget_sample->update();

    }
}

void ToolOperations::snapshotDialogOpened()
{
    if(sD->ui->gtRadio->isChecked()){
      pccset->ui->widget_gt->update();
      pccset->ui->widget_gt->GetInteractor()->Render();
      pccset->ui->widget_gt->GetRenderWindow()->Render();
      sleep(5);
      pccset->gtScreen->saveScreenshot(snapshotBrowsePath);
    }
    else if(sD->ui->sampleRadio->isChecked()){
      pccset->ui->widget_sample->update();
      pccset->ui->widget_sample->GetInteractor()->Render();
      pccset->ui->widget_sample->GetRenderWindow()->Render();
      sleep(5);
      pccset->sampleScreen->saveScreenshot(snapshotBrowsePath);
    }
    else if(sD->ui->spesificAreaRadio->isChecked())
      QMessageBox::StandardButton info= QMessageBox::information(pccset, "Important!", "Hit 'X' on the screens and pick the area you want!");
}

void ToolOperations::snapshotBrowseButtonClicked()
{
    QString filename = QFileDialog::getSaveFileName(pccset, "Save", snapshotCurrentPath, "Images (*.png *.xpm *.jpg)",nullptr,QFileDialog::DontUseNativeDialog);
    snapshotBrowsePath = filename.toStdString() + ".png";
    string path=snapshotBrowsePath;

    int found = path.find_last_of("/");

    path.erase(path.begin()+found,path.end());
    snapshotCurrentPath = QString::fromStdString(path);

    if(sD->ui->folders->findText(snapshotCurrentPath) == -1){
      sD->ui->folders->insertItem(0,snapshotCurrentPath);
      sD->ui->folders->setCurrentText(snapshotCurrentPath);
    }
}

void ToolOperations::excelDialogOpened()
{
    string sampleString = eD->ui->sampleNameEdit->text().toStdString();
    string methodString = eD->ui->methodNameEdit->text().toStdString();
    string elapsedString = eD->ui->elapsedTimeEdit->text().toStdString();
    string timeUnitString;

    if(eD->ui->timeUnitCombo->currentText() == "miliseconds")
      timeUnitString = "ms";
    else if(eD->ui->timeUnitCombo->currentText() == "seconds")
      timeUnitString = "s";
    else if(eD->ui->timeUnitCombo->currentText() == "minutes")
      timeUnitString = "m";  

    creatingSheet(sampleString,methodString,elapsedString,timeUnitString);

    if(eD->ui->fixParamaters->isChecked()){
      eD->ui->methodNameEdit->setEnabled(false);
      eD->ui->timeUnitCombo->setEnabled(false);
      eD->ui->folders->setEnabled(false);
      eD->ui->browseButton->setEnabled(false);
    }
    else{
      eD->ui->methodNameEdit->setEnabled(true);
      eD->ui->timeUnitCombo->setEnabled(true);
      eD->ui->folders->setEnabled(true);
      eD->ui->browseButton->setEnabled(true);
    }
}

void ToolOperations::excelBrowseButtonClicked()
{
    QString filename = QFileDialog::getSaveFileName(pccset, "Save", excelCurrentPath, "Excel Files (*.xlsx *.xlsm *.xls *.xltm)",nullptr,QFileDialog::DontUseNativeDialog);
    excelBrowsePath = filename.toStdString() + ".xlsx";
    string path = excelBrowsePath;
    
    int found = path.find_last_of("/");

    path.erase(path.begin()+found,path.end());
    excelCurrentPath = QString::fromStdString(path);

    if(eD->ui->folders->findText(excelCurrentPath) == -1){
      eD->ui->folders->insertItem(0,excelCurrentPath);
      eD->ui->folders->setCurrentText(excelCurrentPath);
    }
}

void ToolOperations::creatingSheet(string sampleString,string methodString,string elapsedString,string timeUnitString)
{ 
    xlnt::workbook wb;
    xlnt::worksheet ws = wb.active_sheet();

    ws.cell("A2").value("Sample Name");
    ws.cell("B2").value("Method Name");
    ws.cell("C2").value("Elapsed Time (" + timeUnitString + ")");
    ws.cell("E2").value("Precision");
    ws.cell("F2").value("Recall");
    ws.cell("G2").value("F1");
    ws.cell("H2").value("IoU");
    ws.cell("I2").value("MIoU");
    ws.cell("J2").value("Accuracy");

    string sampleNameString = "A3:A" + to_string(pccset->gtLabels.size()+2);
    string methodNameString = "B3:B" + to_string(pccset->gtLabels.size()+2);
    string elapsedTimeString = "C3:C" + to_string(pccset->gtLabels.size()+2);
    string miouString = "I3:I" + to_string(pccset->gtLabels.size()+2);
    string accuracyString = "J3:J" + to_string(pccset->gtLabels.size()+2);

    int labelID = 0;
    for(set<int>::iterator it=pccset->gtLabels.begin();it!=pccset->gtLabels.end();++it)
    {
      string labelString = "Label " + to_string(*it);
      string labelCellString = "D" + to_string(labelID+3);
      ws.cell(labelCellString).value(labelString);
      ++labelID;
    }

    for(int j=0;j<pccset->gtLabels.size();++j){
      
      string precisionValueString = to_string_excel(pccset->metricResults[j][0]);
      string recallValueString = to_string_excel(pccset->metricResults[j][1]);
      string f1ValueString = to_string_excel(pccset->metricResults[j][2]);
      string iouValueString = to_string_excel(pccset->metricResults[j][3]);

      string precisionString = "E" + to_string(j+3);
      string recallString = "F" + to_string(j+3);
      string f1String = "G" + to_string(j+3);
      string iouString = "H" + to_string(j+3);

      ws.cell(precisionString).value(precisionValueString);
      ws.cell(recallString).value(recallValueString);
      ws.cell(f1String).value(f1ValueString);
      ws.cell(iouString).value(iouValueString);
    }

    ws.merge_cells(sampleNameString);
    ws.merge_cells(methodNameString);
    ws.merge_cells(elapsedTimeString);
    ws.merge_cells(miouString);
    ws.merge_cells(accuracyString);

    string miouValueString = to_string_excel(pccset->metricResults[pccset->gtLabels.size()][3]);
    string accuracyValueString = to_string_excel(pccset->metricResults[pccset->gtLabels.size()][0]);

    ws.cell("A3").value(sampleString);
    ws.cell("B3").value(methodString);
    ws.cell("C3").value(elapsedString);
    ws.cell("I3").value(miouValueString);
    ws.cell("J3").value(accuracyValueString);

    ws.cell("A3").alignment(xlnt::alignment().vertical(xlnt::vertical_alignment::center).horizontal(xlnt::horizontal_alignment::center));
    ws.cell("B3").alignment(xlnt::alignment().vertical(xlnt::vertical_alignment::center).horizontal(xlnt::horizontal_alignment::center));
    ws.cell("C3").alignment(xlnt::alignment().vertical(xlnt::vertical_alignment::center).horizontal(xlnt::horizontal_alignment::center));
    ws.cell("I3").alignment(xlnt::alignment().vertical(xlnt::vertical_alignment::center).horizontal(xlnt::horizontal_alignment::center));
    ws.cell("J3").alignment(xlnt::alignment().vertical(xlnt::vertical_alignment::center).horizontal(xlnt::horizontal_alignment::center));

    wb.save(excelBrowsePath);

}

string ToolOperations::to_string_excel(double value)
{
    std::ostringstream out;
    out << setw(6)<< setprecision(4)<< value;
    return out.str();
}




