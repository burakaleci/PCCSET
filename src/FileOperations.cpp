
// Standard
#include <iostream>
#include <string>
#include <set>
#include <vector>
#include <typeinfo>
#include <fstream>
#include <algorithm>    
#include <chrono>

// PCL
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/file_io.h>
#include <pcl/common/io.h>

// QT
#include <QMainWindow>
#include <QFileDialog>
#include <QMessageBox>
#include <QApplication>
#include <QProgressDialog>
#include <QThread>
#include <QStandardPaths>

// User defined
#include "FileOperations.h"

using namespace std;

FileOperations::FileOperations(PCCSET *p)
    : pccset(p)
{
    gtFileName = QString(QStandardPaths::HomeLocation).toStdString();
    sampleFileName = QString(QStandardPaths::HomeLocation).toStdString();
    source = "GT";
    cloud = pcl::PointCloud<pcl::PointXYZRGBL>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBL>);
}

FileOperations::~FileOperations(void)
{
    
}

void FileOperations::fileOpen(const string & src)
{
    
    QString fileName = QFileDialog::getOpenFileName(pccset, QObject::tr("Open File"),"/home",QObject::tr("Pcd Files (*.pcd);;Text Files(*.txt)"),nullptr,QFileDialog::DontUseNativeDialog);
    source = src;
    bool fileOpened=false;

    QProgressDialog *progressDialog= new QProgressDialog("Reading data",nullptr,0,3,pccset);
    progressDialog->setWindowTitle("Opening file");
    progressDialog->setMinimumDuration(0);
    
    if (source.find("GT")!=string::npos){
      progressDialog->setRange(0,3);
      progressDialog->move(pccset->ui->widget_gt->width()/2,pccset->ui->widget_gt->height()/3);
    }
    if (source.find("Sample")!=string::npos){
      progressDialog->setRange(0,4);
      progressDialog->move(pccset->ui->widget_gt->width()+pccset->ui->widget_sample->width()/2,pccset->ui->widget_gt->height()/3);
    }

    progressDialog->show();
    QThread::sleep(1); 
    QCoreApplication::processEvents();
    
    
    if (source.find("GT")!=string::npos)
      gtFileName=fileName.toStdString();
  
    if (source.find("Sample")!=string::npos)
      sampleFileName=fileName.toStdString();

    if (fileName.toStdString().find(".txt")!=string::npos)
      fileOpened=fileOpenTXT();
  
    if (fileName.toStdString().find(".pcd")!=string::npos)
      fileOpened=fileOpenPCD(); 

    if (fileOpened){
      if (source.find("GT")!=string::npos){

        progressDialog->setValue(1);
        progressDialog->setLabelText("Analyze data");
        QCoreApplication::processEvents();
        
        analyzeAndSeparateData();

        progressDialog->setValue(2);
        progressDialog->setLabelText("Visualize data");
        QCoreApplication::processEvents();
        
        printInitialInfo();
        initializeGTAndButtons();
      }
      
      if (source.find("Sample")!=string::npos){

        QMessageBox::StandardButton reply= QMessageBox::question(pccset, "Arrange Sample Data", "PCCSET uses point indices. If GT and Sample points  do not have the same order, please apply rearrangement step !!!",QMessageBox::Apply|QMessageBox::Abort);

        if (reply==QMessageBox::Apply){
          progressDialog->setValue(1);
          progressDialog->setLabelText("Arrange Sample data");
          QCoreApplication::processEvents();

          arrangeSampleData();
        }

        progressDialog->setValue(2);
        progressDialog->setLabelText("Analyze data");
        QCoreApplication::processEvents();
        analyzeAndSeparateData();

        progressDialog->setValue(3);
        progressDialog->setLabelText("Visualize data");
        QCoreApplication::processEvents();

        printInitialInfo();
        initializeSampleAndButtons();
      }
      
    }
    else
      QMessageBox::warning(pccset, "Error!", "Couldn't read the file!");

    delete progressDialog;
}

void FileOperations::fileSave(const string &src)
{
    QMessageBox::StandardButton reply= QMessageBox::question(pccset, "Saving PCD file", "PCD File will be saved as in XYZRGBL Type over the existing file!!!",QMessageBox::Apply|QMessageBox::Abort);

    if (reply==QMessageBox::Apply){

      if (src.find("GT")!=string::npos){
        gtFileName.replace(gtFileName.end()-4,gtFileName.end(),".pcd");
        pcl::io::savePCDFile(gtFileName, *pccset->cloud_groundtruth);
      }
        
      if (src.find("Sample")!=string::npos){
        sampleFileName.replace(sampleFileName.end()-4,sampleFileName.end(),".pcd");
        pcl::io::savePCDFile(sampleFileName, *pccset->cloud_sample);
      }
    }  
}

void FileOperations::fileSaveAs(const string &src)
{
    QString fileName = QFileDialog::getSaveFileName(pccset, "Save", "/home", "Pcd Files (*.pcd)",nullptr,QFileDialog::DontUseNativeDialog);
    string filePath = fileName.toStdString() + ".pcd";

    if (src.find("GT")!=string::npos)
      pcl::io::savePCDFile(filePath,*pccset->cloud_groundtruth);
        
    if (src.find("Sample")!=string::npos)
      pcl::io::savePCDFile(filePath,*pccset->cloud_sample);   
}

void FileOperations::fileExportAs(const string &src)
{
    QString fileName = QFileDialog::getSaveFileName(pccset, "Save", "/home", "Images (*.png *.xpm *.jpg)",nullptr,QFileDialog::DontUseNativeDialog);
    string filePath = fileName.toStdString() + ".png";

    if (src.find("GT")!=string::npos)
      pccset->gtScreen->saveScreenshot(filePath);
       
    if (src.find("Sample")!=string::npos)
      pccset->sampleScreen->saveScreenshot(filePath);
    
        
}

void FileOperations::appClose(void)
{
    QMessageBox::StandardButton reply = QMessageBox::question(pccset, "Are you sure?", "Do you want to save changes on test sample?",QMessageBox::Yes|QMessageBox::No);
    if (reply == QMessageBox::Yes) {
      try{
        if(pccset->cloud_sample->size()==0)
          throw pccset->cloud_sample->size();
        fileSave("Sample");
      }
      catch(...){
        QMessageBox::warning(pccset, "Error!", "There is no sample yet.");
      }
    }
    QApplication::quit();
}

bool FileOperations::fileOpenTXT(void)
{
    QMessageBox::StandardButton reply = QMessageBox::question(pccset, "Reading txt file", "Txt File should contain only x, y, and z coordinates!",QMessageBox::Apply|QMessageBox::Abort);

    if (reply==QMessageBox::Apply){
      if(readPointFile()){
        if (mergePointsAndLabels()){
          swapPointClouds();
          return true;
        }
      }
    }
    
    return false;
}

bool FileOperations::fileOpenPCD(void)
{
    pcl::PCLPointCloud2 pcdFileHeader;
    pcl::PCDReader reader;
    string fileName;

    cloud->clear();

    if (source.find("GT")!=string::npos)
      fileName=gtFileName;
    if (source.find("Sample")!=string::npos)
      fileName=sampleFileName;

    if (reader.read(fileName, *cloud,0) == -1)
      return false;
    
    if (reader.readHeader(fileName, pcdFileHeader) == -1)
      return false;

    const string fieldNames=pcl::getFieldsList(pcdFileHeader);
    
    if (fieldNames.find("rgb")==string::npos){

      QMessageBox::warning(pccset, "Warning!", "Initialize rgba field with white color");
      uint8_t r = 255,g=255,b=255;
      uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
      for(auto &p: cloud->points) 
        p.rgb=*reinterpret_cast<float*>(&rgb);

    }
      
    if (fieldNames.find("label")==string::npos)
      if (!mergePointsAndLabels())
        return false;
    
    swapPointClouds();
    return true;
}

bool FileOperations::readLabelFile(const string & fileName,vector < int > & l)
{
    int label;
    string data;

    ifstream myLabelFile;
    myLabelFile.open(fileName);

    if(myLabelFile.fail())// checks to see if file opended 
      return false;

    while (true) {
      
      myLabelFile >> data;
      if (!(istringstream(data) >> label >> ws).eof())  // checks to see if the data is valid 
        return false;

      if( !myLabelFile.eof())   // checks to see if the data is valid
        l.push_back(label);
      else
          break; 
    }
    
    myLabelFile.close(); 
    return true;
}

bool FileOperations::readPointFile(void)
{
    pcl::PointXYZRGBL p;
    double pC[3]={};
    string data;
    string fileName;

    cloud->clear();

    if (source.find("GT")!=string::npos)
      fileName=gtFileName;
    if (source.find("Sample")!=string::npos)
      fileName=sampleFileName;

    ifstream myPointFile;
    myPointFile.open(fileName);

    if(myPointFile.fail()) // checks to see if file opended 
      return false;
    else {

      while (true) {
      
        for(int i=0;i<3;i++){
          myPointFile >> data;
          if (!(istringstream(data) >> pC[i] >> ws).eof())  // checks to see if the data is valid 
            return false;
        }
        
        if( !myPointFile.eof() ) {
          p.x=pC[0];
          p.y=pC[1];
          p.z=pC[2];

          cloud->push_back(p);
        }
        else 
          break;
          
      }
        
      myPointFile.close();

      QMessageBox::warning(pccset, "Warning!", "Initialize rgba field with white color");

      uint8_t r = 255,g=255,b=255;
      uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
      for(auto &p: cloud->points)
        p.rgb=*reinterpret_cast<float*>(&rgb);

      return true;
    }
}

bool FileOperations::mergePointsAndLabels(void)
{
    QMessageBox::warning(pccset, "Warning!", "The point cloud must include Label field. You can open label txt.");
    QString label_txt=QFileDialog::getOpenFileName(pccset, QObject::tr("Open File"),"/home",QObject::tr("Txt Files (*.txt)"),nullptr,QFileDialog::DontUseNativeDialog);

    vector < int > l;
    if (!readLabelFile(label_txt.toStdString(),l))
      return false;

    if (l.size()!=cloud->points.size()){
      QMessageBox::warning(pccset, "Error!", "Number of point and label does not match!");
      return false;
    }
    else{
      for (int i=0;i<l.size();i++)
        cloud->points[i].label=l[i];
    }
    return true;
}

void FileOperations::swapPointClouds(void)
{
    if (source.find("GT")!=string::npos)
      pcl::copyPointCloud(*cloud,*pccset->cloud_groundtruth);
    if (source.find("Sample")!=string::npos)
      pcl::copyPointCloud(*cloud,*pccset->cloud_sample);
}

void FileOperations::arrangeSampleData(void)
{
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr arranged (new pcl::PointCloud<pcl::PointXYZRGBL>);
    pcl::KdTreeFLANN<pcl::PointXYZRGBL> kdtreeSample;
    kdtreeSample.setInputCloud (pccset->cloud_sample);

    int K=1;
    for(int i=0;i<pccset->cloud_groundtruth->points.size();i++){

      vector<int> pointIdxNKNSearch(K);
      vector<float> pointNKNSquaredDistance(K);

      if ( kdtreeSample.nearestKSearch (pccset->cloud_groundtruth->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        arranged->push_back(pccset->cloud_sample->points[pointIdxNKNSearch[0]]);

    }

    pccset->cloud_sample.swap (arranged);
}

void FileOperations::analyzeAndSeparateData(void)
{
    set<int> labels; 
    vector < set< int > > setVector;
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr sourceCloud (new pcl::PointCloud<pcl::PointXYZRGBL>);

    if (source.find("GT")!=string::npos)
      sourceCloud=pccset->cloud_groundtruth;
    if (source.find("Sample")!=string::npos)
      sourceCloud=pccset->cloud_sample;

    // First determine label ids
    for(int i=0;i<sourceCloud->points.size();i++)
      labels.insert(sourceCloud->points[i].label);

    // Separates the segments into set or point cloud depending on data type. 
    for (auto labelsItr = labels.begin(); labelsItr != labels.end(); labelsItr++){
      set< int > labelIDs;
      for(int i=0;i<sourceCloud->points.size();i++){
        if (*labelsItr==sourceCloud->points[i].label)
          labelIDs.insert(i);
      }

      setVector.emplace_back(labelIDs.begin(),labelIDs.end());
    }

    if (source.find("GT")!=string::npos){
      pccset->gtLabels=labels;
      pccset->gtSetVector=setVector;
    }
    if (source.find("Sample")!=string::npos){
      pccset->sampleLabels=labels;
      pccset->sampleSetVector=setVector;
    }  
}

void FileOperations::initializeGTAndButtons(void)
{
    pccset->ui->b_gtSave->setEnabled(true);
    pccset->ui->m_gtSave->setEnabled(true);
    pccset->ui->b_gtSaveAs->setEnabled(true);
    pccset->ui->m_gtSaveAs->setEnabled(true);
    pccset->ui->b_gtExportAs->setEnabled(true);
    pccset->ui->m_gtExportAs->setEnabled(true);

    pccset->ui->b_gtClear->setEnabled(true);
    pccset->ui->m_gtClear->setEnabled(true);

    pccset->ui->m_increasePointSize->setEnabled(true);
    pccset->ui->m_decreasePointSize->setEnabled(true);
    pccset->ui->m_resetPointSize->setEnabled(true);

    pccset->ui->b_gtColorize->setEnabled(true);
    pccset->ui->m_gtColorize->setEnabled(true);

    pccset->ui->b_sampleOpen->setEnabled(true);
    pccset->ui->m_sampleOpen->setEnabled(true);
    
    // Set first dimensions of TP,FN,FP
    pccset->TP.resize(pccset->gtLabels.size());
    pccset->FN.resize(pccset->gtLabels.size());
    pccset->FP.resize(pccset->gtLabels.size());
    pccset->metricResults.resize(pccset->gtLabels.size() + 1,vector<double>(4,0));

    pccset->gtScreen->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBL> rgb(pccset->cloud_groundtruth);
    pccset->gtScreen->addPointCloud<pcl::PointXYZRGBL>(pccset->cloud_groundtruth,rgb,"cloud_groundtruth");
    pccset->gtScreen->setCameraPosition(0, 30, 0,    0, 0, 0,   0, 0, 1);
}
    
void FileOperations::initializeSampleAndButtons(void)
{
    pccset->ui->b_sampleSave->setEnabled(true);
    pccset->ui->m_sampleSave->setEnabled(true);
    pccset->ui->b_sampleSaveAs->setEnabled(true);
    pccset->ui->m_sampleSaveAs->setEnabled(true);
    pccset->ui->b_sampleExportAs->setEnabled(true);
    pccset->ui->m_sampleExportAs->setEnabled(true);

    pccset->ui->b_sampleClear->setEnabled(true);
    pccset->ui->m_sampleClear->setEnabled(true);

    pccset->ui->b_background->setEnabled(true);
    pccset->ui->m_background->setEnabled(true);
    pccset->ui->b_snapshot->setEnabled(true);
    pccset->ui->m_snapshot->setEnabled(true);
    pccset->ui->b_synchroniser->setEnabled(true);
    pccset->ui->m_synchroniser->setEnabled(true);
    pccset->ui->b_sampleColorize->setEnabled(true);
    pccset->ui->m_sampleColorize->setEnabled(true);

    pccset->ui->b_calculateMetrics->setEnabled(true);
    pccset->ui->m_calculateMetrics->setEnabled(true);
    pccset->ui->b_automaticPair->setEnabled(true);
    pccset->ui->m_automaticPair->setEnabled(true);
    pccset->ui->m_manualPair->setEnabled(true);
    pccset->ui->b_manualPair->setEnabled(true);

    pccset->eO->sColorVector.resize(pccset->sampleLabels.size());

    for (int i=0;i<pccset->sampleLabels.size();i++)
        pccset->eO->sColorVector[i] = *reinterpret_cast<int*>(&(*pccset->cloud_sample)[*(pccset->sampleSetVector[i].begin())].rgb);

    pccset->sampleScreen->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBL> rgbXYZ(pccset->cloud_sample);
    pccset->sampleScreen->addPointCloud<pcl::PointXYZRGBL>(pccset->cloud_sample,rgbXYZ,"cloud_sample");
    pccset->sampleScreen->setCameraPosition(0, 30, 0,    0, 0, 0,   0, 0, 1);
}
    
void FileOperations::printInitialInfo(void)
{
    set<int> labels; 
    vector < set< int > > setVector;
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr sourceCloud (new pcl::PointCloud<pcl::PointXYZRGBL>);

    if (source.find("GT")!=string::npos){
      sourceCloud=pccset->cloud_groundtruth;
      labels=pccset->gtLabels;
      setVector=pccset->gtSetVector;
    }
      
    if (source.find("Sample")!=string::npos){
      sourceCloud=pccset->cloud_sample;
      labels=pccset->sampleLabels;
      setVector=pccset->sampleSetVector;
    }
      
    string s="NOP for " +source+ " " +to_string(sourceCloud->points.size())+"\n";

    s = s+source+ " Label IDs:\n";
    for (auto lItr = labels.begin(); lItr != labels.end(); lItr++)
      s=s+to_string(*lItr)+" ";

    s=s+"\n";
    int i=0;
    for (auto lItr = labels.begin(); lItr != labels.end(); lItr++,i++)
      s=s+to_string(*lItr)+": "+to_string(setVector[i].size())+"\n";

    s=s+"\n\n"; 
    
    pccset->ui->infoTextEdit->insertPlainText(QString::fromStdString(s));
}

