
// Standard
#include <iostream>

// PCL
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/features/moment_of_inertia_estimation.h>

// QT
#include <QMainWindow>
#include <QMessageBox>
#include <QProgressDialog>
#include <QThread>

// User defined
#include "EvaluateOperations.h"

using namespace std;

EvaluateOperations::EvaluateOperations(PCCSET *p)
    : pccset(p)
{
    NOM=0;
    NOManual=0;
    flagGTTaken=false;
    flagSampleTaken=false; 
}

EvaluateOperations::~EvaluateOperations(void)
{
    
}

void EvaluateOperations::calculateMetrics(void)
{
    shared_ptr<UndoRedoBase> cAction(new CalculateMetricsAction(pccset)); // create command for classification metrics
    pccset->sampleEO->executeCmd(cAction);
}

void EvaluateOperations::calculateClassicationMetrics(void)
{
    initializeOperation();

    for (int i=0;i<pccset->gtLabels.size();i++){
        calculateClassMetrics(i,i);
        printClassInfo(i,i);
    }
        
    calculateOverallMetrics();
    printOverallInfo();
    updateMenusAndButtons("Classification",true);
    
    QMessageBox::StandardButton reply = QMessageBox::question(pccset, "Test", "Do you want to colorize sample with to same colors of GT?",QMessageBox::Yes|QMessageBox::No);

    if(reply==QMessageBox::Yes)
        colorizeSampleWithGTColors();
}

void EvaluateOperations::undoClassicationMetrics(void)
{
    clearMetrics();
    updateMenusAndButtons("Classification",false);
    
    vector <uint32_t> colorVector;
    colorVector.resize(pccset->sampleLabels.size());

    colorVector=sampleColorVectorStack.top();
    sampleColorVectorStack.pop();

    for (int i=0;i<pccset->sampleLabels.size();i++)
        pccset->tO->colorize(i,colorVector[i],"Sample");

}

void EvaluateOperations::automaticPair(void)
{
    shared_ptr<UndoRedoBase> aAction(new AutomaticPairAction(pccset)); // create command for automatic pair
    pccset->sampleEO->executeCmd(aAction);
}

void EvaluateOperations::findPairSegments(void)
{
    initializeOperation();

    int pointThreshold=pccset->cloud_groundtruth->points.size()/10;  
    NOM=0;
    auto lItr=pccset->gtLabels.begin();
    QProgressDialog *progressDialog= new QProgressDialog("Searching for GT 0",nullptr,0,pccset->gtSetVector.size(),pccset);
    progressDialog->setWindowTitle("Searching pair segments");
    progressDialog->setMinimumDuration(0);
    progressDialog->setGeometry(pccset->ui->widget_gt->width()+pccset->ui->widget_sample->width()/2,pccset->ui->widget_gt->height()/3,300,75);
    progressDialog->show();
    QThread::sleep(1); 
    QCoreApplication::processEvents();

    for (int i=0;i<pccset->gtSetVector.size();i++){

        progressDialog->setValue(i);
        string s="Searching for GT " + to_string(*lItr);
        progressDialog->setLabelText(QString::fromStdString(s));
        QCoreApplication::processEvents();

        if (checkSegmentedLabels("GT",i)){

            for (int j=0;j<pccset->sampleSetVector.size();j++){
                
                if (checkSegmentedLabels("Sample",j)){

                    int a=pccset->gtSetVector[i].size()-pccset->sampleSetVector[j].size();

                    if (abs(a) < pointThreshold){
                        
                        if (isSegmentsMatched(i,j)){
                        
                            processSegment(i,j);
                            break;
                        }
                    }
                }
            }
        }
        lItr++;
    }
  
    delete progressDialog;
    makeDecision();

}

void EvaluateOperations::undoAutomaticPair(void)
{
    clearMetrics();
    updateMenusAndButtons("Segmentation",false);
    
    segmentedGt.clear();
    segmentedSample.clear();

    vector <uint32_t> colorVector;
    colorVector.resize(pccset->sampleLabels.size());

    colorVector=sampleColorVectorStack.top();
    sampleColorVectorStack.pop();

    for (int i=0;i<pccset->sampleLabels.size();i++)
        pccset->tO->colorize(i,colorVector[i],"Sample");
 
}

void EvaluateOperations::manualPair(void)
{
    QMessageBox::StandardButton pickPoint= QMessageBox::information(pccset, "Pick a Point", "Use SHift + Left click when you pick.");
}

bool EvaluateOperations::PointPicking(const pcl::visualization::PointPickingEvent& event,pcl::visualization::PCLVisualizer::Ptr viewer)
{
    const char *s =viewer->getRenderWindow()->GetWindowName();
    string source(s);
    
    if (source.find("GT")!=string::npos){
        gtPosition=distance(pccset->gtLabels.begin(), pccset->gtLabels.find(pccset->cloud_groundtruth->points[event.getPointIndex()].label));
        if (checkSegmentedLabels(source,gtPosition)){
            string info="\n"+source+" point taken\n";
            pccset->ui->infoTextEdit->insertPlainText(QString::fromStdString(info));
            flagGTTaken=true;
        }   
        else
            QMessageBox::StandardButton pickPoint= QMessageBox::information( nullptr, "Pick a Point", "GT segment already evaluated. Please pick another segment.");
    }
    if (source.find("Sample")!=string::npos){
        samplePosition=distance(pccset->sampleLabels.begin(),pccset->sampleLabels.find(pccset->cloud_sample->points[event.getPointIndex()].label));
        if (checkSegmentedLabels(source,samplePosition)){
            string info="\n"+source+" point taken\n";
            pccset->ui->infoTextEdit->insertPlainText(QString::fromStdString(info));
            flagSampleTaken=true;
        }  
        else
            QMessageBox::StandardButton pickPoint= QMessageBox::information(pccset, "Pick a Point", "Sample segment already evaluated. Please pick another segment.");
    }

    if (flagGTTaken==true && flagSampleTaken==true){

        shared_ptr<UndoRedoBase> mAction(new ManualPairAction(pccset,gtPosition,samplePosition)); // create command for classification metrics
        pccset->sampleEO->executeCmd(mAction);

        flagGTTaken=false;
        flagSampleTaken=false;

        return true;
    } 
    
    return false;
}

void EvaluateOperations::pairSegments(int gP, int sP)
{
    initializeOperation();
    NOManual++;
    segmentedGt.push_back(gP);
    segmentedSample.push_back(sP);
    processSegment(gP,sP);
    makeDecision();
}

void EvaluateOperations::undoManualPair(int sP)
{
    NOM--;
    NOManual--;
    pccset->TP.resize( pccset->gtLabels.size());
    pccset->FN.resize( pccset->gtLabels.size());
    pccset->FP.resize( pccset->gtLabels.size());

    pccset->metricResults[sP][0]=0;
    pccset->metricResults[sP][1]=0;
    pccset->metricResults[sP][2]=0;
    pccset->metricResults[sP][3]=0;

    segmentedGt.pop_back();
    segmentedSample.pop_back();
    
    vector <uint32_t> colorVector;
    colorVector.resize(pccset->sampleLabels.size());

    colorVector=sampleColorVectorStack.top();
    sampleColorVectorStack.pop();

    for (int i=0;i<pccset->sampleLabels.size();i++)
        pccset->tO->colorize(i,colorVector[i],"Sample");

    pccset->ui->b_excel->setEnabled(false);
    pccset->ui->m_excel->setEnabled(false);

    pccset->ui->m_errorMapClassification->setEnabled(false);
    pccset->ui->m_errorMapSegmentation->setEnabled(false);
    pccset->ui->b_errorMap->setEnabled(false);

    pccset->ui->b_manualPair->setEnabled(true);
    pccset->ui->m_manualPair->setEnabled(true); 

    if (NOManual==0 && NOM==0)
        updateMenusAndButtons("Segmentation",false);
   
}

void EvaluateOperations::errorMap(void)
{
    shared_ptr<UndoRedoBase> eAction(new ErrorMapAction(pccset)); 
    pccset->sampleEO->executeCmd(eAction); 
}

void EvaluateOperations::drawErrorMap(void)
{
    initializeOperation();
    
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr eM(new pcl::PointCloud<pcl::PointXYZRGBL>);
  
    uint8_t r = 0,g=0,b=255;
    uint32_t rgbCorrect = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    r = 255,g=0,b=0;
    uint32_t rgbInCorrect = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

    *eM=*eM+* pccset->cloud_sample;
  
    for(int i=0;i< pccset->cloud_sample->points.size();i++)
        eM->points[i].rgb =*reinterpret_cast<float*>(&rgbInCorrect);
  
    for (int i=0;i< pccset->TP.size();i++){
        for (set<int>::iterator it=  pccset->TP[i].begin(); it!= pccset->TP[i].end();++it){
            pcl::PointXYZRGBL p= pccset->cloud_sample->points[ *it ];
            p.rgb=*reinterpret_cast<float*>(&rgbCorrect); 
            eM->push_back(p);
        }     
    }

    pccset->ui->m_errorMapClassification->setEnabled(false);
    pccset->ui->m_errorMapSegmentation->setEnabled(false);
    pccset->ui->b_errorMap->setEnabled(false);

    pccset->sampleScreen->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBL> eMColor(eM);
    pccset->sampleScreen->addPointCloud<pcl::PointXYZRGBL>(eM,eMColor,"Error Map");
    pccset->ui->widget_sample->update();
}

void EvaluateOperations::undoErrorMap(void)
{
    vector <uint32_t> colorVector;
    colorVector.resize(pccset->sampleLabels.size());

    colorVector=sampleColorVectorStack.top();
    sampleColorVectorStack.pop();

    for (int i=0;i<pccset->sampleLabels.size();i++)
        pccset->tO->colorize(i,colorVector[i],"Sample");

}

void EvaluateOperations::initializeOperation(void)
{
    sampleColorVectorStack.push(sColorVector);

    pccset->ui->b_sampleUndo->setEnabled(true);
    pccset->ui->m_sampleUndo->setEnabled(true);
}

void EvaluateOperations::calculateClassMetrics(int gtIndex, int sampleIndex)
{
    set_intersection(pccset->gtSetVector[gtIndex].begin(), pccset->gtSetVector[gtIndex].end(), pccset->sampleSetVector[sampleIndex].begin(), pccset->sampleSetVector[sampleIndex].end(), inserter(pccset->TP[gtIndex],pccset->TP[gtIndex].begin()));
    set_difference(pccset->gtSetVector[gtIndex].begin(), pccset->gtSetVector[gtIndex].end(), pccset->sampleSetVector[sampleIndex].begin(), pccset->sampleSetVector[sampleIndex].end(), inserter(pccset->FN[gtIndex], pccset->FN[gtIndex].begin()));
    set_difference(pccset->sampleSetVector[sampleIndex].begin(), pccset->sampleSetVector[sampleIndex].end(), pccset->gtSetVector[gtIndex].begin(), pccset->gtSetVector[gtIndex].end(), inserter(pccset->FP[gtIndex], pccset->FP[gtIndex].begin()));

    pccset->metricResults[gtIndex][0]=pccset->TP[gtIndex].size()/(double)(pccset->TP[gtIndex].size()+pccset->FP[gtIndex].size());
    pccset->metricResults[gtIndex][1]=pccset->TP[gtIndex].size()/(double)(pccset->TP[gtIndex].size()+pccset->FN[gtIndex].size());
    pccset->metricResults[gtIndex][2]=2*(pccset->metricResults[gtIndex][0]*pccset->metricResults[gtIndex][1])/(pccset->metricResults[gtIndex][0]+pccset->metricResults[gtIndex][1]);
    pccset->metricResults[gtIndex][3]=pccset->TP[gtIndex].size()/(double)(pccset->TP[gtIndex].size()+pccset->FP[gtIndex].size()+pccset->FN[gtIndex].size());
  
    NOM++;
}

void EvaluateOperations::calculateOverallMetrics()
{
    int size=pccset->gtLabels.size();

    pccset->metricResults[size][0] = 0;
    pccset->metricResults[size][1] = 0;
    pccset->metricResults[size][2] = 0;
    pccset->metricResults[size][3] = 0;

    for (int i=0;i<size;i++){
        pccset->metricResults[size][0] = pccset->metricResults[size][0]+pccset->TP[i].size();
        pccset->metricResults[size][1] = pccset->metricResults[size][1]+pccset->metricResults[i][1];
        pccset->metricResults[size][2] = pccset->metricResults[size][2]+pccset->metricResults[i][2];
        pccset->metricResults[size][3] = pccset->metricResults[size][3]+pccset->metricResults[i][3];
    }

    pccset->metricResults[size][0] = pccset->metricResults[size][0]/pccset->cloud_sample->points.size();
    pccset->metricResults[size][1] = pccset->metricResults[size][1]/size;
    pccset->metricResults[size][2] = pccset->metricResults[size][2]/size;
    pccset->metricResults[size][3] = pccset->metricResults[size][3]/size;
}

void EvaluateOperations::printClassInfo(int gtIndex, int sampleIndex)
{
    int gtLabel=pccset->cloud_groundtruth->points[*(pccset->gtSetVector[gtIndex].begin())].label;
    int sampleLabel=pccset->cloud_sample->points[*(pccset->sampleSetVector[sampleIndex].begin())].label;

    string s="\nGT: "+to_string(gtLabel)+"-----> S: "+to_string(sampleLabel)+"\n";

    s=s+"TP: " +to_string(pccset->TP[gtIndex].size())+"\n";
    s=s+"FN: " +to_string(pccset->FN[gtIndex].size())+"\n";
    s=s+"FP: " +to_string(pccset->FP[gtIndex].size())+"\n";

    s = s+ to_string_with_format("Precision",pccset->metricResults[gtIndex][0]);
    s = s+ to_string_with_format("Recall",pccset->metricResults[gtIndex][1]);
    s = s+ to_string_with_format("F1",pccset->metricResults[gtIndex][2]);
    s = s+ to_string_with_format("IoU",pccset->metricResults[gtIndex][3]);

    pccset->ui->infoTextEdit->insertPlainText(QString::fromStdString(s));
}

void EvaluateOperations::printOverallInfo(void)
{
    string s =to_string_with_format("MIoU",pccset->metricResults[pccset->gtLabels.size()][3]);
    s = s+ to_string_with_format("Accuracy",pccset->metricResults[pccset->gtLabels.size()][0]);
    
    pccset->ui->infoTextEdit->insertPlainText(QString::fromStdString(s));
}

void EvaluateOperations::colorizeSampleWithGTColors(void)
{
    for (int i=0;i<pccset->gtLabels.size();i++)
        colorizeSegmentWithGTColor(i,i);
}

void EvaluateOperations::updateMenusAndButtons(string source, bool flag)
{
    pccset->ui->b_errorMap->setEnabled(flag);
    pccset->ui->b_excel->setEnabled(flag);
    pccset->ui->m_excel->setEnabled(flag);

    pccset->ui->b_calculateMetrics->setEnabled(!flag);
    pccset->ui->m_calculateMetrics->setEnabled(!flag);
    pccset->ui->b_automaticPair->setEnabled(!flag);
    pccset->ui->m_automaticPair->setEnabled(!flag);
    pccset->ui->b_manualPair->setEnabled(!flag);
    pccset->ui->m_manualPair->setEnabled(!flag);  

    if (source.find("Classification")!=string::npos)
        pccset->ui->m_errorMapClassification->setEnabled(flag);
    
    if (source.find("Segmentation")!=string::npos)
        pccset->ui->m_errorMapSegmentation->setEnabled(flag);
}

void EvaluateOperations::clearMetrics(void)
{
    NOM=0;
    pccset->TP.resize(pccset->gtLabels.size());
    pccset->FN.resize(pccset->gtLabels.size());
    pccset->FP.resize(pccset->gtLabels.size());

    for (int y = 0; y < pccset->metricResults.size(); y++){
        for (int x = 0; x < pccset->metricResults[y].size(); x++)
            pccset->metricResults[y][x] = 0;
    }
}

bool EvaluateOperations::checkSegmentedLabels(string source, int index)
{
    if (source.find("GT")!=string::npos){
        if (segmentedGt.size()>0 ){
            vector<int>::iterator itGT = std::find (segmentedGt.begin(), segmentedGt.end(), index);
            if (itGT != segmentedGt.end())
                return false;  
        }
    }

    if (source.find("Sample")!=string::npos){
        if (segmentedSample.size()>0){
            vector<int>::iterator itSample = std::find (segmentedSample.begin(), segmentedSample.end(), index);
            if (itSample != segmentedSample.end())
                return false;
        }
    }

    return true;
}

bool EvaluateOperations::isSegmentsMatched(int gtIndex, int sampleIndex)
{

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr g(new pcl::PointCloud<pcl::PointXYZRGBL>);
        
    for (auto gtItr = pccset->gtSetVector[gtIndex].begin(); gtItr != pccset->gtSetVector[gtIndex].end(); gtItr++)
        g->push_back(pccset->cloud_groundtruth->points[*gtItr]);

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr s(new pcl::PointCloud<pcl::PointXYZRGBL>);
        
    for (auto sampleItr = pccset->sampleSetVector[sampleIndex].begin(); sampleItr != pccset->sampleSetVector[sampleIndex].end(); sampleItr++)
            s->push_back(pccset->cloud_sample->points[*sampleItr]);   

    pcl::MomentOfInertiaEstimation <pcl::PointXYZRGBL> gt_FE;
    pcl::MomentOfInertiaEstimation <pcl::PointXYZRGBL> sample_FE;
  
    pcl::PointXYZRGBL gt_min_point_AABB;
    pcl::PointXYZRGBL gt_max_point_AABB;

    pcl::PointXYZRGBL sample_min_point_AABB;
    pcl::PointXYZRGBL sample_max_point_AABB;

    double BBThreshold=0.1;

    gt_FE.setInputCloud (g);
    gt_FE.compute();

    sample_FE.setInputCloud (s);
    sample_FE.compute ();

    gt_FE.getAABB (gt_min_point_AABB, gt_max_point_AABB);
    sample_FE.getAABB (sample_min_point_AABB,sample_max_point_AABB);

    double minX=fabs(gt_min_point_AABB.x-sample_min_point_AABB.x);
    double minY=fabs(gt_min_point_AABB.y-sample_min_point_AABB.y);
    double minZ=fabs(gt_min_point_AABB.z-sample_min_point_AABB.z);
    double maxX=fabs(gt_max_point_AABB.x-sample_max_point_AABB.x);
    double maxY=fabs(gt_max_point_AABB.y-sample_max_point_AABB.y);
    double maxZ=fabs(gt_max_point_AABB.z-sample_max_point_AABB.z);

    if (minX<BBThreshold && minY<BBThreshold && minZ<BBThreshold && maxX<BBThreshold && maxY<BBThreshold && maxZ<BBThreshold){
        pccset->sampleScreen->addCube(sample_min_point_AABB.x, sample_max_point_AABB.x, sample_min_point_AABB.y, sample_max_point_AABB.y, sample_min_point_AABB.z, sample_max_point_AABB.z, 1.0, 0.0, 0.0, to_string(s->points[0].label));
        pccset->sampleScreen->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, to_string(s->points[0].label));
        pccset->gtScreen->addCube (gt_min_point_AABB.x, gt_max_point_AABB.x, gt_min_point_AABB.y, gt_max_point_AABB.y, gt_min_point_AABB.z, gt_max_point_AABB.z, 1.0, 0.0, 0.0, to_string(g->points[0].label));
        pccset->gtScreen->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, to_string(g->points[0].label));
        pccset->ui->widget_gt->update();
        pccset->ui->widget_sample->update();
        return true;
    }
    else
        return false;
  
}

void EvaluateOperations::processSegment(int gtIndex, int sampleIndex)
{
    QMessageBox::StandardButton reply = QMessageBox::question(pccset, "Test", "Match Found!",QMessageBox::Apply|QMessageBox::Abort);

    if (reply==QMessageBox::Apply){

        segmentedGt.push_back(gtIndex);
        segmentedSample.push_back(sampleIndex);
        calculateClassMetrics(gtIndex,sampleIndex);
        printClassInfo(gtIndex,sampleIndex);
    
        QMessageBox::StandardButton reply2 = QMessageBox::question(pccset, "Test", "Do you want to colorize segment with to same colors of GT?",QMessageBox::Yes|QMessageBox::No);

        if(reply2==QMessageBox::Yes)
            colorizeSegmentWithGTColor(gtIndex,sampleIndex);
     
    }

    pccset->sampleScreen->removeAllShapes();
    pccset->gtScreen->removeAllShapes();
    pccset->ui->widget_gt->update();
    pccset->ui->widget_gt->GetInteractor()->Render();
    pccset->ui->widget_gt->GetRenderWindow()->Render();
    pccset->ui->widget_sample->update();
    pccset->ui->widget_sample->GetInteractor()->Render();
    pccset->ui->widget_sample->GetRenderWindow()->Render();

}

void EvaluateOperations::colorizeSegmentWithGTColor(int gtIndex, int sampleIndex)
{
    uint32_t rgbLabel = *reinterpret_cast<int*>(&(*pccset->cloud_groundtruth)[*(pccset->gtSetVector[gtIndex].begin())].rgb);
    sColorVector[sampleIndex]=rgbLabel;

    for (set<int>::iterator it= pccset->TP[gtIndex].begin(); it!=pccset->TP[gtIndex].end();++it)
        pccset->cloud_sample->points[ *it ].rgb=*reinterpret_cast<float*>(&rgbLabel);      

    pccset->sampleScreen->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBL> rgbXYZ(pccset->cloud_sample);
    pccset->sampleScreen->addPointCloud<pcl::PointXYZRGBL>(pccset->cloud_sample,rgbXYZ,"cloud_sample");
    pccset->ui->widget_sample->update();
}

void EvaluateOperations::makeDecision(void)
{
    if (NOM==pccset->gtLabels.size() || NOM==pccset->sampleLabels.size()-1){
        QMessageBox::warning(pccset, "Done", "All segments are evaluated.");
        calculateOverallMetrics();
        printOverallInfo();
        updateMenusAndButtons("Segmentation",true);
        
    }
    else{
        QMessageBox::warning(pccset, "Done", "Please pair remaining segments manually.");
        pccset->ui->b_automaticPair->setEnabled(false);
        pccset->ui->m_automaticPair->setEnabled(false);
        pccset->ui->b_calculateMetrics->setEnabled(false);
        pccset->ui->m_calculateMetrics->setEnabled(false);
        pccset->ui->m_errorMapClassification->setEnabled(false);
    }
}

string EvaluateOperations::to_string_with_format(string a_value, double value)
{
    ostringstream out;
    out << setw(10) << left << a_value << ":" << setw(6)<< setprecision(4) << value << endl;
    return out.str();
}
