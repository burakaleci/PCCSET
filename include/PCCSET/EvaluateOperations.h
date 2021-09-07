
#ifndef EVALUATEOPERATIONS_H
#define EVALUATEOPERATIONS_H

// Standard
#include <iostream>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// User defined
#include "PCCSET.h"
#include "ui_PCCSET.h"

using namespace std;

// Forward declaration
class PCCSET;

class EvaluateOperations
{
   
public:

    // Constructor and Destructor
    EvaluateOperations(PCCSET *);
    ~EvaluateOperations(void);
    
    // Public functions for evaluate operations
    void calculateMetrics(void);
    void calculateClassicationMetrics(void);
    void undoClassicationMetrics(void);
    void automaticPair(void);
    void findPairSegments(void);
    void undoAutomaticPair(void);
    void manualPair(void);
    bool PointPicking(const pcl::visualization::PointPickingEvent&,pcl::visualization::PCLVisualizer::Ptr);
    void pairSegments(int,int);
    void undoManualPair(int);
    void errorMap(void);
    void drawErrorMap(void);
    void undoErrorMap(void);

    vector <uint32_t> sColorVector;
   
private:

    // Utility functions for evaluate operations
    void initializeOperation(void);
    void calculateClassMetrics(int,int);
    void calculateOverallMetrics(void);
    void printClassInfo(int,int);
    void printOverallInfo(void);
    void colorizeSampleWithGTColors(void);
    void updateMenusAndButtons(string,bool);
    void clearMetrics(void);
    bool checkSegmentedLabels(string, int);
    bool isSegmentsMatched(int,int);
    void processSegment(int,int);
    void colorizeSegmentWithGTColor(int,int);
    void makeDecision(void);
    string to_string_with_format(string, double);

    // Private data attributes for evaluate operations
    PCCSET *pccset;
    
    stack < vector <uint32_t> > sampleColorVectorStack;
    int NOM;
    int NOManual;
    vector <int> segmentedGt;
    vector <int> segmentedSample;
    bool flagGTTaken;
    bool flagSampleTaken;
    int gtPosition;
    int samplePosition;
};

#endif // EVALUATEOPERATIONS_H
