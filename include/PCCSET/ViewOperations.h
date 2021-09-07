
#ifndef VIEWOPERATIONS_H
#define VIEWOPERATIONS_H

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

class ViewOperations
{
   
public:

    // Constructor and Destructor
    ViewOperations(PCCSET *);
    ~ViewOperations(void);
    
    // Public functions for view operations
    void resetPointSize(void);
    void increasePointSize(void);
    void decreasePointSize(void);
   
private:

    // Private data attributes for view operations
    int pointSize;
    PCCSET *pccset;
};

#endif // VIEWOPERATIONS_H
