#ifndef EDITOPERATIONS_H
#define EDITOPERATIONS_H

// Standard
#include <iostream>
#include <memory>
#include <stack>

// User defined
#include "PCCSET.h"
#include "ui_PCCSET.h"

using namespace std;

// Forward declaration
class PCCSET;

// ----- the Undo Redo Base Class -----
class UndoRedoBase 
{

public:

    virtual void execute(void) = 0;
    virtual void undo(void) = 0;
    virtual void redo(void) = 0;
};

// ----- our CONTROLLER with undo/redo -----
typedef stack< shared_ptr<UndoRedoBase> > UndoRedoOperationStack;

class EditOperations 
{
  
public:
  
    EditOperations(PCCSET *);
    ~EditOperations(void);

    void gtClear(void);
    void sampleClear(void);

    void gtUndo(void);
    void gtRedo(void);
    void sampleUndo(void);
    void sampleRedo(void);

    void executeCmd(shared_ptr<UndoRedoBase>); 
    void undo(void);
    void redo(void); 

    int getUndoStackSize(void);
    int getRedoStackSize(void);
    
private:

    PCCSET *pccset;
    UndoRedoOperationStack undoStack;
    UndoRedoOperationStack redoStack;

};


// ----- concrete Undo Redo operations -----
class BackgroundAction : public UndoRedoBase 
{
    
public:

    BackgroundAction(PCCSET *,QColor,QColor);
    ~BackgroundAction(void);

    void execute(void);    
    void undo(void);       
    void redo(void);       

private:

    PCCSET *pccset;
    QColor previous;
    QColor current;

};


class ColorizeAction : public UndoRedoBase 
{
    
public:

    ColorizeAction(PCCSET *,int,uint32_t,uint32_t,string);
    ~ColorizeAction(void);

    void execute(void);    
    void undo(void);       
    void redo(void);       

private:

    PCCSET *pccset;
    int position;
    uint32_t previous;
    uint32_t current;
    string source;

};

class CalculateMetricsAction : public UndoRedoBase 
{
    
public:

    CalculateMetricsAction(PCCSET *);
    ~CalculateMetricsAction(void);

    void execute(void);    
    void undo(void);       
    void redo(void);

private:

    PCCSET *pccset;

};

class AutomaticPairAction : public UndoRedoBase 
{
    
public:

    AutomaticPairAction(PCCSET *);
    ~AutomaticPairAction(void);

    void execute(void);    
    void undo(void);       
    void redo(void);

private:

    PCCSET *pccset;
    
};

class ManualPairAction : public UndoRedoBase 
{

public:

    ManualPairAction(PCCSET *,int,int);
    ~ManualPairAction(void);

    void execute(void);    
    void undo(void);       
    void redo(void);

private:

    PCCSET *pccset;
    int gtPosition;
    int samplePosition;
};

class ErrorMapAction : public UndoRedoBase 
{

public:

    ErrorMapAction(PCCSET *);
    ~ErrorMapAction(void);

    void execute(void);    
    void undo(void);       
    void redo(void);

private:

    PCCSET *pccset;
    
};


#endif // EDITOPERATIONS_H
