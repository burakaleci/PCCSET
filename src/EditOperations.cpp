
// Standard
#include <iostream>
#include <string>
#include <memory>
#include <stack>

// QT
#include <QMessageBox>

// User defined
#include "EditOperations.h"

using namespace std;

EditOperations::EditOperations(PCCSET *p)
    : pccset(p)
{
   // Empty constructor   
}

EditOperations::~EditOperations(void)
{
    
}

void EditOperations::gtClear(void)
{
    pccset->gtScreen->removeAllPointClouds();
    pccset->ui->widget_gt->update();
    pccset->ui->b_sampleOpen->setEnabled(false);
    pccset->ui->m_sampleOpen->setEnabled(false);
    pccset->ui->b_gtSave->setEnabled(false);
    pccset->ui->m_gtSave->setEnabled(false);
    pccset->ui->b_gtSaveAs->setEnabled(false);
    pccset->ui->m_gtSaveAs->setEnabled(false);
    pccset->ui->b_gtExportAs->setEnabled(false);
    pccset->ui->m_gtExportAs->setEnabled(false);
    
    pccset->ui->b_gtClear->setEnabled(false);
    pccset->ui->m_gtClear->setEnabled(false);
    pccset->ui->b_gtUndo->setEnabled(false);
    pccset->ui->m_gtUndo->setEnabled(false);
    pccset->ui->b_gtRedo->setEnabled(false);
    pccset->ui->m_gtRedo->setEnabled(false);
    
    pccset->ui->m_increasePointSize->setEnabled(false);
    pccset->ui->m_decreasePointSize->setEnabled(false);
    pccset->ui->m_resetPointSize->setEnabled(false);

    pccset->ui->b_gtColorize->setEnabled(false);
    pccset->ui->m_gtColorize->setEnabled(false);

    pccset->ui->infoTextEdit->clear();

    pccset->gtScreen->setBackgroundColor(0.0,0.0,0.0);
    pccset->ui->widget_gt->update();
    
}

void EditOperations::sampleClear(void)
{
    QMessageBox::StandardButton reply= QMessageBox::information(pccset, "Important!", "Do you want to save changes you made?",QMessageBox::Ok|QMessageBox::Cancel);
        
    if(reply == QMessageBox::Ok)
        pccset->sampleSaveAsClicked();

    pccset->sampleScreen->removeAllPointClouds();
    pccset->ui->widget_sample->update();

    pccset->ui->b_sampleSave->setEnabled(false);
    pccset->ui->m_sampleSave->setEnabled(false);
    pccset->ui->b_sampleSaveAs->setEnabled(false);
    pccset->ui->m_sampleSaveAs->setEnabled(false);
    pccset->ui->b_sampleExportAs->setEnabled(false);
    pccset->ui->m_sampleExportAs->setEnabled(false);

    pccset->ui->b_sampleClear->setEnabled(false);
    pccset->ui->m_sampleClear->setEnabled(false);
    pccset->ui->b_sampleUndo->setEnabled(false);
    pccset->ui->m_sampleUndo->setEnabled(false);
    pccset->ui->b_sampleRedo->setEnabled(false);
    pccset->ui->m_sampleRedo->setEnabled(false);

    pccset->ui->b_excel->setEnabled(false);
    pccset->ui->m_excel->setEnabled(false);
    pccset->ui->b_background->setEnabled(false);
    pccset->ui->m_background->setEnabled(false);
    pccset->ui->b_snapshot->setEnabled(false);
    pccset->ui->m_snapshot->setEnabled(false);
    pccset->ui->b_synchroniser->setEnabled(false);
    pccset->ui->m_synchroniser->setEnabled(false);
    pccset->ui->b_sampleColorize->setEnabled(false);
    pccset->ui->m_sampleColorize->setEnabled(false);

    pccset->ui->b_calculateMetrics->setEnabled(false);
    pccset->ui->m_calculateMetrics->setEnabled(false);  
    pccset->ui->b_automaticPair->setEnabled(false);
    pccset->ui->m_automaticPair->setEnabled(false);
    pccset->ui->b_manualPair->setEnabled(false);
    pccset->ui->m_manualPair->setEnabled(false);
    pccset->ui->m_errorMapClassification->setEnabled(false);
    pccset->ui->m_errorMapSegmentation->setEnabled(false);
    pccset->ui->b_errorMap->setEnabled(false);

    pccset->ui->infoTextEdit->clear();

    pccset->sampleScreen->setBackgroundColor(0.0,0.0,0.0);
    pccset->ui->widget_sample->update();
}

void EditOperations::gtUndo(void)
{
    pccset->gtEO->undo();
    pccset->ui->b_gtRedo->setEnabled(true);
    pccset->ui->m_gtRedo->setEnabled(true);
    if (pccset->gtEO->getUndoStackSize()==0){
        pccset->ui->b_gtUndo->setEnabled(false);
        pccset->ui->m_gtUndo->setEnabled(false);
        pccset->ui->b_gtRedo->setEnabled(false);
        pccset->ui->m_gtRedo->setEnabled(false);
    }
}

void EditOperations::gtRedo(void)
{
    pccset->gtEO->redo();
    if (pccset->gtEO->getRedoStackSize()==0){
        pccset->ui->b_gtRedo->setEnabled(false);
        pccset->ui->m_gtRedo->setEnabled(false);
    }  
}

void EditOperations::sampleUndo(void)
{
    pccset->sampleEO->undo();
    pccset->ui->b_sampleRedo->setEnabled(true);
    pccset->ui->m_sampleRedo->setEnabled(true);
    if (pccset->sampleEO->getUndoStackSize()==0){
        pccset->ui->b_sampleUndo->setEnabled(false);
        pccset->ui->m_sampleUndo->setEnabled(false);
        pccset->ui->b_sampleRedo->setEnabled(false);
        pccset->ui->m_sampleRedo->setEnabled(false);
    }
}

void EditOperations::sampleRedo(void)
{
    pccset->sampleEO->redo();
    if (pccset->sampleEO->getRedoStackSize()==0){
        pccset->ui->b_sampleRedo->setEnabled(false);
        pccset->ui->m_sampleRedo->setEnabled(false);
    }
}

void EditOperations::executeCmd(shared_ptr<UndoRedoBase> command) 
{
    redoStack = UndoRedoOperationStack(); // clear the redo stack
    command->execute();
    undoStack.push(command);
}

void EditOperations::undo(void)
{
    if (getUndoStackSize() <= 0) 
        return;
    undoStack.top()->undo();          // undo most recently executed command
    redoStack.push(undoStack.top()); // add undone command to undo stack
    undoStack.pop();                  // remove top entry from undo stack

}
  
void EditOperations::redo(void) 
{
    if (getRedoStackSize() <= 0) 
        return;
        
    redoStack.top()->redo();          // redo most recently executed command
    undoStack.push(redoStack.top()); // add undone command to redo stack
    redoStack.pop();                  // remove top entry from redo stack
}

int EditOperations::getUndoStackSize(void)
{
    return undoStack.size();
}

int EditOperations::getRedoStackSize(void)
{
    return redoStack.size();
}

BackgroundAction::BackgroundAction(PCCSET *p, QColor pre, QColor cur)
    :pccset(p),
    previous(pre),
    current(cur)
{
    // Empty constructor
}

BackgroundAction::~BackgroundAction(void)
{
      
}
    
void BackgroundAction::execute(void)    
{
    pccset->tO->background(current);  
}

void BackgroundAction::undo(void)       
{   
    pccset->tO->background(previous);
}

void BackgroundAction::redo(void)       
{ 
    pccset->tO->background(current); 
}


ColorizeAction::ColorizeAction(PCCSET *p, int pos, uint32_t pre, uint32_t cur, string s)
    :pccset(p),
    position(pos),
    previous(pre),
    current(cur),
    source(s)
{
    // Empty constructor
}

ColorizeAction::~ColorizeAction(void)
{
     
}
    
void ColorizeAction::execute(void)    
{
    pccset->tO->colorize(position,current,source);  
}

void ColorizeAction::undo(void)       
{   
    pccset->tO->colorize(position,previous,source);
}

void ColorizeAction::redo(void)       
{ 
    pccset->tO->colorize(position,current,source); 
}


CalculateMetricsAction::CalculateMetricsAction(PCCSET *p)
    :pccset(p)
{
    // Empty constructor
}

CalculateMetricsAction::~CalculateMetricsAction(void)
{
      
}
    
void CalculateMetricsAction::execute(void)    
{
    pccset->eO->calculateClassicationMetrics();
}

void CalculateMetricsAction::undo(void)       
{   
    pccset->eO->undoClassicationMetrics();
}

void CalculateMetricsAction::redo(void)       
{ 
    pccset->eO->calculateClassicationMetrics();
}

AutomaticPairAction::AutomaticPairAction(PCCSET *p)
    :pccset(p)
{
    // Empty constructor
}

AutomaticPairAction::~AutomaticPairAction(void)
{
     
}

void AutomaticPairAction::execute(void)    
{
    pccset->eO->findPairSegments();
}

void AutomaticPairAction::undo(void)       
{   
    pccset->eO->undoAutomaticPair();
}
    
void AutomaticPairAction::redo(void)       
{ 
    pccset->eO->findPairSegments();
}

ManualPairAction::ManualPairAction(PCCSET *p, int gP, int sP)
    :pccset(p),
    gtPosition(gP),
    samplePosition(sP)
{
    // Empty constructor
}

ManualPairAction::~ManualPairAction(void)
{
     
}

void ManualPairAction::execute(void)    
{
    pccset->eO->pairSegments(gtPosition,samplePosition);
}

void ManualPairAction::undo(void)       
{   
    pccset->eO->undoManualPair(samplePosition);
}
    
void ManualPairAction::redo(void)       
{ 
    pccset->eO->pairSegments(gtPosition,samplePosition);
}

ErrorMapAction::ErrorMapAction(PCCSET *p)
    :pccset(p)
    
{
    // Empty constructor
}

ErrorMapAction::~ErrorMapAction(void)
{
     
}

void ErrorMapAction::execute(void)    
{
    pccset->eO->drawErrorMap();
}

void ErrorMapAction::undo(void)       
{   
    pccset->eO->undoErrorMap();
}
    
void ErrorMapAction::redo(void)       
{ 
    pccset->eO->drawErrorMap();
}


