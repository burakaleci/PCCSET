#ifndef EXCELDIALOG_H
#define EXCELDIALOG_H

#include <QDialog>

namespace Ui {class ExcelDialog;}

class ExcelDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ExcelDialog(QWidget *parent = 0);
    ~ExcelDialog();
    Ui::ExcelDialog *ui;
    
};

#endif // EXCELDIALOG_H

