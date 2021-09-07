#ifndef SNAPSHOTDIALOG_H
#define SNAPSHOTDIALOG_H

#include <QDialog>

namespace Ui {class SnapshotDialog;}

class SnapshotDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SnapshotDialog(QWidget *parent = 0);
    ~SnapshotDialog();
    Ui::SnapshotDialog *ui;   
};

#endif // SNAPSHOTDIALOG_H
