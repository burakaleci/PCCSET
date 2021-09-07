#include "SnapshotDialog.h"
#include "ui_SnapshotDialog.h"

SnapshotDialog::SnapshotDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SnapshotDialog)
{
    ui->setupUi(this);
}

SnapshotDialog::~SnapshotDialog()
{
    delete ui;
}
